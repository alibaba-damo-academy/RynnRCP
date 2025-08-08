/**
 * @file auth_manager.cpp
 * @brief Implementation of the CAuthManager class
 */

#include "auth_manager.hpp"

namespace rynnrcp { namespace fw { namespace common {

CAuthManager::CAuthManager(const std::string &config_filename) {
  loadConfig(config_filename);

  std::map<std::string, std::string> ext_args = {
      {"productKey", _productKey},
      {"deviceName", _deviceName},
  };

  std::string ts = std::to_string(getTimeMs());
  std::string nonce = std::to_string(getNonce());
  std::string sign =
      genSignature(_productKey, ts, nonce, ext_args, _deviceSecret);

  _payload = {{"productKey", _productKey},
              {"deviceName", _deviceName},
              {"auth",
               {{"authKey", _productKey},
                {"nonce", nonce},
                {"timestamp", ts},
                {"sign", sign}}}};
}

void CAuthManager::loadConfig(const std::string &config_filename) {
  try {
    YAML::Node config = YAML::LoadFile(config_filename);

    _httpUrl = config["http_url"].as<std::string>();
    _productKey = config["product_key"].as<std::string>();
    _deviceName = config["device_name"].as<std::string>();
    _deviceSecret = config["device_secret"].as<std::string>();
    _endpointMqtt = config["endpoint_mqtt"].as<std::string>();
    _endpointWebsocket = config["endpoint_websocket"].as<std::string>();

  } catch (const YAML::Exception &e) {
    throw std::runtime_error("Failed to load config file: "
                             + std::string(e.what()));
  }
}

std::string CAuthManager::genSignContent(
    const std::string &product_key, const std::string &timestamp,
    const std::string &nonce,
    const std::map<std::string, std::string> &ext_args) {
  std::vector<std::pair<std::string, std::string>> wait_sign_params;

  wait_sign_params.emplace_back("nonce", nonce);
  wait_sign_params.emplace_back("authKey", product_key);
  wait_sign_params.emplace_back("timestamp", timestamp);

  for (const auto &entry : ext_args) {
    wait_sign_params.emplace_back(entry.first, entry.second);
  }

  sort(wait_sign_params.begin(), wait_sign_params.end(),
       [](const std::pair<std::string, std::string> &a,
          const std::pair<std::string, std::string> &b) {
         if (a.first == b.first) return a.second < b.second;
         return a.first < b.first;
       });

  std::stringstream cb;
  for (size_t i = 0; i < wait_sign_params.size(); ++i) {
    if (i != 0) cb << ";";
    cb << wait_sign_params[i].first << "=" << wait_sign_params[i].second;
  }
  return cb.str();
}

std::string CAuthManager::hmacSha1(const std::string &content,
                                   const std::string &secret) {
  unsigned int digest_len;
  unsigned char digest[EVP_MAX_MD_SIZE];

  HMAC(EVP_sha1(), secret.data(), secret.length(),
       reinterpret_cast<const unsigned char *>(content.data()),
       content.length(), digest, &digest_len);

  BIO *b64 = BIO_new(BIO_f_base64());
  BIO_set_flags(b64, BIO_FLAGS_BASE64_NO_NL);
  BIO *bio = BIO_new(BIO_s_mem()); // free with b64
  BIO_push(b64, bio);

  BIO_write(b64, digest, digest_len);
  BIO_flush(b64);

  BUF_MEM *buffer_ptr;
  BIO_get_mem_ptr(b64, &buffer_ptr);

  std::string result(buffer_ptr->data, buffer_ptr->length);
  BIO_free_all(b64);

  return result;
}

std::string CAuthManager::genSignature(
    const std::string &product_key, const std::string &timestamp,
    const std::string &nonce,
    const std::map<std::string, std::string> &ext_args,
    const std::string &secret) {
  std::string content = genSignContent(product_key, timestamp, nonce, ext_args);
  return hmacSha1(content, secret);
}

Json CAuthManager::getMqttConfig() {
  httplib::Client cli(_httpUrl.c_str());

#ifdef CPPHTTPLIB_OPENSSL_SUPPORT
  cli.enable_server_certificate_verification(false);
#endif

  auto res_mqtt =
      cli.Post(_endpointMqtt.c_str(), _payload.dump(), "application/json");

  if (res_mqtt == nullptr) {
    throw std::runtime_error("Request token failed, got NULL, recheck config");
  }

  if (res_mqtt->status == httplib::OK_200) {
    try {
      auto payload = Json::parse(res_mqtt->body);
      LOG(INFO) << "[AUTH][HTTP]: MQTT Response: " << payload.dump(4);

      _clientId = payload["data"]["clientId"];
      _mqttHost = payload["data"]["endpoint"];
      _deviceAk = payload["data"]["accessKeyId"];
      _deviceSk = payload["data"]["accessKeySecret"];
      _instanceId = payload["data"]["instanceId"];
      _mqttPort = kMqttPort;

    } catch (const std::exception &e) {
      throw std::runtime_error("MQTT response body parse failed: "
                               + std::string(e.what()));
    }
  } else {
    throw std::runtime_error("Request mqtt token failed with status: "
                             + std::to_string(res_mqtt->status));
  }

  return {
      {"product_key", _productKey}, {"device_name", _deviceName},
      {"device_ak", _deviceAk},     {"device_sk", _deviceSk},
      {"client_id", _clientId},     {"mqtt_host", _mqttHost},
      {"instance_id", _instanceId}, {"mqtt_port", _mqttPort},
  };
}

Json CAuthManager::getWsConfig() {
  httplib::Client cli(_httpUrl.c_str());

#ifdef CPPHTTPLIB_OPENSSL_SUPPORT
  cli.enable_server_certificate_verification(false);
#endif

  auto res_ws =
      cli.Post(_endpointWebsocket.c_str(), _payload.dump(), "application/json");

  if (res_ws == nullptr) {
    throw std::runtime_error("Request token failed, got NULL, recheck config");
  }

  if (res_ws->status == httplib::OK_200) {
    try {
      auto payload = Json::parse(res_ws->body);
      LOG(INFO) << "[AUTH][HTTP]: WS Response: " << payload.dump(4);

      _expire = payload["data"]["expire"];
      _token = payload["data"]["token"];
      _websocketUrl = payload["data"]["uri"];

      char websocket_host[kMaxUrlPartLength + 1];
      char websocket_path[kMaxUrlPartLength + 1];
      memset(websocket_host, 0, sizeof(websocket_host));
      memset(websocket_path, 0, sizeof(websocket_path));

      std::string url_without_protocol;
      if (strncmp(_websocketUrl.data(), "wss://", strlen("wss://")) == 0) {
        url_without_protocol = _websocketUrl.substr(strlen("wss://"));
      } else if (strncmp(_websocketUrl.data(), "ws://", strlen("ws://")) == 0) {
        url_without_protocol = _websocketUrl.substr(strlen("ws://"));
      } else {
        throw std::runtime_error("URL must start with 'ws://' or 'wss://'");
      }

      // Find the first '/' character to split host and path
      size_t slash_pos = url_without_protocol.find('/');
      std::string host_part, path_part;

      if (slash_pos != std::string::npos) {
        host_part = url_without_protocol.substr(0, slash_pos);
        path_part = url_without_protocol.substr(slash_pos);
      } else {
        host_part = url_without_protocol;
        path_part = "/";
      }

      if (host_part.length() > kMaxUrlPartLength) {
        throw std::runtime_error("WebSocket host part too long, max length is "
                                 + std::to_string(kMaxUrlPartLength));
      }

      if (path_part.length() > kMaxUrlPartLength) {
        throw std::runtime_error("WebSocket path part too long max length is "
                                 + std::to_string(kMaxUrlPartLength));
      }

      strncpy(websocket_host, host_part.c_str(), host_part.length());
      strncpy(websocket_path, path_part.c_str(), path_part.length());

      _websocketHost = websocket_host;
      _websocketPath = websocket_path;
      _websocketPort = kWebsocketPort;

    } catch (const std::exception &e) {
      throw std::runtime_error("WebSocket response body parse failed: "
                               + std::string(e.what()));
    }
  } else {
    throw std::runtime_error("Request WebSocket token failed with status: "
                             + std::to_string(res_ws->status));
  }

  return {{"token", _token},
          {"expire", _expire},
          {"websocket_host", _websocketHost},
          {"websocket_path", _websocketPath},
          {"websocket_port", _websocketPort}};
}

}}} // namespace rynnrcp::fw::common

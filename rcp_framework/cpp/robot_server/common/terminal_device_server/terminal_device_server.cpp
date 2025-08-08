/**
 * @file terminal_device_server.cpp
 * @brief Implementation of the CTerminalDeviceServer class
 */

#include "terminal_device_server.hpp"

namespace rynnrcp { namespace fw { namespace robot {

CTerminalDeviceServer::CTerminalDeviceServer(
    const std::string &name, const std::shared_ptr<CJsonRpcComm> jsonrpc_client,
    const std::shared_ptr<CDataComm> data_client) :
    CServer(name, jsonrpc_client, data_client) {
}

CTerminalDeviceServer::~CTerminalDeviceServer() {
}

int32_t CTerminalDeviceServer::handleMessage(MessagePointer msg) {
  auto protocol_type = msg->getProtocolType();
  switch (protocol_type) {
  case rynnrcp::fw::common::ProtocolType::COMM_JSONRPC: {
    auto jsonrpc_msg = std::dynamic_pointer_cast<CJsonRpcMessage>(msg);
    return processJsonRpcMsg(jsonrpc_msg);
  }
  case rynnrcp::fw::common::ProtocolType::COMM_DATA: {
    auto ws_msg = std::dynamic_pointer_cast<CDataMessage>(msg);
    return processDataMsg(ws_msg);
  }
  default: return -1;
  }
}

// Implemented in CSensorServer and CActionServer class
int32_t CTerminalDeviceServer::processDataMsg(
    const std::shared_ptr<CDataMessage> msg) {
  return 0;
}

inline std::string constructSendTopic(const std::string &topic) {
  if (topic.empty()) { return ""; }

  std::istringstream iss(topic);
  std::string item;
  std::vector<std::string> parts;

  while (std::getline(iss, item, '/')) {
    if (!item.empty()) { parts.push_back(item); }
  }

  if (parts.size() < 7) { return ""; }

  std::string send_topic =
      parts[3] + "/response/" + parts[5] + "/" + parts[6] + "/";

  return send_topic;
}

inline bool endsWith(const std::string &str, const std::string &suffix) {
  return str.length() >= suffix.length()
         && str.compare(str.length() - suffix.length(), suffix.length(), suffix)
                == 0;
}

int32_t CTerminalDeviceServer::acquireDevice(
    const std::shared_ptr<CJsonRpcMessage> msg) {
  std::string send_payload;
  int32_t send_msgid = msg->getMsgid();
  std::string send_topic = constructSendTopic(msg->getTopic());

  try {
    Json request_json;
    request_json = Json::parse(msg->getPayload());

    Json response_json;
    response_json["jsonrpc"] = "2.0";
    response_json["id"] = request_json["id"].get<std::string>();
    response_json["result"]["$TIME"] = getTimeMs();

    std::unique_lock<std::mutex> lk(_mutex);

    if (_occupied.load(std::memory_order_acquire)) {
      response_json["error"]["code"] = kDeviceBusyCode;
      response_json["error"]["message"] = "Device is busy";
      LOG(INFO) << "[" << _serverName
                << "][JSONRPC]: Occupy failed: Device is already occupied.";
    } else {
      _occupied.store(true, std::memory_order_release);
      LOG(INFO) << "[" << _serverName
                << "][JSONRPC]: Occupy success: Device is now occupied.";
    }

    send_payload = response_json.dump();

  } catch (const std::exception &e) {
    LOG(ERROR) << "[" << _serverName
               << "][JSONRPC]: Failed to parse JSON request: " << e.what();

    Json error_response;
    error_response["jsonrpc"] = "2.0";
    error_response["id"] = "-1";
    error_response["error"]["code"] = kJsonParseErrorCode;
    error_response["error"]["message"] =
        "Failed to parse JSON request: " + std::string(e.what());

    send_payload = error_response.dump();
  }

  MessagePointer protocol_msg =
      std::make_shared<CJsonRpcMessage>(send_topic, send_msgid, send_payload);

  _jsonrpcClient->send(protocol_msg);
  LOG(INFO) << "[" << _serverName << "][JSONRPC]: Send response";

  return 0;
}

int32_t CTerminalDeviceServer::releaseDevice(
    const std::shared_ptr<CJsonRpcMessage> msg) {
  std::string send_payload;
  int32_t send_msgid = msg->getMsgid();
  std::string send_topic = constructSendTopic(msg->getTopic());

  try {
    Json request_json;
    request_json = Json::parse(msg->getPayload());

    Json response_json;
    response_json["jsonrpc"] = "2.0";
    response_json["id"] = request_json["id"].get<std::string>();
    response_json["result"]["$TIME"] = getTimeMs();

    std::unique_lock<std::mutex> lk(_mutex);

    if (!_occupied.load(std::memory_order_acquire)) {
      response_json["error"]["code"] = kDeviceNotOccupiedCode;
      response_json["error"]["message"] = "No need to release";
      LOG(INFO) << "[" << _serverName
                << "][JSONRPC]: Release failed: Device is not occupied.";
    } else {
      _occupied.store(false, std::memory_order_release);
      LOG(INFO) << "[" << _serverName << "][JSONRPC]: Release success.";
    }

    send_payload = response_json.dump();

  } catch (const std::exception &e) {
    Json error_response;
    error_response["jsonrpc"] = "2.0";
    error_response["id"] = "-1";
    error_response["error"]["code"] = kJsonParseErrorCode;
    error_response["error"]["message"] =
        "Failed to parse JSON request: " + std::string(e.what());

    send_payload = error_response.dump();

    LOG(ERROR) << "[" << _serverName
               << "][JSONRPC]: Failed to parse JSON request: " << e.what();
  }

  MessagePointer protocol_msg =
      std::make_shared<CJsonRpcMessage>(send_topic, send_msgid, send_payload);
  _jsonrpcClient->send(protocol_msg);

  LOG(INFO) << "[" << _serverName << "][JSONRPC]: Send response";
  return 0;
}

int32_t CTerminalDeviceServer::processJsonRpcMsg(
    const std::shared_ptr<CJsonRpcMessage> msg) {
  LOG(INFO) << "[" << _serverName
            << "][JSONRPC]: Received topic: " << msg->getTopic()
            << " payload: " << msg->getPayload();

  std::string recv_topic = msg->getTopic();

  if (endsWith(recv_topic, "acquire_device")) {
    acquireDevice(msg);
  } else if (endsWith(recv_topic, "release_device")) {
    releaseDevice(msg);
  } else {
    // TODO: response other topics
  }

  return 0;
}
}}} // namespace rynnrcp::fw::robot
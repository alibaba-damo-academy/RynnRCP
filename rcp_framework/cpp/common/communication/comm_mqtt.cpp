/**
 * @file comm_mqtt.cpp
 * @brief Implementation of CMqttComm class
 */

#include "comm_mqtt.hpp"

namespace rynnrcp { namespace fw { namespace common {

CMqttComm::CMqttComm() {
}

CMqttComm::~CMqttComm() {
  close();
}

void CMqttComm::init(const std::string &config_file) {
  std::unique_lock<std::mutex> lk(_mutex);

  _authClient = std::make_shared<CAuthManager>(config_file);

  Json config = _authClient->getMqttConfig();
  try {
    _productKey = config["product_key"].get<std::string>();
    _deviceName = config["device_name"].get<std::string>();

    _deviceAk = config["device_ak"].get<std::string>();
    _deviceSk = config["device_sk"].get<std::string>();

    _mqttPort = config["mqtt_port"].get<int32_t>();
    _mqttHost = config["mqtt_host"].get<std::string>();
    _clientId = config["client_id"].get<std::string>();
    _instanceId = config["instance_id"].get<std::string>();

  } catch (const std::exception &e) {
    throw std::runtime_error("Parse config failed: " + std::string(e.what()));
  }

  // Generate client ID, username, and password based on Aliyun example
  unsigned char temp_data[kPasswordMaxLen];
  unsigned int len = 0;
  HMAC(EVP_sha1(), _deviceSk.c_str(), _deviceSk.length(),
       (const unsigned char *)_clientId.data(), _clientId.size(), temp_data,
       &len);

  char result_data[kPasswordMaxLen] = {0};
  int32_t password_len =
      EVP_EncodeBlock((unsigned char *)result_data, temp_data, len);
  result_data[password_len] = '\0';

  char user_name_data[kUsernameMaxLen] = {0};
  int32_t username_len =
      snprintf(user_name_data, sizeof(user_name_data), "DeviceCredential|%s|%s",
               _deviceAk.c_str(), _instanceId.c_str());
  if (username_len < 0 || username_len >= sizeof(user_name_data)) {
    throw std::runtime_error("Username too long for buffer, max length: "
                             + std::to_string(kUsernameMaxLen));
  }

  char url[kUrlMaxLen] = {0};
  int32_t url_len =
      snprintf(url, sizeof(url), "tcp://%s:%d", _mqttHost.c_str(), _mqttPort);
  if (url_len < 0 || url_len >= sizeof(url)) {
    throw std::runtime_error("MQTT URL too long for buffer, max length: "
                             + std::to_string(kUrlMaxLen));
  }

  _clientUrl = url;
  _clientUsername = user_name_data;
  _clientPassword = result_data;

  _connected.store(false, std::memory_order_release);

  // Set topics
  _topicPrefix = "sys/" + _productKey + "/" + _deviceName + "/";
  _subTopics.push_back(_topicPrefix + "rrpc/request/+/acquire_device");
  _subTopics.push_back(_topicPrefix + "rrpc/request/+/release_device");
}

void CMqttComm::onConnectLost(void *context, char *cause) {
  LOG(INFO) << "[MQTT]: Connection lost, cause: "
            << (cause ? cause : "Unknown reason");
  CMqttComm *me = reinterpret_cast<CMqttComm *>(context);

  std::unique_lock<std::mutex> lk(me->_mutex);
  me->_connected.store(false, std::memory_order_release);
}

void CMqttComm::onConnectSucc(void *context, MQTTAsync_successData *response) {
}

void CMqttComm::onConnectFail(void *context, MQTTAsync_failureData *response) {
  LOG(INFO) << "[MQTT]: Connection failed, cause: "
            << (response ? response->message : "Unknown error");
  CMqttComm *me = reinterpret_cast<CMqttComm *>(context);

  std::unique_lock<std::mutex> lk(me->_mutex);
  me->_connected.store(false, std::memory_order_release);
}

void CMqttComm::onConnectEstablished(void *context, char *cause) {
  CMqttComm *me = reinterpret_cast<CMqttComm *>(context);
  std::unique_lock<std::mutex> lk(me->_mutex);

  // Subscribe to topics
  for (auto &topic : me->_subTopics) {
    MQTTAsync_responseOptions sub_opts = MQTTAsync_responseOptions_initializer;
    sub_opts.onSuccess = onSubscribeSucc;
    sub_opts.onFailure = onSubscribeFail;

    int rc;
    LOG(INFO) << "[MQTT]: Subscribing to topic: " << topic
              << ", QoS: " << MQTT_QOS_1;
    if ((rc = MQTTAsync_subscribe(me->_client, topic.c_str(), int(MQTT_QOS_1),
                                  &sub_opts))
        != MQTTASYNC_SUCCESS) {
      throw std::runtime_error("[MQTT]: Async subscribe to topic failed: "
                               + std::to_string(rc));
    }
  }

  me->_connected.store(true, std::memory_order_release);
}

void CMqttComm::onDisconnectSucc(void *context,
                                 MQTTAsync_successData *response) {
  LOG(INFO) << "[MQTT]: Disconnection successful";
  CMqttComm *me = reinterpret_cast<CMqttComm *>(context);

  std::unique_lock<std::mutex> lk(me->_mutex);
  me->_connected.store(false, std::memory_order_release);
}

int32_t CMqttComm::onMessageArrived(void *context, char *topic, int topic_len,
                                    MQTTAsync_message *message) {
  if (!message || !context || !topic || topic_len <= 0) {
    LOG(ERROR) << "[MQTT]: Error: Invalid parameters";
    return -1;
  }

  CMqttComm *me = reinterpret_cast<CMqttComm *>(context);

  std::string topic_str;
  std::string payload_str;
  topic_str = std::string(topic, topic_len);
  payload_str = std::string((char *)(message->payload), message->payloadlen);
  LOG(INFO) << "[MQTT]: Message arrived - topic: '" << topic_str
            << "', message id: " << message->msgid;

  MessagePointer tmp_msg =
      std::make_shared<CJsonRpcMessage>(topic_str, message->msgid, payload_str);

  for (auto &recv_buf : me->_recvBuffer) { recv_buf->enqueue(tmp_msg); }

  MQTTAsync_freeMessage(&message);
  MQTTAsync_free(topic);

  return 1;
}

void CMqttComm::onSubscribeSucc(void *context,
                                MQTTAsync_successData *response) {
}

void CMqttComm::onSubscribeFail(void *context,
                                MQTTAsync_failureData *response) {
  throw std::runtime_error("[MQTT]: Subscription failed with code: "
                           + std::to_string(response->code));
}

void CMqttComm::onPublishSucc(void *context, MQTTAsync_successData *response) {
  if (response) {
    LOG(INFO) << "[MQTT]: Publication successful - Topic: "
              << response->alt.pub.destinationName
              << ", Message ID: " << response->alt.pub.message.msgid;
  }
}

void CMqttComm::onPublishFail(void *context, MQTTAsync_failureData *response) {
  if (response) {
    LOG(INFO) << "[MQTT]: Publication failed - Code: " << response->code
              << ", Message: " << response->message;
  }
}

void CMqttComm::start() {
  std::unique_lock<std::mutex> lk(_mutex);

  MQTTAsync_createOptions create_opts = MQTTAsync_createOptions_initializer;
  create_opts.sendWhileDisconnected = 0;
  create_opts.maxBufferedMessages = kMaxMsgBufSize;
  LOG(INFO) << "[MQTT]: Client URL: " << _clientUrl
            << " | Client ID: " << _clientId;

  int rc;
  if ((rc = MQTTAsync_createWithOptions(
           &_client, _clientUrl.c_str(), _clientId.c_str(),
           MQTTCLIENT_PERSISTENCE_NONE, NULL, &create_opts))
      != MQTTCLIENT_SUCCESS) {
    throw std::runtime_error("Failed to create client, return code "
                             + std::to_string(rc));
  }

  if ((rc = MQTTAsync_setCallbacks(_client, this, &onConnectLost,
                                   &onMessageArrived, NULL))
      != MQTTCLIENT_SUCCESS) {
    throw std::runtime_error("Failed to set callbacks, return code "
                             + std::to_string(rc));
  }

  MQTTAsync_connectOptions conn_opts = MQTTAsync_connectOptions_initializer;
  conn_opts.MQTTVersion = MQTTVERSION_3_1_1;
  conn_opts.cleansession = 1;
  conn_opts.connectTimeout = kMqttConnTimeout;
  conn_opts.keepAliveInterval = kMqttKeepaliveInterval;
  conn_opts.onSuccess = onConnectSucc;
  conn_opts.onFailure = onConnectFail;
  conn_opts.context = this;
  conn_opts.ssl = NULL;
  conn_opts.automaticReconnect = 1;
  conn_opts.minRetryInterval = 2;                  // in seconds
  conn_opts.maxRetryInterval = 365 * 24 * 60 * 60; // in seconds
  conn_opts.username = _clientUsername.c_str();
  conn_opts.password = _clientPassword.c_str();

  LOG(INFO) << "[MQTT]: Username: " << conn_opts.username
            << " | Password: " << conn_opts.password;

  MQTTAsync_setConnected(_client, this, onConnectEstablished);

  if ((rc = MQTTAsync_connect(_client, &conn_opts)) != MQTTASYNC_SUCCESS) {
    throw std::runtime_error("Failed to connect, return code "
                             + std::to_string(rc));
  }

  lk.unlock(); // Unlock the mutex to allow other operations

  while (!_connected.load(std::memory_order_acquire)) {
    LOG(INFO) << "[MQTT]: Waiting for connection...";
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  LOG(INFO) << "[MQTT]: Initialization successful";
}

void CMqttComm::close() {
  std::unique_lock<std::mutex> lk(_mutex);

  MQTTAsync_disconnectOptions disc_opts =
      MQTTAsync_disconnectOptions_initializer;
  disc_opts.onSuccess = onDisconnectSucc;
  disc_opts.context = this;

  int rc = 0;
  if ((rc = MQTTAsync_disconnect(_client, &disc_opts)) != MQTTASYNC_SUCCESS) {
    throw std::runtime_error("[MQTT]: Failed to disconnect, return code "
                             + std::to_string(rc));
  }

  lk.unlock();
  while (1) {
    {
      std::unique_lock<std::mutex> lk(_mutex);
      if (!_connected.load(std::memory_order_acquire)) { break; }
    }
    sleep(1);
  }

  MQTTAsync_destroy(&_client);
  _client = NULL;
}

void CMqttComm::send(MessagePointer &msg) {
  if (!_connected.load(std::memory_order_acquire)) {
    LOG(ERROR) << "[MQTT]: MQTT client is not connected";
    return;
  }

  std::unique_lock<std::mutex> lk(_mutex);

  auto mqtt_msg = std::dynamic_pointer_cast<CJsonRpcMessage>(msg);
  if (mqtt_msg->getProtocolType() != COMM_JSONRPC) {
    LOG(ERROR) << "[MQTT]: Invalid message type, expected CJsonRpcMessage";
    return;
  }

  std::string payload_send = mqtt_msg->getPayload();

  MQTTAsync_message pubmsg = MQTTAsync_message_initializer;
  pubmsg.payload = const_cast<char *>(payload_send.c_str());
  pubmsg.payloadlen = payload_send.size();
  pubmsg.qos = MQTT_QOS_1;
  pubmsg.retained = 0;

  MQTTAsync_responseOptions response_opts =
      MQTTAsync_responseOptions_initializer;
  response_opts.onSuccess = onPublishSucc;
  response_opts.onFailure = onPublishFail;

  std::string send_topic = _topicPrefix + mqtt_msg->getTopic();

  int rc = 0;
  if ((rc = MQTTAsync_sendMessage(_client, send_topic.c_str(), &pubmsg,
                                  &response_opts))
      != MQTTASYNC_SUCCESS) {
    LOG(ERROR) << "[MQTT]: Failed to send message, return code " << rc;
    return;
  }

  LOG(INFO) << "[MQTT]: Message sent, topic: " << send_topic
            << " payload: " << mqtt_msg->getPayload();
}

void CMqttComm::bindRecvBuffer(MessageQueue *recv_buffer) {
  std::unique_lock<std::mutex> lk(_mutex);
  _recvBuffer.push_back(recv_buffer);
}

}}} // namespace rynnrcp::fw::common

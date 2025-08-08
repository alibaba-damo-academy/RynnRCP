/**
 * @file comm_mqtt.hpp
 * @brief Header file for CMqttComm class
 */

#pragma once

#include <MQTTAsync.h>
#include <MQTTClient.h>
#include <openssl/bio.h>
#include <openssl/hmac.h>

#include "comm_jsonrpc.hpp"
#include "comm_type.hpp"

namespace rynnrcp { namespace fw { namespace common {

/**
 * @class CMqttComm
 * @brief MQTT communication implementation
 *
 * Implements MQTT protocol for communication between devices and servers
 */
class CMqttComm : public CJsonRpcComm {
public:
  CMqttComm();
  ~CMqttComm();

  /**
   * @brief Initialize MQTT communication with the provided configuration
   * @param config_file Configuration file path
   */
  void init(const std::string &config_file) override;

  /**
   * @brief Start the MQTT communication
   */
  void start() override;

  /**
   * @brief Close the MQTT connection
   */
  void close() override;

  /**
   * @brief Send a message through the communication channel
   * @param msg Message to be sent
   */
  void send(MessagePointer &msg) override;

  /**
   * @brief Bind a message queue to the communication channel
   * @param recv_buffer The message queue to bind
   */
  void bindRecvBuffer(MessageQueue *recv_buffer) override;

private:
  // MQTT Callbacks
  static void onConnectLost(void *context, char *cause);
  static void onConnectEstablished(void *context, char *cause);
  static void onConnectSucc(void *context, MQTTAsync_successData *response);
  static void onConnectFail(void *context, MQTTAsync_failureData *response);
  static void onDisconnectSucc(void *context, MQTTAsync_successData *response);
  static void onSubscribeSucc(void *context, MQTTAsync_successData *response);
  static void onSubscribeFail(void *context, MQTTAsync_failureData *response);
  static void onPublishSucc(void *context, MQTTAsync_successData *response);
  static void onPublishFail(void *context, MQTTAsync_failureData *response);
  static int32_t onMessageArrived(void *context, char *topic, int topic_len,
                                  MQTTAsync_message *message);

private:
  // Authentication client
  std::shared_ptr<CAuthManager> _authClient;

  // MQTT configuration parameters
  std::string _productKey; ///< Product key
  std::string _deviceName; ///< Device name
  std::string _mqttHost;   ///< MQTT broker host URL
  int32_t _mqttPort;       ///< MQTT port number
  std::string _instanceId; ///< Instance ID
  std::string _groupId;    ///< Group ID
  std::string _deviceAk;   ///< Device access key
  std::string _deviceSk;   ///< Device secret key

  // MQTT client information
  std::string _clientUrl;      ///< Full MQTT connection URL
  std::string _clientId;       ///< MQTT client ID
  std::string _clientUsername; ///< MQTT username
  std::string _clientPassword; ///< MQTT password

  // MQTT client handle
  MQTTAsync _client; ///< MQTTAsync client handle

  // Thread and Synchronization
  std::atomic<bool> _connected; ///< Connection status flag
  std::mutex _mutex;            ///< Mutex for thread-safe operations

  // Topic management
  std::vector<std::string> _subTopics; ///< Topics to subscribe to
  std::string _topicPrefix;            ///< Prefix for all MQTT topics,
                            ///< e.g., /sys/${ProductKey}/${DeviceName}

  std::vector<MessageQueue *> _recvBuffer; ///< Message queues for receiving
};

}}} // namespace rynnrcp::fw::common

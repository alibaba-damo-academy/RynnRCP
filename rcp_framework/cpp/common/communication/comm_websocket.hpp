/**
 * @file comm_websocket.hpp
 * @brief Header file for CWebsocketComm class
 */

#pragma once

#include "comm_data.hpp"
#include "comm_type.hpp"

namespace rynnrcp { namespace fw { namespace common {

/**
 * @class CWebsocketComm
 * @brief WebSocket communication implementation
 *
 * Implements the WebSocket protocol for communication between devices and
 * servers
 */
class CWebsocketComm : public CDataComm {
public:
  CWebsocketComm();
  ~CWebsocketComm();

  /**
   * @brief Initialize WebSocket communication with provided configuration
   * @param config_file Configuration file path
   */
  void init(const std::string &config_file) override;

  /**
   * @brief Start the WebSocket communication
   */
  void start() override;

  /**
   * @brief Close the WebSocket connection
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
  /**
   * @brief Reconnect to the WebSocket server
   */
  void reconnect();

  /**
   * @brief Handle incoming messages via WebSocket
   * @param user User data pointer
   * @param in Incoming data buffer
   * @param len Length of incoming data
   * @return Status code
   */
  int32_t onMessageArrived(void *user, void *in, int len);

  /**
   * @brief Set up the WebSocket connection callback
   * @param flag Connection flag to indicate the connection status
   */
  void onConnectionStateChanged(WebsocketConnFlag flag);

  /**
   * @brief Callback function for WebSocket events
   * @param wsi Pointer to WebSocket interface
   * @param reason Reason for the callback
   * @param user User data pointer
   * @param in Incoming data buffer
   * @param len Length of incoming data
   * @return Status code
   */
  static int onWebsocketEvent(struct lws *wsi, enum lws_callback_reasons reason,
                              void *user, void *in, size_t len);

private:
  // Auth client
  std::shared_ptr<CAuthManager> _authClient;

  // Members for WebSocket state and configuration
  std::string _websocketUrl;  ///< WebSocket URL
  std::string _websocketHost; ///< WebSocket Host
  std::string _websocketPath; ///< WebSocket Path
  std::string _token;         ///< Token for authentication
  std::string _schema;        ///< Protocol schema
  std::string _tunnelId;      ///< Tunnel ID
  std::string _relayMode;     ///< Relay mode
  int32_t _websocketPort;     ///< WebSocket port number
  int32_t _expire;            ///< Expiration time

  // Connection and state management
  std::atomic<bool> _connected;                      ///< Connection status
  std::atomic<bool> _clientWritable;                 ///< Writeability status
  std::atomic<WebsocketConnFlag> _websocketConnFlag; ///< Connection flag
  std::mutex _mutex;           ///< Mutex for thread-safe operations
  std::condition_variable _cv; ///< Condition variable for synchronization

  // WebSocket context and connection information
  struct lws *_wsInterface;                      ///< WebSocket interface
  struct lws_context *_context;                  ///< WebSocket context
  struct lws_context_creation_info _contextInfo; ///< Context creation info
  struct lws_client_connect_info _connectInfo;   ///< Client connection info

  // Threads for handling events
  std::thread _eventLoopThread; ///< Thread running the event loop

  std::vector<MessageQueue *> _recvBuffer; ///< Message queue for receive
};

}}} // namespace rynnrcp::fw::common

/**
 * @file server.hpp
 * @brief Base server class implementation
 */

#pragma once

#include "comm_mqtt.hpp"
#include "comm_type.hpp"
#include "comm_websocket.hpp"

namespace rynnrcp { namespace fw { namespace robot {

using rynnrcp::fw::common::kMaxMsgBufSize;
using CDataComm = rynnrcp::fw::common::CDataComm;
using CJsonRpcComm = rynnrcp::fw::common::CJsonRpcComm;
using MessageQueue = rynnrcp::fw::common::MessageQueue;
using MessagePointer = rynnrcp::fw::common::MessagePointer;

/**
 * @class CServer
 * @brief Base server class
 */
class CServer {
public:
  /**
   * @brief Constructs a CServer instance.
   * @param name Server name (default: "Server")
   * @param jsonrpc_client Client for JSON-RPC communication
   * @param data_client Client for data communication
   */
  CServer(const std::string &name = "Server",
          const std::shared_ptr<CJsonRpcComm> jsonrpc_client = nullptr,
          const std::shared_ptr<CDataComm> data_client = nullptr);

  virtual ~CServer();

  /**
   * @brief Starts the server
   * @return True if the server started successfully, false otherwise
   */
  bool start();

  /**
   * @brief Stops the server and shuts down all communication threads
   */
  void stop();

  /**
   * @brief Handles a received message
   * @param msg Pointer to the message
   * @return Status of the handling operation
   */
  virtual int32_t handleMessage(MessagePointer msg);

protected:
  std::string _serverName;    ///< Name of the server
  std::string _serverVersion; ///< Version of the server

  std::shared_ptr<CDataComm> _dataClient;       ///< Data client
  std::shared_ptr<CJsonRpcComm> _jsonrpcClient; ///< JSON-RPC client

  std::shared_ptr<MessageQueue> _recvBuffer; ///< Receive buffer

  std::thread _bufferMonitorThread; ///< Thread for monitoring the buffer

  std::mutex _mutex;           ///< Protects shared resources
  std::atomic<bool> _running;  ///< Indicates if the server is running
  std::atomic<bool> _occupied; ///< Indicates if the server is occupied
};

}}} // namespace rynnrcp::fw::robot

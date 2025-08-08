/**
 * @file terminal_device_server.hpp
 * @brief Header file for CTerminalDeviceServer class
 */

#pragma once

#include "server.hpp"

namespace rynnrcp { namespace fw { namespace robot {

using rynnrcp::fw::common::getTimeMs;
using rynnrcp::fw::common::kDeviceBusyCode;
using rynnrcp::fw::common::kJsonParseErrorCode;
using rynnrcp::fw::common::kDeviceNotOccupiedCode;

using Json = rynnrcp::fw::common::Json;
using CDataComm = rynnrcp::fw::common::CDataComm;
using CJsonRpcComm = rynnrcp::fw::common::CJsonRpcComm;
using CDataMessage = rynnrcp::fw::common::CDataMessage;
using MessagePointer = rynnrcp::fw::common::MessagePointer;
using CJsonRpcMessage = rynnrcp::fw::common::CJsonRpcMessage;

/**
 * @class CTerminalDeviceServer
 * @brief Handles JSON-RPC messages, passing DATA messages
 */
class CTerminalDeviceServer : public CServer {
public:
  /**
   * @brief Constructs a CTerminalDeviceServer instance
   * @param name Server name
   * @param jsonrpc_client Client for JSON-RPC communication
   * @param data_client Client for data communication
   */
  CTerminalDeviceServer(const std::string &name,
                        const std::shared_ptr<CJsonRpcComm> jsonrpc_client,
                        const std::shared_ptr<CDataComm> data_client);

  ~CTerminalDeviceServer();

  /**
   * @brief Handles a received message
   * @param msg Pointer to the message to handle
   * @return Status of the handling operation
   */
  int32_t handleMessage(MessagePointer msg) override;

  /**
   * @brief Processes a received JSON-RPC message
   * @param msg Pointer to the JSON-RPC message to process
   * @return Status of the message processing
   */
  int32_t processJsonRpcMsg(const std::shared_ptr<CJsonRpcMessage> msg);

  /**
   * @brief Processes a DATA message
   * @param msg Message to process
   * @return Processing status
   */
  virtual int32_t processDataMsg(const std::shared_ptr<CDataMessage> msg);

  /**
   * @brief Acquires a device using the provided JSON-RPC message
   * @param msg Pointer to the JSON-RPC message
   * @return Status of the acquisition operation
   */
  int32_t acquireDevice(const std::shared_ptr<CJsonRpcMessage> msg);

  /**
   * @brief Releases a device using the provided JSON-RPC message
   * @param msg Pointer to the JSON-RPC message
   * @return Status of the release operation
   */
  int32_t releaseDevice(const std::shared_ptr<CJsonRpcMessage> msg);
};

}}} // namespace rynnrcp::fw::robot

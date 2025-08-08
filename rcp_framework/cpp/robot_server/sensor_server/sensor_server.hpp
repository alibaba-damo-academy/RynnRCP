/**
 * @file sensor_server.hpp
 * @brief Header file for the CSensorServer class
 */

#pragma once

#include "terminal_device_server.hpp"

namespace rynnrcp { namespace fw { namespace robot {

using Json = rynnrcp::fw::common::Json;
using rynnrcp::fw::common::kChannelImageRequest;
using rynnrcp::fw::common::kChannelImageResponse;
using CDataComm = rynnrcp::fw::common::CDataComm;
using CJsonRpcComm = rynnrcp::fw::common::CJsonRpcComm;
using MessagePointer = rynnrcp::fw::common::MessagePointer;
using CJsonRpcMessage = rynnrcp::fw::common::CJsonRpcMessage;

/**
 * @class CSensorServer
 * @brief Extends server functionality to handle sensor data and LCM messages
 */
class CSensorServer : public CTerminalDeviceServer {
public:
  /**
   * @brief Constructs a CSensorServer instance
   * @param name Server name
   * @param jsonrpc_client Client for JSON-RPC communication
   * @param data_client Client for data communication
   */
  CSensorServer(const std::string &name,
                const std::shared_ptr<CJsonRpcComm> jsonrpc_client,
                const std::shared_ptr<CDataComm> data_client);

  ~CSensorServer();

  /**
   * @brief Processes a Data message
   * @param msg Message to process
   * @return Processing status
   */
  int32_t processDataMsg(const std::shared_ptr<CDataMessage> msg);

  /**
   * @brief Sends a LCM image request
   * @param msg Message to send
   * @return Status of the send operation
   */
  int32_t sendLcmImageRequest(const std::shared_ptr<CDataMessage> msg);

  // LCM message handlers
  /**
   * @brief Processes state feedback messages
   * @param rbuf Pointer to the receive buffer
   * @param chan The channel from which the message is received
   * @param msg Pointer to the image response message
   */
  void handleImageResponse(const lcm::ReceiveBuffer *rbuf,
                           const std::string &chan,
                           const lcmSensor::camera_image_response *msg);

private:
  lcm::LCM _lcm;                ///< LCM instance
  std::thread _lcmThread;       ///< Thread for LCM processing
  std::atomic<int32_t> _lcmCnt; ///< Atomic counter for LCM messages
  int32_t _reqImageId;          ///< REQ_IMAGE message ID
};

}}} // namespace rynnrcp::fw::robot

/**
 * @file action_server.hpp
 * @brief Header file for CActionServer class
 */

#pragma once

#include "terminal_device_server.hpp"

namespace rynnrcp { namespace fw { namespace robot {

using rynnrcp::fw::common::kChannelActFeedback;
using rynnrcp::fw::common::kChannelRobotFeedback;
using rynnrcp::fw::common::kChannelMotionCommand;
using rynnrcp::fw::common::kChannelRequestFeedback;

using CDataComm = rynnrcp::fw::common::CDataComm;
using CJsonRpcComm = rynnrcp::fw::common::CJsonRpcComm;
using CDataMessage = rynnrcp::fw::common::CDataMessage;
using MessagePointer = rynnrcp::fw::common::MessagePointer;

/**
 * @class CActionServer
 * @brief Extends server functionality to handle actions and LCM messages
 */
class CActionServer : public CTerminalDeviceServer {
public:
  /**
   * @brief Constructs a CActionServer instance
   * @param name Server name
   * @param jsonrpc_client Client for JSON-RPC communication
   * @param data_client Client for data communication
   */
  CActionServer(const std::string &name,
                const std::shared_ptr<CJsonRpcComm> jsonrpc_client,
                const std::shared_ptr<CDataComm> data_client);

  ~CActionServer();

  /**
   * @brief Processes a data message
   * @param msg Message to process
   * @return Processing status
   */
  int32_t processDataMsg(const std::shared_ptr<CDataMessage> msg);

  /**
   * @brief Sends a LCM state request
   * @param msg Message to send
   * @return Status of the send operation
   */
  int32_t sendLcmStateRequest(const std::shared_ptr<CDataMessage> msg);

  /**
   * @brief Sends LCM action data
   * @param msg Message to send
   * @return Status of the send operation
   */
  int32_t sendLcmActionData(const std::shared_ptr<CDataMessage> msg);

  /**
   * @brief Handles action feedback messages
   * @param rbuf Pointer to the receive buffer
   * @param chan Communication channel name
   * @param msg Pointer to the feedback message
   */
  void handleActFeedback(const lcm::ReceiveBuffer *rbuf,
                         const std::string &chan,
                         const lcmMotion::state_feedback *msg);

  /**
   * @brief Handles robot feedback messages
   * @param rbuf Pointer to the receive buffer
   * @param chan Communication channel name
   * @param msg Pointer to the feedback message
   */
  void handleRobotFeedback(const lcm::ReceiveBuffer *rbuf,
                           const std::string &chan,
                           const lcmMotion::robot_feedback *msg);

private:
  lcm::LCM _lcm;                ///< LCM instance
  std::thread _lcmThread;       ///< Thread for LCM processing
  std::atomic<int32_t> _lcmCnt; ///< LCM message counter
  int32_t _reqStateId;          ///< Message ID for REQ_STATE
  int32_t _actionDataId;        ///< Message ID for ACTION_DATA
};

}}} // namespace rynnrcp::fw::robot

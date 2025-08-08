/**
 * @file action_server.cpp
 * @brief Implementation of the CActionServer server
 */

#include "action_server.hpp"

namespace rynnrcp { namespace fw { namespace robot {

CActionServer::CActionServer(const std::string &name,
                             const std::shared_ptr<CJsonRpcComm> jsonrpc_client,
                             const std::shared_ptr<CDataComm> data_client) :
    CTerminalDeviceServer(name, jsonrpc_client, data_client) {
  if (!_lcm.good()) { throw std::runtime_error("LCM initialization failed"); }

  _lcm.subscribe(kChannelActFeedback, &CActionServer::handleActFeedback, this);
  _lcm.subscribe(kChannelRobotFeedback, &CActionServer::handleRobotFeedback,
                 this);

  LOG(INFO) << "[" << _serverName << "][LCM]: Subscribe channel '"
            << kChannelActFeedback << "' and '" << kChannelRobotFeedback << "'";

  _lcmCnt.store(0, std::memory_order_release);

  _lcmThread = std::thread([&]() {
    while (0 == _lcm.handle()) { ; }
  });
}

CActionServer::~CActionServer() {
}

void CActionServer::handleActFeedback(const lcm::ReceiveBuffer *rbuf,
                                      const std::string &chan,
                                      const lcmMotion::state_feedback *msg) {
  LOG(INFO) << "[" << _serverName
            << "][LCM]: Message arrived on channel: " << chan;

  std::string status_text = "Unknown Act Status";
  switch (msg->act_status_type) {
  case lcmMotion::state_feedback::kIdle: status_text = "Idle"; break;
  case lcmMotion::state_feedback::kSuccess: status_text = "Success"; break;
  case lcmMotion::state_feedback::kExecuting: status_text = "Executing"; break;
  case lcmMotion::state_feedback::kPaused: status_text = "Paused"; break;
  case lcmMotion::state_feedback::kCollision: status_text = "Collision"; break;
  case lcmMotion::state_feedback::kFail: status_text = "Fail"; break;
  default: break;
  }

  std::string stateID_text = "Unknown StateID";
  switch (msg->stateID) {
  case lcmMotion::state_feedback::kGoStand: stateID_text = "GoStand"; break;
  case lcmMotion::state_feedback::kGoHome: stateID_text = "GoHome"; break;
  case lcmMotion::state_feedback::kMove1: stateID_text = "Move1"; break;
  case lcmMotion::state_feedback::kMove2: stateID_text = "Move2"; break;
  case lcmMotion::state_feedback::kError: stateID_text = "ErrorState"; break;
  default: break;
  }

  std::string substateID_text = "Unknown SubStateID";
  switch (msg->substateID) {
  case lcmMotion::state_feedback::kEnter: substateID_text = "Enter"; break;
  case lcmMotion::state_feedback::kDuring: substateID_text = "During"; break;
  case lcmMotion::state_feedback::kExit: substateID_text = "Exit"; break;
  default: break;
  }

  LOG(INFO) << "[" << _serverName << "][LCM]: Received state feedback #"
            << msg->seq << ", act_status: " << msg->act_status_type << " ("
            << status_text << "), "
            << "error: '" << msg->error_msg << "', "
            << "stateID: " << msg->stateID << " (" << stateID_text << "), "
            << "substateID: " << msg->substateID << " (" << substateID_text
            << "), "
            << "state_msg: '" << msg->state_msg << "'"
            << " timestamp: " << msg->utime;

  RobotServer::FinishActionChunk finish_action_chunk;
  if (status_text == "Success") {
    finish_action_chunk.set_code(0);
    finish_action_chunk.set_error_msg(msg->error_msg);
    finish_action_chunk.set_execute_steps(1);
    finish_action_chunk.set_expect_steps(1);
  } else {
    finish_action_chunk.set_code(-1);
    finish_action_chunk.set_error_msg(msg->error_msg);
    finish_action_chunk.set_execute_steps(0);
    finish_action_chunk.set_expect_steps(1);
  }

  std::string serialized_data;
  if (!finish_action_chunk.SerializeToString(&serialized_data)) {
    LOG(ERROR) << "[" << _serverName
               << "][DATA]: Failed to serialize FinishActionChunk";
    return;
  }

  MessagePointer protocol_msg = std::make_shared<CDataMessage>(
      _actionDataId,
      static_cast<int32_t>(RobotServer::PackageType::ACTION_FINISH),
      serialized_data);
  _dataClient->send(protocol_msg);
}

inline float radiansToDegrees(float radians) {
  return radians * 180.0 / M_PI;
}
inline float degreesToRadians(float degrees) {
  return degrees * M_PI / 180.0;
}

void CActionServer::handleRobotFeedback(const lcm::ReceiveBuffer *rbuf,
                                        const std::string &chan,
                                        const lcmMotion::robot_feedback *msg) {
  LOG(INFO) << "[" << _serverName
            << "][LCM]: Message arrived on channel: " << chan;

  RobotServer::MultiState reply_state;

  std::vector<float> state_data;
  int32_t joint_num = msg->numJoint;
  for (int i = 0; i < joint_num; ++i) {
    float radians = msg->qFb[i];
    float degrees = radiansToDegrees(radians);
    LOG(INFO) << "  Joint position[ " << i << "] = " << degrees << " degrees ("
              << radians << " radians)";
    state_data.push_back(degrees);
  }
  int32_t gripper_num = msg->numGripper;
  for (int i = 0; i < gripper_num; ++i) {
    LOG(INFO) << "  Gripper position[ " << i << "] = " << msg->gripperPosFb[i];
    state_data.push_back(msg->gripperPosFb[i]);
  }

  std::string data_string(state_data.size() * sizeof(float), '\0');
  std::memcpy(&data_string[0], state_data.data(),
              state_data.size() * sizeof(float));

  RobotServer::Array robot_state;
  robot_state.set_dtype(RobotServer::DataType::FLOAT32);
  robot_state.add_shape(joint_num + gripper_num);
  robot_state.set_data(data_string);
  reply_state.add_state_list()->CopyFrom(robot_state);

  std::string serialized_data;
  if (!reply_state.SerializeToString(&serialized_data)) {
    LOG(ERROR) << "[" << _serverName
               << "][DATA]: Failed to serialize MultiState";
    return;
  }

  LOG(INFO) << "serialized_data.data: " << serialized_data.size();

  MessagePointer protocol_msg = std::make_shared<CDataMessage>(
      _reqStateId, static_cast<int32_t>(RobotServer::PackageType::STATE_DATA),
      serialized_data);
  _dataClient->send(protocol_msg);
}

int32_t CActionServer::sendLcmStateRequest(
    const std::shared_ptr<CDataMessage> msg) {
  // Protobuf message
  RobotServer::ReqState req_state;
  // LCM message
  lcmMotion::act_request request_feedback;

  if (req_state.ParseFromArray(msg->getData().data(), msg->getData().size())) {
    LOG(INFO) << "[" << _serverName << "][DATA]: PackageType: REQ_STATE";

    _reqStateId = msg->getId();
    for (const auto &robot : req_state.robot()) {
      int32_t robot_id = robot.robot_id();
      std::string robot_name = robot.robot_name();
      LOG(INFO) << "[" << _serverName << "][DATA]: Robot_id: " << robot_id
                << " robot_name: " << robot_name;
    }

    // Only care the message type, not content.
    auto now = std::chrono::system_clock::now();
    auto epoch = now.time_since_epoch();
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(epoch);
    auto nanoseconds =
        std::chrono::duration_cast<std::chrono::nanoseconds>(epoch - seconds);

    request_feedback.sec = seconds.count();
    request_feedback.nanosec = nanoseconds.count();
    request_feedback.utime =
        std::chrono::duration_cast<std::chrono::microseconds>(epoch).count();

    request_feedback.seq = _lcmCnt.fetch_add(1);
    request_feedback.request_type = 1; // 0 → ACT status  1 → Robot status

    _lcm.publish(kChannelRequestFeedback, &request_feedback);
    LOG(INFO) << "[" << _serverName
              << "][LCM] Send request Robot Status - kRobotStatusRequest, seq: "
              << request_feedback.seq;
  } else {
    LOG(ERROR) << "[" << _serverName
               << "][DATA]: Packet.data is not ReqState format";
  }

  return 0;
}
int32_t CActionServer::sendLcmActionData(
    const std::shared_ptr<CDataMessage> msg) {
  // Protobuf message
  RobotServer::MultiAction multi_action;

  // LCM message
  lcmMotion::act_command act_command;

  if (multi_action.ParseFromArray(msg->getData().data(),
                                  msg->getData().size())) {
    _actionDataId = msg->getId();
    for (const auto &action : multi_action.action_list()) {
      RobotServer::ActionType action_type = action.action_type();
      int32_t action_rate = action.action_rate();
      RobotServer::Array action_data = action.action_data();
      RobotServer::DataType dtype = action_data.dtype();
      std::string raw_data = action_data.data();
      int32_t step = action_data.shape(0);
      int32_t joints = action_data.shape(1);

      LOG(INFO) << "[" << _serverName << "][DATA]: PackageType: ACTION_DATA"
                << " Action_rate: " << action_rate << "Step: " << step
                << " joints: " << joints;

      switch (dtype) {
      case RobotServer::DataType::FLOAT32: {
        int32_t item_num = step * joints;
        if (item_num != raw_data.size() / sizeof(float)) {
          LOG(ERROR) << "[" << _serverName
                     << "][DATA]: Parse action data error: action_data "
                        "size != item_num * sizeof(float)";
          break;
        }

        float *data_point = (float *)raw_data.data();
        for (int32_t i = 0; i < step; i++) {
          std::ostringstream output_stream;
          output_stream << "data_point[" << i << "]: ";

          for (int32_t j = 0; j < joints; j++) {
            float angle_value = data_point[i * joints + j];
            float radian_value = degreesToRadians(angle_value);
            output_stream << angle_value << " (radians: " << radian_value
                          << ") ";
          }

          LOG(INFO) << output_stream.str();
        }

        auto epoch = std::chrono::system_clock::now().time_since_epoch();
        auto seconds = std::chrono::duration_cast<std::chrono::seconds>(epoch);
        auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(
            epoch - seconds);

        act_command.sec = seconds.count();
        act_command.nanosec = nanoseconds.count();
        act_command.utime =
            std::chrono::duration_cast<std::chrono::microseconds>(epoch)
                .count();
        act_command.seq = _lcmCnt.fetch_add(1);

        act_command.chunkSize = step;
        act_command.numJoint = joints - 1;
        act_command.numGripper = 1; // Assuming 1 gripper
        act_command.totalNumJoint =
            act_command.chunkSize * act_command.numJoint;
        act_command.totalNumGripper =
            act_command.chunkSize * act_command.numGripper;
        act_command.totalEePos = act_command.chunkSize * 3;
        act_command.totalEeQuat = act_command.chunkSize * 4;
        act_command.workMode = 0;

        // Reserve memory
        act_command.jointPos.resize(act_command.totalNumJoint);
        act_command.jointVel.resize(act_command.totalNumJoint);
        act_command.gripperPos.resize(act_command.totalNumGripper);
        act_command.eePos.resize(act_command.totalEePos);
        act_command.eeQuat.resize(act_command.totalEeQuat);

        // Set joint positions and velocities
        for (int32_t i = 0; i < step; ++i) {
          for (int32_t j = 0; j < joints - 1; ++j) {
            int32_t idx = i * (joints - 1) + j;
            float radian = degreesToRadians(data_point[i * joints + j]);
            act_command.jointPos[idx] = radian;
            act_command.jointVel[idx] = 0.0f;
          }

          act_command.gripperPos[i] = data_point[i * joints + joints - 1];

          // Set end effector positions and orientations
          int32_t ee_pos_base = i * 3;
          act_command.eePos[ee_pos_base + 0] = 0.0f;
          act_command.eePos[ee_pos_base + 1] = 0.0f;
          act_command.eePos[ee_pos_base + 2] = 0.0f;

          int32_t ee_quat_base = i * 4;
          act_command.eeQuat[ee_quat_base + 0] = 0.0f; // w
          act_command.eeQuat[ee_quat_base + 1] = 0.0f; // x
          act_command.eeQuat[ee_quat_base + 2] = 0.0f; // y
          act_command.eeQuat[ee_quat_base + 3] = 0.0f; // z
        }

        _lcm.publish(kChannelMotionCommand, &act_command);
        LOG(INFO) << "[" << _serverName
                  << "][LCM]: Send action command, seq: " << act_command.seq;
      } break;
      // TODO: support other data types
      default:
        LOG(ERROR) << "[" << _serverName
                   << "][DATA]: Unsupported data type: " << dtype;
        break;
      }
    }
  } else {
    LOG(ERROR) << "[" << _serverName
               << "][DATA]: Packet.data is not MultiAction format";
  }

  return 0;
}

int32_t CActionServer::processDataMsg(const std::shared_ptr<CDataMessage> msg) {
  switch (msg->getType()) {
  case RobotServer::PackageType::ACTION_DATA: sendLcmActionData(msg); break;
  case RobotServer::PackageType::REQ_STATE: sendLcmStateRequest(msg); break;
  default: break;
  }

  return 0;
}

}}} // namespace rynnrcp::fw::robot

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

void CActionServer::handleRobotFeedback(const lcm::ReceiveBuffer *rbuf,
                                        const std::string &chan,
                                        const lcmMotion::robot_feedback *msg) {
  LOG(INFO) << "[" << _serverName
            << "][LCM]: Message arrived on channel: " << chan;

  RobotServer::MultiState reply_state;

  // arm state data
  RobotServer::Array arm_state_array;
  RobotServer::State arm_state;

  std::vector<float> arm_state_data;
  int32_t joint_num = msg->numJoint;
  for (int i = 0; i < joint_num; ++i) {
    LOG(INFO) << "  Joint position[ " << i << "] = " << msg->qFb[i];
    arm_state_data.push_back(msg->qFb[i]);
  }

  std::string arm_data_string(arm_state_data.size() * sizeof(float), '\0');
  std::memcpy(&arm_data_string[0], arm_state_data.data(),
              arm_state_data.size() * sizeof(float));

  arm_state_array.add_shape(joint_num);
  arm_state_array.set_data(arm_data_string);
  arm_state_array.set_dtype(RobotServer::DataType::FLOAT32);

  arm_state.set_id(0);
  arm_state.set_name(kArmStateName);
  arm_state.mutable_state_data()->CopyFrom(arm_state_array);
  reply_state.add_state_list()->CopyFrom(arm_state);

  // gripper state string
  RobotServer::Array gripper_state_array;
  RobotServer::State gripper_state;

  std::vector<float> gripper_state_data;
  int32_t gripper_num = msg->numGripper;
  for (int i = 0; i < gripper_num; ++i) {
    LOG(INFO) << "  Gripper position[" << i << "] = " << msg->gripperPosFb[i];
    gripper_state_data.push_back(msg->gripperPosFb[i]);
  }

  std::string gripper_data_string(gripper_state_data.size() * sizeof(float),
                                  '\0');
  std::memcpy(&gripper_data_string[0], gripper_state_data.data(),
              gripper_state_data.size() * sizeof(float));

  gripper_state_array.add_shape(gripper_num);
  gripper_state_array.set_data(gripper_data_string);
  gripper_state_array.set_dtype(RobotServer::DataType::FLOAT32);

  gripper_state.set_id(0);
  gripper_state.set_name(kGripperStateName);
  gripper_state.mutable_state_data()->CopyFrom(gripper_state_array);

  bool all_zero = true;
  if (msg->numFTsensor == 1) {
    for (int32_t j = 0; j < 6; j++) {
      if (msg->ftSensorFb[0][j] != 0.0f) {
        all_zero = false;
        break;
      }
    }
  }

  if (!all_zero) {
    RobotServer::Array wrench_state_array;
    RobotServer::State wrench_state;

    std::vector<float> wrench_state_data;
    for (int32_t i = 0; i < msg->numFTsensor; i++) {
      for (int32_t j = 0; j < 6; j++) {
        LOG(INFO) << "  Wrench position[ " << i << "][" << j
                  << "]= " << msg->ftSensorFb[i][j];
        wrench_state_data.push_back(msg->ftSensorFb[i][j]);
      }
    }

    std::string wrench_data_string(wrench_state_data.size() * sizeof(float),
                                   '\0');
    std::memcpy(&wrench_data_string[0], wrench_state_data.data(),
                wrench_state_data.size() * sizeof(float));

    wrench_state_array.add_shape(wrench_state_data.size());
    wrench_state_array.set_data(wrench_data_string);
    wrench_state_array.set_dtype(RobotServer::DataType::FLOAT32);
    wrench_state.set_id(0);
    wrench_state.set_name(kWrenchStateName);
    wrench_state.mutable_state_data()->CopyFrom(wrench_state_array);
    reply_state.add_state_list()->CopyFrom(wrench_state);
  }

  reply_state.add_state_list()->CopyFrom(gripper_state);

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

  std::vector<int8_t> chunk_sizes;

  if (multi_action.ParseFromArray(msg->getData().data(),
                                  msg->getData().size())) {
    _actionDataId = msg->getId();

    for (const auto &action : multi_action.action_list()) {
      int32_t id = action.id();
      std::string name = action.name();
      int32_t action_rate = action.action_rate();
      RobotServer::Array action_data = action.action_data();

      std::string raw_data = action_data.data();
      RobotServer::DataType dtype = action_data.dtype();
      int32_t step = action_data.shape(0);
      int32_t joints = action_data.shape(1);

      chunk_sizes.push_back(step);

      LOG(INFO) << "[" << _serverName << "][DATA]: PackageType: ACTION_DATA"
                << " Action_rate: " << action_rate << " Step: " << step
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
            output_stream << data_point[i * joints + j] << " ";
          }

          LOG(INFO) << output_stream.str();
        }

        if (name == kArmActionName) {
          act_command.numJoint = joints;
          // Set joint positions and velocities
          for (int32_t i = 0; i < step; ++i) {
            for (int32_t j = 0; j < joints; ++j) {
              act_command.jointPos.push_back(data_point[i * joints + j]);
              act_command.jointVel.push_back(0.0);
            }
          }
        } else if (name == kGripperActionName) {
          act_command.numGripper = joints;
          // Set gripper position
          for (int32_t i = 0; i < step; ++i) {
            for (int32_t j = 0; j < joints; ++j) {
              act_command.gripperPos.push_back(data_point[i * joints + j]);
            }
          }
        } else {
          LOG(ERROR) << "Unsupported action name: " << name;
          return -1;
        }

      } break;
      // TODO: support other data types
      default:
        LOG(ERROR) << "[" << _serverName
                   << "][DATA]: Unsupported data type: " << dtype;
        return -1;
      }
    }
  } else {
    LOG(ERROR) << "[" << _serverName
               << "][DATA]: Packet.data is not MultiAction format";
    return -1;
  }

  // check chunk_sizes should be equal
  for (int32_t i = 0; i < chunk_sizes.size(); i++) {
    if (chunk_sizes[i] != chunk_sizes[0]) {
      LOG(ERROR)
          << "[" << _serverName
          << "][DATA]: All actions in Packet.data should have same chunk size";
      return -1;
    }
  }

  auto epoch = std::chrono::system_clock::now().time_since_epoch();
  auto seconds = std::chrono::duration_cast<std::chrono::seconds>(epoch);
  auto nanoseconds =
      std::chrono::duration_cast<std::chrono::nanoseconds>(epoch - seconds);

  act_command.sec = seconds.count();
  act_command.nanosec = nanoseconds.count();
  act_command.utime =
      std::chrono::duration_cast<std::chrono::microseconds>(epoch).count();
  act_command.seq = _lcmCnt.fetch_add(1);

  act_command.chunkSize = chunk_sizes[0];
  act_command.totalNumJoint = act_command.chunkSize * act_command.numJoint;
  act_command.totalNumGripper = act_command.chunkSize * act_command.numGripper;

  // Set end effector positions and orientations
  act_command.totalEePos = act_command.chunkSize * 3;
  act_command.totalEeQuat = act_command.chunkSize * 4;
  act_command.eePos = std::vector<float>(act_command.totalEePos, 0);
  act_command.eeQuat = std::vector<float>(act_command.totalEeQuat, 0);

  act_command.workMode = 0;

  _lcm.publish(kChannelMotionCommand, &act_command);
  LOG(INFO) << "[" << _serverName
            << "][LCM]: Send action command, seq: " << act_command.seq;

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

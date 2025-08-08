/**
 * @file sensor_server.cpp
 * @brief Implementation of the CSensorServer class
 */

#include "sensor_server.hpp"

namespace rynnrcp { namespace fw { namespace robot {
CSensorServer::CSensorServer(const std::string &name,
                             const std::shared_ptr<CJsonRpcComm> jsonrpc_client,
                             const std::shared_ptr<CDataComm> data_client) :
    CTerminalDeviceServer(name, jsonrpc_client, data_client) {
  if (!_lcm.good()) { throw std::runtime_error("LCM initialization failed"); }

  _lcm.subscribe(kChannelImageResponse, &CSensorServer::handleImageResponse,
                 this);

  LOG(INFO) << "[" << _serverName << "][LCM]: Subscribe channel '"
            << kChannelImageResponse << "'";

  _lcmThread = std::thread([this]() {
    while (0 == _lcm.handle()) { ; }
  });
}

CSensorServer::~CSensorServer() {
}

void CSensorServer::handleImageResponse(
    const lcm::ReceiveBuffer *rbuf, const std::string &chan,
    const lcmSensor::camera_image_response *msg) {
  LOG(INFO) << "[" << _serverName
            << "][LCM]: Message arrived on channel: " << chan;

  RobotServer::MultiImage reply_image;

  for (int i = 0; i < msg->camera_count; ++i) {
    const lcmSensor::image_data &data = msg->images[i];

    RobotServer::Image image;
    image.set_id(data.camera_id);
    image.set_name(data.camera_name);
    image.set_format("png");

    RobotServer::Array image_array;
    image_array.set_dtype(RobotServer::DataType::UINT8);
    image_array.add_shape(data.height);
    image_array.add_shape(data.width);
    image_array.add_shape(data.channels);
    image_array.set_data(data.image_bytes.data(), data.image_bytes.size());

    LOG(INFO) << "[" << _serverName
              << "][LCM]: Received image data feedback camera_id: "
              << data.camera_id << " camera_name: " << data.camera_name
              << " image_size: " << data.image_size
              << " height: " << data.height << " width: " << data.width
              << " channels: " << data.channels;

    image.mutable_image_data()->CopyFrom(image_array);
    reply_image.add_image_list()->CopyFrom(image);
  }

  std::string serialized_data;
  if (!reply_image.SerializeToString(&serialized_data)) {
    LOG(ERROR) << "[" << _serverName
               << "][DATA]: Failed to serialize MultiImage to string";
    return;
  }

  MessagePointer protocol_msg = std::make_shared<CDataMessage>(
      _reqImageId, static_cast<int32_t>(RobotServer::PackageType::IMAGE_DATA),
      serialized_data);
  _dataClient->send(protocol_msg);
}

int32_t CSensorServer::sendLcmImageRequest(
    const std::shared_ptr<CDataMessage> msg) {
  // Protobuf message
  RobotServer::ReqImage req_image;

  // LCM message
  lcmSensor::req_camera_image image_request;

  if (req_image.ParseFromArray(msg->getData().data(), msg->getData().size())) {
    _reqImageId = msg->getId();
    image_request.seq = msg->getId();
    image_request.camera_count = req_image.camera().size();
    LOG(INFO) << "[" << _serverName << "][DATA]: PackageType: REQ_IMAGE";

    for (const auto &camera : req_image.camera()) {
      int32_t camera_id = camera.camera_id();
      std::string camera_name = camera.camera_name();
      LOG(INFO) << "[" << _serverName << "][DATA]: Camera_id: " << camera_id
                << " camera_name: " << camera_name;

      image_request.camera_ids.push_back(camera_id);
      image_request.camera_names.push_back(camera_name);
    }

    _lcm.publish(kChannelImageRequest, &image_request);
    LOG(INFO) << "[" << _serverName
              << "][LCM]: Send image request, seq: " << image_request.seq;
  } else {
    LOG(ERROR) << "[" << _serverName << "][DATA]: invalid req_image message";
  }
  return 0;
}

int32_t CSensorServer::processDataMsg(const std::shared_ptr<CDataMessage> msg) {
  switch (msg->getType()) {
  case RobotServer::PackageType::REQ_IMAGE: sendLcmImageRequest(msg); break;
  default: break;
  }

  return 0;
}

}}} // namespace rynnrcp::fw::robot
/**
 * @file so100_servers.cpp
 * @brief Example implementation of an action/sensor/monitor server.
 *
 * This file sets up logging, loads configurations from a YAML file,
 * initializes MQTT and WebSocket communication, and starts action
 * and sensor servers for data handling.
 */

#include <glog/logging.h>
#include <sys/stat.h>

#include <chrono>
#include <ctime>
#include <iostream>
#include <string>
#include <thread>

#include "robot_server.hpp"

using CJsonRpcComm = rynnrcp::fw::common::CJsonRpcComm;
using CDataComm = rynnrcp::fw::common::CDataComm;
using CMqttComm = rynnrcp::fw::common::CMqttComm;
using CWebsocketComm = rynnrcp::fw::common::CWebsocketComm;
using CActionServer = rynnrcp::fw::robot::CActionServer;
using CSensorServer = rynnrcp::fw::robot::CSensorServer;
using CDeviceMonitorServer = rynnrcp::fw::robot::CDeviceMonitorServer;

int main(int argc, char *argv[]) {
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0]
              << " <path_to_device_config> <path_to_glog_config>" << std::endl;
    return -1;
  }

  std::string device_config_path = argv[1];
  std::string glog_config_path = argv[2];

  rynnrcp::fw::common::initGlogByConfig(glog_config_path);

  LOG(INFO) << "Robot Server Version: " << getServerVersion();

  // 1. Initialize to communication instance by configuration.
  std::shared_ptr<CJsonRpcComm> jsonrpc_client = std::make_shared<CMqttComm>();
  std::shared_ptr<CDataComm> data_client = std::make_shared<CWebsocketComm>();

  try {
    jsonrpc_client->init(device_config_path);
    jsonrpc_client->start();

    data_client->init(device_config_path);
    data_client->start();
  } catch (const std::exception &e) {
    LOG(ERROR) << "Error: " << e.what();
    return -1;
  }

  // 2. Create server
  CActionServer action_server("ActionServer", jsonrpc_client, data_client);
  action_server.start();
  CSensorServer sensor_server("SensorServer", jsonrpc_client, data_client);
  sensor_server.start();
  CDeviceMonitorServer device_monitor_server("DeviceMonitorServer",
                                             jsonrpc_client, data_client);
  device_monitor_server.start();

  while (true) {
    std::cout << "Press Ctrl-C to terminate" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(5));
  }

  return 0;
}

/**
 * @file device_monitor_server.hpp
 * @brief Header file for CDeviceMonitorServer class
 */

#pragma once

#include "terminal_device_server.hpp"

#ifdef __APPLE__
#include <mach/mach.h>
#include <mach/mach_host.h>
#include <mach/processor_info.h>
#include <sys/sysctl.h>
#endif

namespace rynnrcp { namespace fw { namespace robot {

using rynnrcp::fw::common::getNonce;
using rynnrcp::fw::common::getTimeMs;
using Json = rynnrcp::fw::common::Json;
using rynnrcp::fw::common::kChannelCameraDesc;
using CDataComm = rynnrcp::fw::common::CDataComm;
using CJsonRpcComm = rynnrcp::fw::common::CJsonRpcComm;
using MessagePointer = rynnrcp::fw::common::MessagePointer;
using CJsonRpcMessage = rynnrcp::fw::common::CJsonRpcMessage;

/**
 * @class CDeviceMonitorServer
 * @brief Extends server functionality to monitor devices
 */
class CDeviceMonitorServer : public CTerminalDeviceServer {
public:
  /**
   * @brief Constructs a CDeviceMonitorServer instance
   * @param name Server name
   * @param jsonrpc_client Client for JSON-RPC communication
   * @param data_client Client for data communication
   */
  CDeviceMonitorServer(const std::string &name,
                       const std::shared_ptr<CJsonRpcComm> jsonrpc_client,
                       const std::shared_ptr<CDataComm> data_client);

  ~CDeviceMonitorServer();

  /**
   * @brief Reports system properties using the given parameters
   * @param params JSON object containing property details
   */
  void reportProperties(const Json &params);

  /**
   * @brief Reports basic system properties
   */
  void reportBasicProperties();

  /**
   * @brief Retrieves the Ubuntu version
   * @return Ubuntu version string
   */
  std::string getOperatingSystem();

  /**
   * @brief Retrieves the kernel version
   * @return Kernel version string
   */
  std::string getKernelVersion();

  /**
   * @brief Retrieves the system architecture
   * @return Architecture string (e.g., x86_64)
   */
  std::string getArchitecture();

  /**
   * @brief Retrieves the total memory size
   * @return Memory size string
   */
  std::string getMemorySize();

  /**
   * @brief Retrieves the amount of used memory
   * @return Used memory string
   */
  std::string getMemoryUsed();

  /**
   * @brief Retrieves CPU times for various states
   * @return Vector of CPU time values
   */
  std::vector<int64_t> getCpuTimes();

  /**
   * @brief Retrieves the current CPU load
   * @return CPU load string representation
   */
  std::string getCpuLoad();

  /**
   * @brief Handles camera description feedback messages
   * @param rbuf Pointer to the receive buffer
   * @param chan Channel name
   * @param msg Pointer to the camera list description message
   */
  void handleCameraDescFeedback(const lcm::ReceiveBuffer *rbuf,
                                const std::string &chan,
                                const lcmSensor::camera_list_desc *msg);

  /**
   * @brief Sets the robot name
   * @param robot_name
   */
  void setRobotName(const std::string &robot_name);

private:
  lcm::LCM _lcm;                      ///< LCM instance
  std::thread _lcmThread;             ///< Thread for LCM processing
  std::vector<int64_t> _prevCpuTimes; ///< Previous CPU times
  std::vector<int64_t> _currCpuTimes; ///< Current CPU times
  std::thread _periodicReportThread;  ///< Thread for periodic reporting
  std::string _robotName;             ///< Robot name
};

}}} // namespace rynnrcp::fw::robot

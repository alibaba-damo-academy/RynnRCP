/**
 * @file device_monitor_server.cpp
 * @brief Implementation of the CDeviceMonitorServer class
 */

#include "device_monitor_server.hpp"

namespace rynnrcp { namespace fw { namespace robot {

CDeviceMonitorServer::CDeviceMonitorServer(
    const std::string &name, const std::shared_ptr<CJsonRpcComm> jsonrpc_client,
    const std::shared_ptr<CDataComm> data_client) :
    CTerminalDeviceServer(name, jsonrpc_client, data_client) {
  _robotName = "robot";
  reportBasicProperties();

  _prevCpuTimes = getCpuTimes();

  if (!_lcm.good()) { throw std::runtime_error("LCM initialization failed"); }

  _lcm.subscribe(kChannelCameraDesc,
                 &CDeviceMonitorServer::handleCameraDescFeedback, this);

  LOG(INFO) << "[" << _serverName << "][LCM]: Subscribe channel '"
            << kChannelCameraDesc << "'";

  _lcmThread = std::thread([&]() {
    while (0 == _lcm.handle()) { ; }
  });

  _periodicReportThread = std::thread([this]() {
    while (_running.load(std::memory_order_acquire)) {
      std::this_thread::sleep_for(std::chrono::seconds(3));

      Json params;
      params["cpu_load"] = getCpuLoad();
      params["mem_used"] = getMemoryUsed();

      if (_occupied.load(std::memory_order_acquire) == true) {
        LOG(INFO) << "[" << _serverName << "][CPU]: " << params["cpu_load"];
        LOG(INFO) << "[" << _serverName << "][MEM]: " << params["mem_used"];
      }

      reportProperties(params);
    }
  });
}

CDeviceMonitorServer::~CDeviceMonitorServer() {
  if (_periodicReportThread.joinable()) { _periodicReportThread.join(); }
}

void CDeviceMonitorServer::setRobotName(const std::string &robot_name) {
  LOG(INFO) << "[" << _serverName << "] Setting robot name to " << robot_name;
  _robotName = robot_name;
  reportBasicProperties();
}
void CDeviceMonitorServer::reportBasicProperties() {
  Json params;
  params["mem"] = getMemorySize();
  params["arch"] = getArchitecture();
  params["kernel"] = getKernelVersion();
  params["distrib_desc"] = getOperatingSystem();
  params["arm_info"] = _robotName;
  reportProperties(params);
}

void CDeviceMonitorServer::handleCameraDescFeedback(
    const lcm::ReceiveBuffer *rbuf, const std::string &chan,
    const lcmSensor::camera_list_desc *msg) {
  LOG(INFO) << "[" << _serverName
            << "][LCM]: Message arrived on channel: " << chan;

  Json camera_info = Json::array();

  for (int i = 0; i < msg->n; i++) {
    lcmSensor::camera_desc camera_desc = msg->cameras[i];

    LOG(INFO) << "[" << _serverName << "][LCM]: Received camera status message"
              << " id: " << camera_desc.id
              << " product: " << camera_desc.product
              << " name: " << camera_desc.name << " fps: " << camera_desc.fps
              << " width: " << camera_desc.width
              << " height: " << camera_desc.height
              << " format: " << camera_desc.format
              << " status: " << camera_desc.status;

    Json camera_json;
    camera_json["id"] = camera_desc.id;
    camera_json["brand"] = camera_desc.product;
    camera_json["name"] = camera_desc.name;
    camera_json["width"] = camera_desc.width;
    camera_json["height"] = camera_desc.height;

    camera_info.push_back(camera_json);
  }

  Json params;
  params["camera_info"] = camera_info;

  reportProperties(params);
}
void CDeviceMonitorServer::reportProperties(const Json &params) {
  std::string topic = "dm/property/post";
  int64_t msg_id = getTimeMs();

  Json json_rpc_msg;
  json_rpc_msg["jsonrpc"] = "2.0";
  json_rpc_msg["id"] = std::to_string(msg_id);
  json_rpc_msg["method"] = "dm.property.post";
  json_rpc_msg["params"] = params;

  MessagePointer protocol_msg =
      std::make_shared<CJsonRpcMessage>(topic, getNonce(), json_rpc_msg.dump());

  _jsonrpcClient->send(protocol_msg);
}

std::string CDeviceMonitorServer::getOperatingSystem() {
#ifdef __APPLE__
  return "macOS";
#else
  try {
    std::ifstream file("/etc/os-release");
    if (!file.is_open()) { return "unknown"; }

    std::string line;
    while (std::getline(file, line)) {
      if (line.rfind("PRETTY_NAME", 0) == 0) {
        std::string version = line.substr(line.find('=') + 2);
        if (!version.empty() && version.back() == '"') {
          version.pop_back(); // Remove trailing quote
        }
        return version;
      }
    }
  } catch (const std::exception &e) {
    LOG(ERROR) << "Failed to get Ubuntu version: " << e.what();
  }
  return "unknown";
#endif
}

std::string CDeviceMonitorServer::getKernelVersion() {
  try {
    struct utsname buffer;
    if (uname(&buffer) != 0) { return "unknown"; }
    return std::string(buffer.release);
  } catch (const std::exception &e) {
    LOG(ERROR) << "Failed to get Kernel version: " << e.what();
    return "unknown";
  }
}

std::string CDeviceMonitorServer::getArchitecture() {
  try {
    struct utsname buffer;
    if (uname(&buffer) != 0) { return "unknown"; }

    std::string architecture(buffer.machine);

#ifdef __APPLE__
    if (architecture == "x86_64") {
      return "x86_64 (Intel)";
    } else if (architecture == "arm64") {
      return "arm64 (Apple Silicon)";
    } else {
      return architecture;
    }
#else
    return architecture;
#endif

  } catch (const std::exception &e) {
    LOG(ERROR) << "Failed to get Architecture: " << e.what();
    return "unknown";
  }
}

std::string CDeviceMonitorServer::getMemorySize() {
  try {
    int64_t physical_memory =
        sysconf(_SC_PHYS_PAGES) * sysconf(_SC_PAGE_SIZE) / 1024; // KB
    if (physical_memory <= 0) { return "unknown"; }
    return std::to_string(physical_memory);
  } catch (const std::exception &e) {
    LOG(ERROR) << "Failed to get Memory Size: " << e.what();
    return "unknown";
  }
}

std::string CDeviceMonitorServer::getMemoryUsed() {
#ifdef __APPLE__
  try {
    uint64_t total_mem;
    size_t len = sizeof(total_mem);
    if (sysctlbyname("hw.memsize", &total_mem, &len, NULL, 0) != 0) {
      throw std::runtime_error("Failed to get memory size");
    }

    mach_port_t host = mach_host_self();
    vm_statistics64_data_t vm_stat;
    unsigned int count = HOST_VM_INFO64_COUNT;
    vm_size_t page_size;

    if (host_page_size(host, &page_size) != KERN_SUCCESS
        || host_statistics64(host, HOST_VM_INFO64, (host_info64_t)&vm_stat,
                             &count)
               != KERN_SUCCESS) {
      throw std::runtime_error("Failed to get memory statistics");
    }

    uint64_t free = vm_stat.free_count * page_size;
    double mem_used = (double)(total_mem - free) / 1024;

    return std::to_string(mem_used);
  } catch (const std::exception &e) {
    LOG(ERROR) << "Failed to get Memory Used on Mac: " << e.what();
    return "unknown";
  }
#else
  try {
    std::ifstream file("/proc/meminfo");
    if (!file.is_open()) { return "unknown"; }

    std::string line;
    int64_t mem_total = 0;
    int64_t mem_available = 0;

    while (std::getline(file, line)) {
      if (line.compare(0, 9, "MemTotal:") == 0) {
        mem_total = std::stol(line.substr(line.find_first_of("0123456789")));
      }
      if (line.compare(0, 13, "MemAvailable:") == 0) {
        mem_available =
            std::stol(line.substr(line.find_first_of("0123456789")));
      }
    }

    int64_t mem_used = mem_total - mem_available;
    return std::to_string(mem_used < 0 ? 0 : mem_used);
  } catch (const std::exception &e) {
    LOG(ERROR) << "Failed to get Memory Used on Linux: " << e.what();
    return "unknown";
  }
#endif
}

std::vector<int64_t> CDeviceMonitorServer::getCpuTimes() {
  std::vector<int64_t> cpu_times(10, 0);
  try {
    std::ifstream file("/proc/stat");
    if (!file.is_open()) { return cpu_times; }

    std::string line;
    if (std::getline(file, line)) {
      std::istringstream iss(line);
      std::string cpu_label;
      iss >> cpu_label;

      int64_t time;
      cpu_times.clear();
      while (iss >> time) { cpu_times.push_back(time); }
    }
  } catch (const std::exception &e) {
    LOG(ERROR) << "Failed to get CPU Times: " << e.what();
  }
  return cpu_times;
}

std::string CDeviceMonitorServer::getCpuLoad() {
#ifdef __APPLE__
  try {
    host_name_port_t host = mach_host_self();
    mach_msg_type_number_t count;
    processor_cpu_load_info_t cpu_load;
    natural_t processor_count;

    if (host_processor_info(host, PROCESSOR_CPU_LOAD_INFO, &processor_count,
                            (processor_info_array_t *)&cpu_load, &count)
        != KERN_SUCCESS) {
      throw std::runtime_error("Failed to get CPU information");
    }

    sleep(1);

    processor_cpu_load_info_t prev_cpu_load = cpu_load;
    if (host_processor_info(host, PROCESSOR_CPU_LOAD_INFO, &processor_count,
                            (processor_info_array_t *)&cpu_load, &count)
        != KERN_SUCCESS) {
      throw std::runtime_error("Failed to get CPU information");
    }

    unsigned long long total = 0, used = 0;
    for (natural_t i = 0; i < processor_count; i++) {
      unsigned long long user = cpu_load[i].cpu_ticks[CPU_STATE_USER]
                                - prev_cpu_load[i].cpu_ticks[CPU_STATE_USER];
      unsigned long long system =
          cpu_load[i].cpu_ticks[CPU_STATE_SYSTEM]
          - prev_cpu_load[i].cpu_ticks[CPU_STATE_SYSTEM];
      unsigned long long nice = cpu_load[i].cpu_ticks[CPU_STATE_NICE]
                                - prev_cpu_load[i].cpu_ticks[CPU_STATE_NICE];
      unsigned long long idle = cpu_load[i].cpu_ticks[CPU_STATE_IDLE]
                                - prev_cpu_load[i].cpu_ticks[CPU_STATE_IDLE];

      used += user + system + nice;
      total += used + idle;
    }

    double percent = (total > 0) ? ((double)used * 100.0 / total) : 0.0;

    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2) << percent << "%";
    return ss.str();
  } catch (const std::exception &e) {
    LOG(ERROR) << "Failed to get CPU Load on Mac: " << e.what();
    return "0.00%";
  }
#else
  try {
    _currCpuTimes = getCpuTimes();

    if (_prevCpuTimes.size() != _currCpuTimes.size()) {
      _prevCpuTimes = _currCpuTimes;
      return "0.00%";
    }

    int64_t total_diff = 0;
    int64_t idle_diff = _currCpuTimes[3] - _prevCpuTimes[3];

    for (size_t i = 0; i < _currCpuTimes.size(); ++i) {
      total_diff += _currCpuTimes[i] - _prevCpuTimes[i];
    }

    if (total_diff == 0) { return "0.00%"; }

    _prevCpuTimes = _currCpuTimes;

    double cpuLoad = (1.0 - static_cast<double>(idle_diff) / total_diff) * 100;

    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2) << cpuLoad << "%";
    return ss.str();
  } catch (const std::exception &e) {
    LOG(ERROR) << "Failed to get CPU Load: " << e.what();
    return "0.00%";
  }
#endif
}

}}} // namespace rynnrcp::fw::robot
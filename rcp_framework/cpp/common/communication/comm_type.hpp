/**
 * @file comm_type.h
 * @brief Communication type definitions
 */

#pragma once

#include <glog/logging.h>
#include <httplib.h>
#include <libwebsockets.h>
#include <openssl/bio.h>
#include <openssl/buffer.h>
#include <openssl/hmac.h>
#include <pwd.h>
#include <sys/utsname.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <fstream>
#include <functional>
#include <future>
#include <json.hpp>
#include <lcm/lcm-cpp.hpp>
#include <map>
#include <memory>
#include <mutex>
#include <numeric>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "RobotServerCombinePacket.pb.h"
#include "RobotServerData.pb.h"
#include "RobotServerTransportPacket.pb.h"
#include "TunnelTransportService.pb.h"
#include "lcmMotion/act_command.hpp"
#include "lcmMotion/act_request.hpp"
#include "lcmMotion/robot_feedback.hpp"
#include "lcmMotion/state_feedback.hpp"
#include "lcmSensor/camera_ctrl.hpp"
#include "lcmSensor/camera_ctrl_response.hpp"
#include "lcmSensor/camera_desc.hpp"
#include "lcmSensor/camera_image_response.hpp"
#include "lcmSensor/camera_list_desc.hpp"
#include "lcmSensor/camera_status.hpp"
#include "lcmSensor/image_data.hpp"
#include "lcmSensor/req_camera_desc.hpp"
#include "lcmSensor/req_camera_image.hpp"
#include "lcmSensor/req_camera_status.hpp"

namespace rynnrcp { namespace fw { namespace common {

// Constants for communication ports
const int32_t kMqttPort = 1883;
const int32_t kWebsocketPort = 443;
const size_t kMaxUrlPartLength = 255;

// Constants for various limits and configurations
const int32_t kUrlMaxLen = 100;
const int32_t kUsernameMaxLen = 128;
const int32_t kPasswordMaxLen = 128;
const int32_t kMaxMsgBufSize = 128;
const int32_t kMqttConnTimeout = 5;
const int32_t kMqttKeepaliveInterval = 5;
const int32_t kTokenExpiredCode = 4001;

// Error codes for device occupy and release
const int32_t kDeviceBusyCode = 401;
const int32_t kDeviceNotOccupiedCode = 402;
const int32_t kJsonParseErrorCode = 400;

// Channel names for communication
const char kChannelCameraDesc[] = "camera_desc";
const char kChannelMotionCommand[] = "rcp_robotmotion";
const char kChannelRequestFeedback[] = "rcp_request_feedback";
const char kChannelActFeedback[] = "state_feedback";
const char kChannelRobotFeedback[] = "robot_feedback";
const char kChannelImageRequest[] = "image_request";
const char kChannelImageResponse[] = "image_response";

// WebSocket connection status
typedef enum {
  WS_CONN_DEFAULT = 0,
  WS_CONNECTED = 1,
  WS_DISCONNECT = 2,
  WS_CONN_ERR = 3,
} WebsocketConnFlag;

typedef enum { MQTT_QOS_0 = 0, MQTT_QOS_1 = 1, MQTT_QOS_2 = 2 } MqttQos;

// Communication protocol types
typedef enum {
  COMM_NONE = 0,
  COMM_JSONRPC = 1,
  COMM_DATA = 2,
  COMM_MQTT = 3,
  COMM_WEBSOCKET = 4,
} ProtocolType;

/**
 * @brief Base class for protocol messages
 */
class CProtocolMessage {
public:
  virtual ~CProtocolMessage() = default;
  virtual ProtocolType getProtocolType() = 0;
};

/**
 * @brief JSON-RPC class for protocol messages
 */
class CJsonRpcMessage : public CProtocolMessage {
private:
  std::string _topic;
  int32_t _msgid;
  std::string _payload;

public:
  CJsonRpcMessage(const std::string &topic, int32_t msgid,
                  const std::string &payload) {
    _topic = topic;
    _msgid = msgid;
    _payload = payload;
  }

  ProtocolType getProtocolType() override {
    return ProtocolType::COMM_JSONRPC;
  }

  const std::string &getTopic() const {
    return _topic;
  }

  int32_t getMsgid() const {
    return _msgid;
  }

  const std::string &getPayload() const {
    return _payload;
  }
};

/**
 * @brief CDataMessage class for protocol messages
 */
class CDataMessage : public CProtocolMessage {
private:
  int32_t _id;
  int32_t _type;
  std::string _data;

public:
  CDataMessage(int32_t id, int32_t type, const std::string &data) {
    _id = id;
    _type = type;
    _data = data;
  }

  ProtocolType getProtocolType() override {
    return ProtocolType::COMM_DATA;
  }

  int32_t getId() const {
    return _id;
  }

  int32_t getType() const {
    return _type;
  }

  const std::string &getData() const {
    return _data;
  }
};

/**
 * @brief Thread-safe bounded queue for managing items
 * @tparam T The type of items in the queue
 */
template <typename T>
class CBoundedQueue {
public:
  CBoundedQueue(size_t max_size) : _maxSize(max_size) {
    _size.store(0, std::memory_order_release);
  }

  void enqueue(const T &item) {
    std::unique_lock<std::mutex> lk(_mutex);

    // If queue is full, remove the oldest item before enqueueing new item
    if (_size.load(std::memory_order_acquire) >= _maxSize) {
      _queue.erase(_queue.begin());
    }

    _queue.push_back(item);
    _size.fetch_add(1);

    lk.unlock();
    _cv.notify_one();
  }

  T dequeue() {
    std::unique_lock<std::mutex> lk(_mutex);

    while (_size.load(std::memory_order_acquire) == 0) { _cv.wait(lk); }

    T item = _queue.front();
    _queue.erase(_queue.begin());
    _size.fetch_sub(1);

    return item;
  }

  bool empty() {
    std::lock_guard<std::mutex> lk(_mutex);
    return _size.load(std::memory_order_acquire) == 0;
  }

  size_t size() {
    std::lock_guard<std::mutex> lk(_mutex);
    return _size.load(std::memory_order_acquire);
  }

private:
  size_t _maxSize;
  std::atomic<size_t> _size;
  std::vector<T> _queue;
  std::mutex _mutex;
  std::condition_variable _cv;
};

/**
 * @brief Get the current time in milliseconds
 * @return Current time in milliseconds
 */
inline int64_t getTimeMs() {
  auto now = std::chrono::system_clock::now();
  auto milliseconds =
      std::chrono::time_point_cast<std::chrono::milliseconds>(now)
          .time_since_epoch()
          .count();
  return static_cast<int64_t>(milliseconds);
}

/**
 * @brief Get the current time in nanoseconds
 * @return Current time in nanoseconds
 */
inline int64_t getTimeNs() {
  auto now = std::chrono::high_resolution_clock::now();
  auto duration = now.time_since_epoch();
  return std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
}

/**
 * @brief Generate a nonce based on the current time in nanoseconds
 * @return A generated nonce
 */
inline int32_t getNonce() {
  return static_cast<int32_t>(getTimeNs() & (0xffffffff));
}

/**
 * @brief Create directories recursively if they do not exist
 * @param path The path to create directories for
 */
inline void createDirectories(const std::string &path) {
  size_t pos = 0;
  std::string current_path;

  while ((pos = path.find('/', pos)) != std::string::npos) {
    current_path = path.substr(0, pos++);

    if (!current_path.empty()) {
      if (mkdir(current_path.c_str(), 0777) == -1) {
        if (errno != EEXIST) {
          throw std::runtime_error("Error: " + std::string(strerror(errno))
                                   + " while creating directory: "
                                   + current_path);
        }
      }
    }
  }

  if (!current_path.empty()) {
    if (mkdir(path.c_str(), 0777) == -1) {
      if (errno != EEXIST) {
        throw std::runtime_error("Error: " + std::string(strerror(errno))
                                 + " while creating directory: " + path);
      }
    }
  }
}

/**
 * @brief Expand leading '~' in path to the user's home directory
 * @param path The path to get absolute path
 */
inline std::string expandTilde(const std::string &path) {
  if (path.empty()) { return path; }

  if (path[0] == '~') {
    const char *homeDir = getpwuid(getuid())->pw_dir;
    return std::string(homeDir) + path.substr(1);
  }

  return path;
}

/**
 * @brief Initializes Google logging with the specified configuration file
 * @param config_file Path to the YAML configuration file
 */
inline int32_t initGlogByConfig(const std::string &config_file) {
  try {
    YAML::Node config = YAML::LoadFile(config_file);

    std::string log_dir = config["log_dir"].as<std::string>();
    std::string log_name = config["server_node_log_name"].as<std::string>();

    log_dir = expandTilde(log_dir) + "/" + log_name + "/";
    if (log_dir.find(' ') != std::string::npos) {
      throw std::runtime_error("Error: Log dir " + log_dir + " contains space");
    }

    createDirectories(log_dir);

    google::InitGoogleLogging(log_name.c_str());
    std::string file_path = log_dir + log_name;
    google::SetLogDestination(google::GLOG_INFO, file_path.c_str());
    google::SetLogFilenameExtension(".log");

    FLAGS_logbufsecs = config["log_buf_secs"].as<int32_t>();
    FLAGS_minloglevel = config["min_log_level"].as<int32_t>();
    FLAGS_max_log_size = config["max_log_size"].as<int32_t>();
    FLAGS_stderrthreshold = config["stderr_threshold"].as<int32_t>();

    LOG(WARNING) << "Server log file: " << file_path << ".INFO";
  } catch (const std::exception &e) { std::cerr << e.what() << '\n'; }

  return 0;
}

// Type aliases
using Json = nlohmann::json;
using MessagePointer = std::shared_ptr<CProtocolMessage>;
using MessageQueue = CBoundedQueue<MessagePointer>;

}}} // namespace rynnrcp::fw::common

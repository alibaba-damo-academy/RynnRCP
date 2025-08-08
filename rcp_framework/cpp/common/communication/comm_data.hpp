/**
 * @file comm_data.hpp
 * @brief Abstract communication interface CDataComm
 */

#pragma once

#include "auth_manager.hpp"
#include "comm_type.hpp"

namespace rynnrcp { namespace fw { namespace common {

/**
 * @class CDataComm
 * @brief Abstract base class for data communication
 *
 * This class provides an interface for various stream communications
 */
class CDataComm {
public:
  virtual ~CDataComm() = default;

  /**
   * @brief Initialize the communication with the provided configuration
   * @param config_file Configuration file path
   */
  virtual void init(const std::string &config_file) = 0;

  /**
   * @brief Start the communication process
   */
  virtual void start() = 0;

  /**
   * @brief Close the communication session
   */
  virtual void close() = 0;

  /**
   * @brief Send a message through the communication channel
   * @param msg Message to be sent
   */
  virtual void send(MessagePointer &msg) = 0;

  /**
   * @brief Bind a message queue to the communication channel
   * @param recv_buffer The message queue to bind
   */
  virtual void bindRecvBuffer(MessageQueue *recv_buffer) = 0;
};

}}} // namespace rynnrcp::fw::common

/**
 * @file auth_manager.hpp
 * @brief Header file for CAuthManager class
 */

#pragma once

#include "comm_type.hpp"

namespace rynnrcp { namespace fw { namespace common {

/**
 * @class CAuthManager
 * @brief Manages device authentication and configuration parsing
 *
 * Responsibilities:
 * - Load device configuration from a YAML file
 * - Authenticate device and retrieve AK/SK
 * - Parse response data into configuration
 * - Provide MQTT and WebSocket configurations as JSON
 */
class CAuthManager {
public:
  /**
   * @brief Constructor
   * @param config_filename Path to YAML configuration file
   * @throws std::runtime_error if file is missing or invalid
   */
  explicit CAuthManager(const std::string &config_filename);

  /**
   * @brief Get MQTT configuration as JSON
   * @return A JSON object containing the MQTT configuration
   */
  Json getMqttConfig();

  /**
   * @brief Get WebSocket configuration as JSON
   * @return A JSON object containing the WebSocket configuration
   */
  Json getWsConfig();

private:
  /**
   * @brief Load configuration from a YAML file
   * @param config_filename Path to the YAML file
   * @throws std::runtime_error if file is missing or invalid
   */
  void loadConfig(const std::string &config_filename);

  /**
   * @brief Generate signature content
   * @param auth_key Authorization key
   * @param timestamp Current timestamp
   * @param nonce Unique nonce value
   * @param ext_args Additional parameters
   * @return Generated signature content
   */
  std::string genSignContent(
      const std::string &auth_key, const std::string &timestamp,
      const std::string &nonce,
      const std::map<std::string, std::string> &ext_args);

  /**
   * @brief Compute HMAC SHA1 hash
   * @param content Content to hash
   * @param secret Secret key
   * @return HMAC SHA1 hash
   */
  std::string hmacSha1(const std::string &content, const std::string &secret);

  /**
   * @brief Generate a signature for authentication
   * @param auth_key Authorization key
   * @param timestamp Current timestamp
   * @param nonce Unique nonce value
   * @param ext_args Additional parameters
   * @param secret Secret key
   * @return Generated signature
   */
  std::string genSignature(const std::string &auth_key,
                           const std::string &timestamp,
                           const std::string &nonce,
                           const std::map<std::string, std::string> &ext_args,
                           const std::string &secret);

private:
  // Authentication configuration
  Json _payload;                  ///< JSON payload
  std::string _httpUrl;           ///< Base HTTP URL (without endpoints)
  std::string _endpointMqtt;      ///< MQTT HTTP endpoint path
  std::string _endpointWebsocket; ///< WebSocket HTTP endpoint path
  std::string _productKey;        ///< Product key
  std::string _deviceName;        ///< Device name
  std::string _deviceSecret;      ///< Device secret

  // MQTT client configuration
  std::string _deviceAk;   ///< Device Access Key (AK)
  std::string _deviceSk;   ///< Device Secret Key (SK)
  std::string _clientId;   ///< Client ID
  std::string _mqttHost;   ///< MQTT server host
  std::string _instanceId; ///< Instance ID
  int32_t _mqttPort;       ///< MQTT server port

  // WebSocket client configuration
  int32_t _expire;            ///< Token expiration time
  int32_t _websocketPort;     ///< WebSocket port
  std::string _token;         ///< Auth token from the service
  std::string _websocketUrl;  ///< Full WebSocket URL
  std::string _websocketHost; ///< WebSocket host
  std::string _websocketPath; ///< WebSocket path
};

}}} // namespace rynnrcp::fw::common

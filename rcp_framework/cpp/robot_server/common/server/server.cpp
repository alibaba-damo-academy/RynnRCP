/**
 * @file server.cpp
 * @brief Implementation of the base server
 */

#include "server.hpp"

namespace rynnrcp { namespace fw { namespace robot {

CServer::CServer(const std::string &name,
                 const std::shared_ptr<CJsonRpcComm> jsonrpc_client,
                 const std::shared_ptr<CDataComm> data_client) :
    _serverName(name), _jsonrpcClient(jsonrpc_client),
    _dataClient(data_client) {
  _recvBuffer = std::make_shared<MessageQueue>(kMaxMsgBufSize);

  _dataClient->bindRecvBuffer(_recvBuffer.get());
  _jsonrpcClient->bindRecvBuffer(_recvBuffer.get());

  _running.store(false, std::memory_order_release);
  _occupied.store(false, std::memory_order_release);
}

CServer::~CServer() {
  stop();
}

// Implemented in CTerminalDeviceServer class
int32_t CServer::handleMessage(MessagePointer msg) {
  return 0;
}

bool CServer::start() {
  if (_running.load(std::memory_order_acquire)) { return true; }

  _running.store(true, std::memory_order_release);

  _bufferMonitorThread = std::thread([this]() {
    while (_running.load(std::memory_order_acquire)) {
      if (!_recvBuffer->empty()) {
        auto msg = _recvBuffer->dequeue();
        handleMessage(msg);
      }

      std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
  });

  return true;
}

void CServer::stop() {
  if (!_running.load(std::memory_order_acquire)) { return; }

  _running.store(false, std::memory_order_release);

  if (_bufferMonitorThread.joinable()) { _bufferMonitorThread.join(); }
}

}}} // namespace rynnrcp::fw::robot
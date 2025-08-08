/**
 * @file comm_websocket.cpp
 * @brief Implementation of CWebsocketComm class
 */

#include "comm_websocket.hpp"

namespace rynnrcp { namespace fw { namespace common {

CWebsocketComm::CWebsocketComm() {
}

CWebsocketComm::~CWebsocketComm() {
  close();
}
void CWebsocketComm::init(const std::string &config_file) {
  std::unique_lock<std::mutex> lk(_mutex);

  _authClient = std::make_shared<CAuthManager>(config_file);

  Json config = _authClient->getWsConfig();
  try {
    _token = config["token"].get<std::string>();
    _expire = config["expire"].get<int32_t>();
    _websocketHost = config["websocket_host"].get<std::string>();
    _websocketPath = config["websocket_path"].get<std::string>();
    _websocketPort = config["websocket_port"].get<int32_t>();
  } catch (const std::exception &e) {
    throw std::runtime_error("[WebSocket]: Config parsing failed: "
                             + std::string(e.what()));
  }

  LOG(INFO) << "[WebSocket]: WebSocket token: " << _token
            << ", host: " << _websocketHost << ", path: " << _websocketPath
            << ", port: " << _websocketPort;

  static const struct lws_protocols protocols[] = {
      {"lws-mirror-protocol", &onWebsocketEvent, 0, 1024 * 1024, 0, NULL,
       1024 * 1024},
      {NULL, NULL, 0, 0, 0, NULL, 0}};

  lws_set_log_level(LLL_ERR | LLL_WARN, NULL);

  memset(&_contextInfo, 0, sizeof(_contextInfo));
  _contextInfo.port = CONTEXT_PORT_NO_LISTEN;
  _contextInfo.protocols = protocols;
  _contextInfo.pt_serv_buf_size = 2 * 1024 * 1024;
  _contextInfo.timeout_secs = 60 * 60;
  _contextInfo.options = LWS_SERVER_OPTION_DO_SSL_GLOBAL_INIT;

  _context = lws_create_context(&_contextInfo);
  if (NULL == _context) {
    throw std::runtime_error("[WebSocket]: Create LWS context failed");
  }

  memset(&_connectInfo, 0, sizeof(_connectInfo));
  _connectInfo.context = _context;
  _connectInfo.port = _websocketPort;
  _connectInfo.address = _websocketHost.c_str();
  _connectInfo.path = _websocketPath.c_str();
  _connectInfo.host = _connectInfo.address;
  _connectInfo.protocol = protocols[0].name;
  _connectInfo.pwsi = &_wsInterface;
  _connectInfo.userdata = this;
  _connectInfo.ssl_connection = LCCSCF_USE_SSL | LCCSCF_ALLOW_SELFSIGNED
                                | LCCSCF_SKIP_SERVER_CERT_HOSTNAME_CHECK
                                | LCCSCF_ALLOW_INSECURE;

  _websocketConnFlag.store(WS_CONN_DEFAULT, std::memory_order_release);
  _clientWritable.store(false, std::memory_order_release);
}

int32_t CWebsocketComm::onMessageArrived(void *user, void *in, int len) {
  CWebsocketComm *me = reinterpret_cast<CWebsocketComm *>(user);

  RobotServer::DataPacket data_packet_proto;

  if (!data_packet_proto.ParseFromArray(in, len)) {
    LOG(ERROR) << "[WebSocket]: Received data format is not DataPacket";
    return -1;
  }

  RobotServer::CommonPartAttr attr = data_packet_proto.common_part_attr();

  LOG(INFO) << "[WebSocket]: Message arrived, type: " << attr.type();

  MessagePointer tmp_msg = std::make_shared<CDataMessage>(
      attr.id(), attr.type(), data_packet_proto.data());

  for (auto &recv_buf : me->_recvBuffer) { recv_buf->enqueue(tmp_msg); }

  return 0;
}

int CWebsocketComm::onWebsocketEvent(struct lws *wsi,
                                     enum lws_callback_reasons reason,
                                     void *user, void *in, size_t len) {
  CWebsocketComm *me = reinterpret_cast<CWebsocketComm *>(user);

  switch (reason) {
  case LWS_CALLBACK_CLIENT_RECEIVE: {
    LOG(INFO) << "[WebSocket]: Callback LWS_CALLBACK_CLIENT_RECEIVE";
    me->onMessageArrived(user, in, (int)len);
  } break;
  case LWS_CALLBACK_CLIENT_WRITEABLE: {
    LOG(INFO) << "[WebSocket]: Callback LWS_CALLBACK_CLIENT_WRITEABLE";
    me->_clientWritable.store(true, std::memory_order_release);
  } break;
  case LWS_CALLBACK_CLIENT_ESTABLISHED: {
    LOG(INFO) << "[WebSocket]: Callback LWS_CALLBACK_CLIENT_ESTABLISHED";
    me->onConnectionStateChanged(WS_CONNECTED);
    lws_callback_on_writable(me->_wsInterface);
  } break;

  case LWS_CALLBACK_CLIENT_CLOSED: {
    LOG(INFO) << "[WebSocket]: Callback LWS_CALLBACK_CLIENT_CLOSED";
    me->onConnectionStateChanged(WS_DISCONNECT);
    me->_clientWritable.store(false, std::memory_order_release);
  } break;

  case LWS_CALLBACK_CLIENT_CONNECTION_ERROR: {
    LOG(ERROR) << "[WebSocket]: Callback LWS_CALLBACK_CLIENT_CONNECTION_ERROR";
    me->onConnectionStateChanged(WS_CONN_ERR);
    me->_clientWritable.store(false, std::memory_order_release);
  } break;

  case LWS_CALLBACK_CLIENT_APPEND_HANDSHAKE_HEADER: {
    unsigned char **p = (unsigned char **)in;
    unsigned char *end = (*p) + len;

    const char *robot_id_header = "tunnel-access-token:";
    const char *robot_id_value = me->_token.c_str();
    if (lws_add_http_header_by_name(wsi, (const unsigned char *)robot_id_header,
                                    (const unsigned char *)robot_id_value,
                                    strlen(robot_id_value), p, end)) {
      return -1;
    }
  } break;
  case LWS_CALLBACK_WS_PEER_INITIATED_CLOSE: {
    if (len >= 2) {
      uint16_t close_code = ntohs(*(uint16_t *)in);
      if (close_code == kTokenExpiredCode) {
        LOG(INFO) << "[WebSocket]: Token expired, remote closed connection.";

        std::unique_lock<std::mutex> lk(me->_mutex);

        Json config = me->_authClient->getWsConfig();
        try {
          me->_token = config["token"].get<std::string>();
          me->_expire = config["expire"].get<int32_t>();
          LOG(INFO) << "[WebSocket]: Get new WebSocket token: " << me->_token;
        } catch (const std::exception &e) {
          throw std::runtime_error(e.what());
        }
      } else {
        LOG(INFO) << "[WebSocket]: Closed by remote end with code: "
                  << close_code;
      }
      return 0;
    } else {
      return 1;
    }
  }
  case LWS_CALLBACK_WSI_DESTROY:
    LOG(INFO) << "[WebSocket]: Callback LWS_CALLBACK_WSI_DESTROY";
    break;
  default: break;
  }
  return 0;
}

void CWebsocketComm::onConnectionStateChanged(WebsocketConnFlag flag) {
  _websocketConnFlag.store(flag, std::memory_order_release);
}

void CWebsocketComm::start() {
  _wsInterface = lws_client_connect_via_info(&_connectInfo);
  if (NULL == _wsInterface) {
    lws_context_destroy(_context);
    throw std::runtime_error("[WebSocket]: Connect server failed");
  }

  _eventLoopThread = std::thread([this]() {
    while (_wsInterface && _context) {
      lws_service(_context, 1000);
      if (WS_CONNECTED != _websocketConnFlag.load(std::memory_order_acquire)
          && WS_CONN_DEFAULT
                 != _websocketConnFlag.load(std::memory_order_acquire)) {
        reconnect();
      }
    }
  });
}

void CWebsocketComm::reconnect() {
  if (!_wsInterface) { return; }

  LOG(INFO) << "[WebSocket]: Trying to reconnect";
  std::unique_lock<std::mutex> lk(_mutex);

  struct timeval tv;
  gettimeofday(&tv, NULL);
  int64_t gap_time = 1; // 1 second
  static int64_t g_connect_timestamp_s = 0L;
  if (tv.tv_sec > g_connect_timestamp_s + gap_time) {
    g_connect_timestamp_s = tv.tv_sec;
    _wsInterface = lws_client_connect_via_info(&_connectInfo);
    if (NULL == _wsInterface) {
      lws_context_destroy(_context);
      LOG(ERROR) << "[WebSocket]: Reconnect server failed";
    }
  }
}

void CWebsocketComm::close() {
  if (_wsInterface == NULL && _context == NULL) { return; }

  {
    std::unique_lock<std::mutex> lk(_mutex);
    if (_context != NULL) {
      lws_context_destroy(_context);
      _context = NULL;
    }
    _wsInterface = NULL;
  }

  if (_eventLoopThread.joinable()) { _eventLoopThread.join(); }
}

void CWebsocketComm::send(MessagePointer &msg) {
  if (_clientWritable.load(std::memory_order_acquire) == false) {
    LOG(ERROR) << "[WebSocket]: WebSocket client not writable, send failed";
    return;
  }

  std::unique_lock<std::mutex> lk(_mutex);

  auto ws_msg = std::dynamic_pointer_cast<CDataMessage>(msg);
  if (ws_msg->getProtocolType() != COMM_DATA) {
    LOG(ERROR) << "[WebSocket]: Invalid message type, expected CDataMessage";
    return;
  }

  int64_t ts = getTimeMs();
  RobotServer::DataPacket data_packet;
  RobotServer::CommonPartAttr *common_attr =
      data_packet.mutable_common_part_attr();
  common_attr->set_timestamp(ts);
  common_attr->set_id(ws_msg->getId());
  common_attr->set_type(
      static_cast<RobotServer::PackageType>(ws_msg->getType()));
  data_packet.set_data(ws_msg->getData());

  std::string data = data_packet.SerializeAsString();

  LOG(INFO) << "[WebSocket]: Message send size: "
            << float(data.size()) / 1024 / 1024
            << " MB, packet.type: " << ws_msg->getType();

  std::string buf(LWS_PRE + data.size(), '\0');
  std::memcpy(&buf[LWS_PRE], data.data(), data.size());
  lws_write(_wsInterface, reinterpret_cast<unsigned char *>(&buf[LWS_PRE]),
            data.size(), LWS_WRITE_BINARY);
}

void CWebsocketComm::bindRecvBuffer(MessageQueue *recv_buffer) {
  std::unique_lock<std::mutex> lk(_mutex);
  _recvBuffer.push_back(recv_buffer);
}
}}} // namespace rynnrcp::fw::common

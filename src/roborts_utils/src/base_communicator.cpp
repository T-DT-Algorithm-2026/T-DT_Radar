#include "base_communicator.h"

#include <memory>

#include "base_taskscheduler.h"

namespace tdttoolkit {
UDPServer::UDPServer(
    int port, std::function<bool(std::string, int)> new_connection_callback) {
  init(port, new_connection_callback);
}

void UDPServer::init(
    int port, std::function<bool(std::string, int)> new_connection_callback) {
  port_ = port;
  io_context_ = std::make_unique<boost::asio::io_context>();
  socket_ = std::make_unique<boost::asio::ip::udp::socket>(*io_context_);
  socket_->open(boost::asio::ip::udp::v4());
  auto self_endpoint =
      boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), port);
  socket_->bind(self_endpoint);
  if (new_connection_callback != nullptr) {
    new_connection_callback_ = new_connection_callback;
  }
  TaskScheduler heart_check_scheduler, recv_scheduler;
  heart_check_scheduler.async_run(1.0 / 10, &UDPServer::heart_check, this);
  recv_scheduler.async_run(1.0 / 10, &UDPServer::recv_handler, this);
  init_done_ = true;
}

void UDPServer::heart_check() {
  for (auto client : client_info_) {
    if (Time::GetTimeNow() - client->last_heart_time > TDT_UDP_HEART_TIME_OUT) {
      client->online = false;
      if (client->offline_callback_set) {
        client->offline_callback(client->ip, client->port);
      }
      client_info_.erase(
          std::find(client_info_.begin(), client_info_.end(), client));
    }
  }
}

void UDPServer::recv_handler() {
  boost::asio::ip::udp::endpoint sender_endpoint;
  size_t len = 0;
  auto recv_buf_ = std::make_unique<char[]>(TDT_UDP_MSG_MAX_LEN);
  try {
    len = socket_->receive_from(
        boost::asio::buffer(recv_buf_.get(), TDT_UDP_MSG_MAX_LEN),
        sender_endpoint);
  } catch (boost::exception const& ex) {
    return;
  } catch (...) {
    return;
  }
  auto client = std::find_if(
      client_info_.begin(), client_info_.end(),
      [sender_endpoint](std::shared_ptr<ClientInfo> client) {
        return client->ip == sender_endpoint.address().to_string() &&
               client->port == sender_endpoint.port();
      });
  if (client == client_info_.end()) {
    if (new_connection_callback_ != nullptr &&
        new_connection_callback_(sender_endpoint.address().to_string(),
                                 sender_endpoint.port())) {
      auto new_client = std::make_shared<ClientInfo>();
      new_client->ip = sender_endpoint.address().to_string();
      new_client->port = sender_endpoint.port();
      new_client->last_heart_time = Time::GetTimeNow();
      new_client->online = true;
      client_info_.push_back(new_client);
      client = client_info_.end() - 1;
    } else {
      return;
    }
  }
  if (len > 0 && std::string(recv_buf_.get(), len) == TDT_UDP_HEART_MSG) {
    (*client)->last_heart_time = Time::GetTimeNow();
    return;
  } else if (len > 0) {
    auto msg = std::make_shared<UDPMessage>();
    msg->ip = sender_endpoint.address().to_string();
    msg->port = sender_endpoint.port();
    msg->msg = std::string(recv_buf_.get(), len);
    (*client)->last_heart_time = Time::GetTimeNow();
    (*client)->msg_callback(msg);
    return;
  }
}

UDPServer::UDPServerHandle UDPServer::add_client(
    std::string ip, int port,
    std::function<void(std::shared_ptr<UDPMessage>)> msg_callback,
    std::function<void(std::string, int)> offline_callback) {
  auto client = std::make_shared<ClientInfo>();
  client->ip = ip;
  client->port = port;
  client->msg_callback = msg_callback;
  client->offline_callback = offline_callback;
  if (offline_callback != nullptr) {
    client->offline_callback_set = true;
  }
  client_info_.push_back(client);
  return UDPServer::UDPServerHandle(client);
}

bool UDPServer::send(std::string ip, int port, std::string msg) {
  try {
    boost::asio::ip::udp::endpoint endpoint(
        boost::asio::ip::address::from_string(ip), port);
    socket_->send_to(boost::asio::buffer(msg), endpoint);
    return true;
  } catch (...) {
    return false;
  }
}

bool UDPServer::send(UDPServerHandle client, std::string msg) {
  try {
    boost::asio::ip::udp::endpoint endpoint(
        boost::asio::ip::address::from_string(client->ip), client->port);
    socket_->send_to(boost::asio::buffer(msg), endpoint);
    return true;
  } catch (...) {
    return false;
  }
}

UDPClient::UDPClient(int port) { init(port); }

void UDPClient::init(int port) {
  port_ = port;
  io_context_ = std::make_unique<boost::asio::io_context>();
  socket_ = std::make_unique<boost::asio::ip::udp::socket>(*io_context_);
  socket_->open(boost::asio::ip::udp::v4());
  auto self_endpoint =
      boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), port);
  socket_->bind(self_endpoint);
  TaskScheduler heart_send_scheduler, recv_scheduler;
  heart_send_scheduler.async_run(1.0 / TDT_UDP_HEART_FREQUENCY,
                                 &UDPClient::heart_send, this);
  recv_scheduler.async_run(1.0 / 10, &UDPClient::recv_handler, this);
  init_done_ = true;
}

void UDPClient::heart_send() {
  for (auto& server : server_info_) {
    send(server->ip, server->port, TDT_UDP_HEART_MSG);
  }
}

void UDPClient::recv_handler() {
  boost::asio::ip::udp::endpoint sender_endpoint;
  size_t len = 0;
  auto recv_buf_ = std::make_unique<char[]>(TDT_UDP_MSG_MAX_LEN);
  try {
    len = socket_->receive_from(
        boost::asio::buffer(recv_buf_.get(), TDT_UDP_MSG_MAX_LEN),
        sender_endpoint);
  } catch (boost::exception const& ex) {
    return;
  } catch (...) {
    return;
  }
  auto client = std::find_if(
      server_info_.begin(), server_info_.end(),
      [sender_endpoint](std::shared_ptr<ServerInfo> client) {
        return client->ip == sender_endpoint.address().to_string() &&
               client->port == sender_endpoint.port();
      });
  if (client == server_info_.end()) {
    return;
  }
  if (len > 0 && std::string(recv_buf_.get(), len) == TDT_UDP_HEART_MSG) {
    (*client)->last_heart_time = Time::GetTimeNow();
    return;
  } else if (len > 0) {
    auto msg = std::make_shared<UDPMessage>();
    msg->ip = sender_endpoint.address().to_string();
    msg->port = sender_endpoint.port();
    msg->msg = std::string(recv_buf_.get(), len);
    (*client)->last_heart_time = Time::GetTimeNow();
    (*client)->msg_callback(msg);
    return;
  }
}

UDPClient::UDPClientHandle UDPClient::add_server(
    std::string ip, int port,
    std::function<void(std::shared_ptr<UDPMessage>)> msg_callback) {
  auto server = std::make_shared<ServerInfo>();
  server->ip = ip;
  server->port = port;
  server->msg_callback = msg_callback;
  server_info_.push_back(server);
  return UDPClient::UDPClientHandle(server);
}

bool UDPClient::send(std::string ip, int port, std::string msg) {
  try {
    boost::asio::ip::udp::endpoint endpoint(
        boost::asio::ip::address::from_string(ip), port);
    socket_->send_to(boost::asio::buffer(msg), endpoint);
    return true;
  } catch (...) {
    return false;
  }
}

bool UDPClient::send(UDPClientHandle server, std::string msg) {
  try {
    boost::asio::ip::udp::endpoint endpoint(
        boost::asio::ip::address::from_string(server->ip), server->port);
    socket_->send_to(boost::asio::buffer(msg), endpoint);
    return true;
  } catch (...) {
    return false;
  }
}

}  // namespace tdttoolkit
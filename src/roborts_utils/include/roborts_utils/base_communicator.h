/**
 * @Name: base_communicator,h
 * @Description: 通信类
 * @Version: 1.0.0.1
 * @Author: 20视觉严俊涵
 * @Date: 2023-05-24 14：51：00
 * @LastEditors: 20视觉严俊涵
 * @LastEditTime: 2023-05-24 14：51：00
 */

#ifndef __BASE_COMMUNICAROR_H
#define __BASE_COMMUNICAROR_H
#include <boost/asio.hpp>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#define TDT_UDP_HEART_TIME_OUT 3
#define TDT_UDP_HEART_FREQUENCY 2
#define TDT_UDP_MSG_MAX_LEN 67108864
#define TDT_UDP_HEART_MSG "tdt_udp_heart"

namespace tdttoolkit {
struct UDPMessage {
  std::string ip;
  int port;
  std::string msg;
};

class UDPServer {
 public:
  struct ClientInfo {
    std::string ip = "0.0.0.0";
    int port = 0;
    double last_heart_time = 0;
    bool online = false;

    std::function<void(std::shared_ptr<UDPMessage>)> msg_callback =
        nullptr;  // 接收消息回调函数

    bool offline_callback_set = false;
    std::function<void(std::string, int)>
        offline_callback;  // 超时回调函数 传入ip和port
  };

  class UDPServerHandle {
   private:
    std::shared_ptr<ClientInfo> handle;

   public:
    UDPServerHandle(std::shared_ptr<ClientInfo> handle) : handle(handle) {}
    ~UDPServerHandle() {}
    std::shared_ptr<ClientInfo> operator->() { return handle; }
    std::string ip() { return handle->ip; }
    int port() { return handle->port; }
  };

 private:
  int port_;
  std::unique_ptr<boost::asio::io_context> io_context_;
  std::unique_ptr<boost::asio::ip::udp::socket> socket_;
  std::vector<std::shared_ptr<ClientInfo>> client_info_;

  std::function<bool(std::string, int)> new_connection_callback_ =
      nullptr;  // 新连接回调函数, 若返回True则添加到client_info中

  bool init_done_ = false;

  void heart_check();

  void recv_handler();

 public:
  UDPServer(){};

  UDPServer(
      int port,
      std::function<bool(std::string, int)> new_connection_callback = nullptr);

  /******
   * @name: init
   * @description: 初始化
   * @param {int} port 端口号
   * @param {std::function<bool(std::string, int)>} new_connection_callback
   * 新连接回调函数, 若返回True则添加到客户端列表中
   */
  void init(
      int port,
      std::function<bool(std::string, int)> new_connection_callback = nullptr);

  /******
   * @name: add_client
   * @description: 添加客户端
   * @param {std::string} ip ip地址
   * @param {int} port 端口号
   * @return {UDPServerHandle} 客户端句柄
   */
  UDPServerHandle add_client(
      std::string ip, int port,
      std::function<void(std::shared_ptr<UDPMessage>)> msg_callback,
      std::function<void(std::string, int)> offline_callback = nullptr);

  /******
   * @name: send
   * @description: 发送消息
   * @param {std::string} ip ip地址
   * @param {int} port 端口号
   * @param {std::string} msg 消息
   * @return {bool} 是否发送成功
   */
  bool send(std::string ip, int port, std::string msg);

  /******
   * @name: send
   * @description: 发送消息
   * @param {UDPServer::UDPServerHandle} client 客户端句柄
   * @param {std::string} msg 消息
   * @return {bool} 是否发送成功
   */
  bool send(UDPServerHandle client, std::string msg);
};

typedef std::shared_ptr<UDPMessage> UDPMessagePtr;

class UDPClient {
 public:
  struct ServerInfo {
    std::string ip = "0.0.0.0";
    int port = 0;
    double last_heart_time = 0;

    std::function<void(std::shared_ptr<UDPMessage>)> msg_callback =
        nullptr;  // 接收消息回调函数
  };

  class UDPClientHandle {
   private:
    std::shared_ptr<ServerInfo> handle;

   public:
    UDPClientHandle(std::shared_ptr<ServerInfo> handle) : handle(handle) {}
    ~UDPClientHandle() {}
    std::shared_ptr<ServerInfo> operator->() { return handle; }
    std::string ip() { return handle->ip; }
    int port() { return handle->port; }
  };

 private:
  int port_;
  std::unique_ptr<boost::asio::io_context> io_context_;
  std::unique_ptr<boost::asio::ip::udp::socket> socket_;

  std::vector<std::shared_ptr<ServerInfo>> server_info_;

  bool init_done_ = false;

  void heart_send();

  void recv_handler();

 public:
  UDPClient(){};

  UDPClient(int port);

  /******
   * @name: init
   * @description: 初始化
   * @param {int} port 端口号
   * @param {std::function<bool(std::string, int)>} new_connection_callback
   * 新连接回调函数, 若返回True则添加到客户端列表中
   */
  void init(int port);

  /******
   * @name: add_client
   * @description: 添加服务端
   * @param {std::string} ip ip地址
   * @param {int} port 端口号
   * @return {UDPServerHandle} 客户端句柄
   */
  UDPClientHandle add_server(
      std::string ip, int port,
      std::function<void(std::shared_ptr<UDPMessage>)> msg_callback);

  /******
   * @name: send
   * @description: 发送消息
   * @param {std::string} ip ip地址
   * @param {int} port 端口号
   * @param {std::string} msg 消息
   * @return {bool} 是否发送成功
   */
  bool send(std::string ip, int port, std::string msg);

  /******
   * @name: send
   * @description: 发送消息
   * @param {UDPServer::UDPServerHandle} client 客户端句柄
   * @param {std::string} msg 消息
   * @return {bool} 是否发送成功
   */
  bool send(UDPClientHandle client, std::string msg);
};
}  // namespace tdttoolkit
#endif
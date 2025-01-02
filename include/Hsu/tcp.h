#pragma once

// ---- third ----
#include <boost/asio.hpp>
#include <boost/asio/steady_timer.hpp>
#include <nlohmann/json.hpp>  // JSON library
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/spdlog.h>
// ---- standard ----
#include <functional>
#include <string>

namespace Hsu {
class TCPConnection {
 public:
  using tcp = boost::asio::ip::tcp;
  using json = nlohmann::json;
  TCPConnection(const std::string& host, int port);

  struct Request {
    std::string cmd;
    int code;
    json payload;
  };

  struct Response {
    int code;
    json payload;
  };

  // Start as a server
  void start_server();

  // Start as a client
  void start_client();

  void stop_server();  // 新增安全停止服务器的方法

  // Register a command handler
  void connect(const std::string& cmd, std::function<Response(int, const json&)> handler);

  // Send a message to the server or connected client
  json send_message(const std::string& cmd, int code, const json& payload);

  // Stop the server or client
  void close();

  ~TCPConnection();

 private:
  std::string host_;
  int port_;
  boost::asio::io_context io_context_;
  tcp::acceptor acceptor_;
  tcp::socket socket_;
  bool is_server_;
  std::unordered_map<std::string, std::function<Response(int, const json&)>> handlers_;

  std::atomic<bool> running_;  // 新增用于标记服务器状态的变量
  std::thread server_thread_;

  void accept_connections();

  void handle_client(tcp::socket client_socket);
};

namespace detail {
std::shared_ptr<spdlog::logger> tcp_logger();
}
}  // namespace Hsu
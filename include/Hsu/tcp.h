#pragma once

// ---- third ----
#include <boost/asio.hpp>
#include <boost/asio/steady_timer.hpp>
#include <nlohmann/json.hpp>
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

  void start_server();

  void start_client();

  void stop_server();  // 新增安全停止服务器的方法

  void connect(const std::string& cmd, std::function<Response(int, const json&)> handler);

  json send_message(const std::string& cmd, int code, const json& payload);

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

  std::atomic<bool> running_;
  std::thread server_thread_;

  void accept_connections();

  void handle_client(tcp::socket client_socket);
};
}  // namespace Hsu
#include <Hsu/hsu_module_log.h>
#include <Hsu/tcp.h>
#include <boost/asio/error.hpp>
#include <fmt/base.h>
#include <future>
#include <thread>
#include <unordered_map>

using json = nlohmann::json;

GENERATE_LOGGER(tcp)

namespace Hsu {
TCPConnection::TCPConnection(const std::string& host, int port)
    : host_(host), port_(port), io_context_(), acceptor_(io_context_), socket_(io_context_), is_server_(false) {}

void TCPConnection::start_server() {
  is_server_ = true;
  running_ = true;  // 标记服务器正在运行
  tcp::endpoint endpoint(tcp::v4(), port_);
  acceptor_.open(endpoint.protocol());
  acceptor_.set_option(boost::asio::socket_base::reuse_address(true));
  acceptor_.bind(endpoint);
  acceptor_.listen();
  INFO("TCP服务开启在：[{}:{}]", host_, port_);

  server_thread_ = std::thread([this]() {
    accept_connections();
    io_context_.run();
  });
}

void TCPConnection::stop_server() {
  INFO("正在停止服务器...");
  running_ = false;  // 更新标志变量，表示服务器需要停止

  // 关闭接收器，防止新的连接进入
  if (acceptor_.is_open()) {
    acceptor_.close();
  }

  // 停止 io_context
  io_context_.stop();

  // 等待服务器线程结束
  if (server_thread_.joinable()) {
    server_thread_.join();
  }

  INFO("服务器已停止");
}

void TCPConnection::start_client() {
  is_server_ = false;
  tcp::endpoint endpoint(boost::asio::ip::make_address(host_), port_);
  socket_.connect(endpoint);
  INFO("作为客户端连接到：[{}:{}]", host_, port_);
}

void TCPConnection::connect(const std::string& cmd, std::function<Response(int, const json&)> handler) {
  handlers_[cmd] = handler;
}

json TCPConnection::send_message(const std::string& cmd, int code, const json& payload) {
  json message = {{"cmd", cmd}, {"code", code}, {"payload", payload}};
  std::string serialized = message.dump();
  boost::asio::write(socket_, boost::asio::buffer(serialized + "\n"));

  // Read response
  boost::asio::streambuf response_buffer;
  boost::asio::read_until(socket_, response_buffer, "\n");
  std::istream response_stream(&response_buffer);
  std::string response_data;
  std::getline(response_stream, response_data);

  return json::parse(response_data);
}

void TCPConnection::close() {
  if (socket_.is_open()) {
    socket_.close();
    INFO("连接关闭");
  }
  if (is_server_ && acceptor_.is_open()) {
    acceptor_.close();
    INFO("连接关闭");
  }
}

TCPConnection::~TCPConnection() { stop_server(); }

void TCPConnection::accept_connections() {
  if (!running_) return;  // 检查服务器是否处于运行状态
  acceptor_.async_accept([this](std::error_code ec, tcp::socket client_socket) {
    if (!ec && running_) {
      INFO("接收到连接请求：[{}:{}]", client_socket.remote_endpoint().address().to_string(),
           client_socket.remote_endpoint().port());

      std::thread(&TCPConnection::handle_client, this, std::move(client_socket)).detach();
    }

    if (running_) {
      accept_connections();  // 接受下一个连接
    }
  });
}

class TimedReader {
 public:
  TimedReader(boost::asio::io_context& io_context, boost::asio::ip::tcp::socket& socket)
      : io_context_(io_context), socket_(socket), timer_(io_context) {}

  std::size_t read_until_with_timeout(const std::string& delimiter, int timeout_seconds,
                                      boost::system::error_code& ec) {
    std::promise<std::size_t> promise;
    auto future = promise.get_future();

    timer_.expires_after(std::chrono::seconds(timeout_seconds));
    timer_.async_wait([this, &promise](const boost::system::error_code& timer_ec) {
      if (!timer_ec) {
        socket_.cancel();
      }
    });

    boost::asio::async_read_until(
        socket_, buffer_, delimiter,
        [this, &promise, &ec](const boost::system::error_code& read_ec, std::size_t bytes_transferred) {
          timer_.cancel();
          ec = read_ec;
          if (ec == boost::asio::error::operation_aborted) {
            ec = boost::asio::error::timed_out;  // 如果提前 cancel，异步操作错误码为 中断
          }
          promise.set_value(bytes_transferred);
        });

    std::size_t bytes_transferred = future.get();

    return bytes_transferred;
  }

  boost::asio::streambuf& buffer() { return buffer_; }

 private:
  boost::asio::io_context& io_context_;
  boost::asio::ip::tcp::socket& socket_;
  boost::asio::steady_timer timer_;
  boost::asio::streambuf buffer_;
};

void TCPConnection::handle_client(tcp::socket client_socket) {
  try {
    TimedReader reader(io_context_, client_socket);
    boost::system::error_code ec;

    while (client_socket.is_open() && running_) {
      auto bytes_transferred = reader.read_until_with_timeout("\n", 5, ec);

      if (!ec) {
        std::istream request_stream(&reader.buffer());
        std::string request_data;
        std::getline(request_stream, request_data);
        json request = json::parse(request_data);

        INFO("接收到请求：{}", request.dump());

        std::string cmd = request["cmd"];
        int code = request["code"];
        json payload = request["payload"];
        json response;

        if (handlers_.find(cmd) != handlers_.end()) {
          Response res = handlers_[cmd](code, payload);
          response = {{"cmd", cmd + "_res"}, {"code", res.code}, {"payload", res.payload}};
        } else {
          response = {{"cmd", cmd + "_res"}, {"code", 1}, {"payload", {{"error", "Unknown command"}}}};
        }

        // 发送响应
        std::string serialized = response.dump();
        if (client_socket.is_open()) {
          boost::asio::write(client_socket, boost::asio::buffer(serialized + "\n"));
        }
      } else if (ec == boost::asio::error::timed_out) {
        WARN("处理请求超时");
        break;
      } else if (ec == boost::asio::error::eof) {
        INFO("请求关闭，EOF");
        break;
      } else {
        WARN("读取请求时出现错误[{}]", ec.message());
        break;
      }
    }

    client_socket.close();
  } catch (const std::exception& e) {
    ERROR("处理客户端请求出现错误：{}", e.what());
  }
}
}  // namespace Hsu
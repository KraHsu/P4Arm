#include <Hsu/modbus_tcp.h>
// ---- third ----
#include <bits/stdint-uintn.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/spdlog.h>
// ---- standard ----
#include <chrono>
#include <cstddef>
#include <filesystem>
#include <functional>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

using boost::asio::ip::tcp;

#define DEBUG(...) Hsu::detail::mtcp_logger()->debug(__VA_ARGS__)
#define INFO(...) Hsu::detail::mtcp_logger()->info(__VA_ARGS__)
#define WARN(...) Hsu::detail::mtcp_logger()->warn(__VA_ARGS__)
#define ERROR(...) Hsu::detail::mtcp_logger()->error(__VA_ARGS__)

namespace Hsu::detail {
std::shared_ptr<spdlog::logger> mtcp_logger() {
  std::filesystem::create_directories("./log");
  static std::shared_ptr<spdlog::logger> LOGGER = spdlog::get("Hsu mTCP Logger");
  if (!LOGGER) {
    LOGGER = spdlog::basic_logger_mt("Hsu mTCP Logger", "./log/Hsu_mtcp.log");
    LOGGER->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [thread %t] [%^%l%$] %v");
    INFO("==== New ====");
  }
  return LOGGER;
}
}  // namespace Hsu::detail

namespace Hsu {
ModbusTCP::ModbusTCP(const std::string& host, uint16_t port)
    : host_(host), port_(port), io_context_(), socket_(io_context_), buffer_(256) {
  connect();
}
ModbusTCP::~ModbusTCP() { disconnect(); }

// 连接到 Modbus 服务器
bool ModbusTCP::connect() {
  try {
    tcp::resolver resolver(io_context_);
    auto endpoints = resolver.resolve(host_, std::to_string(port_));
    boost::asio::connect(socket_, endpoints);
    INFO("成功连接到 Modbus 服务器\n");
    return true;
  } catch (const std::exception& e) {
    ERROR("连接失败: {}\n", e.what());
    return false;
  }
}

// 从 Modbus 服务器断开连接
void ModbusTCP::disconnect() {
  if (socket_.is_open()) {
    socket_.close();
    ERROR("已断开与 Modbus 服务器的连接\n");
  }
}

std::vector<uint8_t> ModbusTCP::read_holding_registers_(uint16_t address, size_t size) {
  int len = (size + 1) / 2;
  if (len > 125) {
    throw std::invalid_argument("最多一次读取125个寄存器");
  }
  static std::vector<uint8_t> request = {
      0x00,   0x01,      // 事务 ID
      0x00,   0x00,      // 协议 ID
      0x00,   0x06,      // 长度
      0xFF,              // 单元 ID
      0x03,              // 功能码（读取保持寄存器）
      0 >> 8, 0 & 0xFF,  // 起始地址
      0 >> 8, 0 & 0xFF   // 寄存器数量
  };

  request[8] = address >> 8;
  request[9] = address & 0xFF;
  request[10] = len >> 8;
  request[11] = len & 0xFF;

  return sendRequest(request, std::chrono::milliseconds(1000));
}

std::vector<uint8_t> ModbusTCP::read_holding_registers(uint16_t address, size_t size) {
  std::vector<uint8_t> res;
  res.reserve(size);
  while (size > 0) {
    auto read_size = std::min<size_t>(size, 250U);
    auto response = read_holding_registers_(address, read_size);
    res.insert(res.end(), response.begin(), response.end());
    size -= read_size;
  }
  return res;
}

std::vector<uint8_t> ModbusTCP::write_holding_registers_(uint16_t address, const uint8_t* const data, size_t N) {
  if (N > 240) {
    throw std::invalid_argument("一次最多写入240个字节");
  }

  std::vector<uint8_t> request = {
      0x00,   0x01,      // 事务 ID
      0x00,   0x00,      // 协议 ID
      0x00,   0x00,      // 长度
      0xFF,              // 单元 ID
      0x10,              // 功能码（写保持寄存器）
      0 >> 8, 0 & 0xFF,  // 起始地址（0x0000）
      0 >> 8, 0 & 0xFF,  // 寄存器数量（1）
      0x00               // 字节数
  };

  int total_len = N * 2 + 7;
  request[4] = total_len >> 8;
  request[5] = total_len & 0xFF;

  request[8] = address >> 8;
  request[9] = address & 0xFF;

  int len = (N + 1) / 2;
  request[10] = len >> 8;
  request[11] = len & 0xFF;

  request[12] = N;

  request.reserve(request.size() + N);
  request.insert(request.end(), data, data + N);

  return sendRequest(request, std::chrono::milliseconds(1000));
}

void ModbusTCP::write_holding_registers(uint16_t address, std::vector<uint8_t> const& data) {
  int size = data.size();
  const auto* ptr = data.data();

  while (size) {
    auto write_size = std::min<size_t>(size, 240);
    auto res = write_holding_registers_(address, ptr, write_size);
    ptr += write_size;
    size -= write_size;
  }
}

// 发送 Modbus 请求并接收响应
std::vector<uint8_t> ModbusTCP::sendRequest(const std::vector<uint8_t>& request, std::chrono::milliseconds timeout) {
  try {
    boost::asio::steady_timer timer(io_context_);
    boost::system::error_code ec;
    bool timeout_occurred = false;

    // 设置超时处理
    timer.expires_after(timeout);
    timer.async_wait([&](const boost::system::error_code& error) {
      if (!error) {
        // 如果定时器到期且未被取消，说明超时发生
        timeout_occurred = true;
        socket_.cancel();  // 取消所有挂起的 I/O 操作
      }
    });

    // 发送请求
    boost::asio::write(socket_, boost::asio::buffer(request), ec);
    if (timeout_occurred) {
      throw std::runtime_error("发送请求超时");
    } else if (ec) {
      throw boost::system::system_error(ec);
    }

    // 读取响应头部（7 字节）
    std::size_t total_length = 0;
    boost::asio::read(socket_, boost::asio::buffer(buffer_, 7), ec);
    if (timeout_occurred) {
      throw std::runtime_error("读取响应头部超时");
    } else if (ec) {
      throw boost::system::system_error(ec);
    }

    total_length = ((buffer_[4] << 8) | buffer_[5]) - 1;  // 解析剩余数据长度

    // 确保缓冲区足够大
    if (buffer_.size() < 7 + total_length) {
      buffer_.resize(7 + total_length);  // 动态扩展（仅必要时）
    }

    // 读取剩余数据
    boost::asio::read(socket_, boost::asio::buffer(buffer_.data() + 7, total_length), ec);
    if (timeout_occurred) {
      throw std::runtime_error("读取响应数据超时");
    } else if (ec) {
      throw boost::system::system_error(ec);
    }

    // 返回响应数据
    return std::vector<uint8_t>(buffer_.begin() + 7, buffer_.begin() + 7 + total_length);

  } catch (const std::exception& e) {
    ERROR("通信错误: {}\n", e.what());
    return {};
  }
}

}  // namespace Hsu

namespace Hsu {
std::shared_ptr<ModbusActor_TCP> ModbusTCP::to_actor() {
  if (!ma_) {
    ma_ = std::shared_ptr<ModbusActor_TCP>(new ModbusActor_TCP(this));

    return ma_;
  }
  return ma_;
}

ModbusActor_TCP::ModbusActor_TCP(ModbusTCP* handle) : handle_{handle} {}

int ModbusActor_TCP::read_holding_registers(int address, int device) {
  throw std::runtime_error("未实现");
  return 0;
}

std::vector<int> ModbusActor_TCP::read_multiple_holding_registers(int address, int device, int len) {
  auto res = handle_->read_holding_registers(address, len * 2);

  std::vector<int> result;

  result.reserve(len);

  for (int i = 0; i < len; i++) {
    result[i] = res[2 * i] << 8 | res[2 * i + 1];
  }

  return result;
};

int ModbusActor_TCP::read_input_registers(int address, int device) {
  throw std::runtime_error("未实现");
  return 0;
}

void ModbusActor_TCP::write_single_register(int address, int device, int data) {
  std::vector<uint8_t> vec;
  vec.reserve(2);

  vec[0] = data >> 8;
  vec[1] = data & 0xFF;

  handle_->write_holding_registers(address, vec);
}

void ModbusActor_TCP::write_multiple_registers(int address, int device, std::vector<int> const& data) {
  std::vector<uint8_t> vec;
  vec.reserve(data.size() * 2);

  for (int value : data) {
    vec.push_back((value >> 8) & 0xFF);
    vec.push_back(value & 0xFF);
  }

  handle_->write_holding_registers(address, vec);
}
}  // namespace Hsu
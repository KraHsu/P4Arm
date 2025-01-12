#include <Hsu/hsu_module_log.h>
#include <Hsu/modbus_tcp.h>
// ---- third ----
#include <bits/stdint-uintn.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/spdlog.h>
// ---- standard ----
#include <chrono>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

using boost::asio::ip::tcp;

GENERATE_LOGGER(modbus_tcp)

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
    WARN("已断开与 Modbus 服务器的连接\n");
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
    res.insert(res.end(), response.begin() + 2, response.end());
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
std::weak_ptr<ModbusActor_TCP> ModbusTCP::produce_modbus_actor() {
  // if (modbus_actor_.get()) {
  //   ERROR("每个TCP只能生产一个Modbus接口");
  // }
  modbus_actor_ = std::make_shared<ModbusActor_TCP>(ModbusActor_TCP::Passkey{}, shared_from_this());

  return modbus_actor_;
}

ModbusActor_TCP::ModbusActor_TCP(ModbusActor_TCP::Passkey, std::weak_ptr<ModbusTCP> tcp) : tcp_{tcp} {}

int ModbusActor_TCP::read_holding_registers(int const& address) {
  throw std::runtime_error("未实现");
  return 0;
}

std::vector<int> ModbusActor_TCP::read_multiple_holding_registers(int const& address, int const& len) {
  if (auto tcp = tcp_.lock()) {
    auto res = tcp->read_holding_registers(address, len * 2);
    std::vector<int> result;
    result.reserve(len);

    for (int i = 0; i < len; i++) {
      result.push_back(res[2 * i] << 8 | res[2 * i + 1]);
    }

    return result;
  } else {
    ERROR("tcp已销毁，接口失效");
  }

  return {-1024};
};

int ModbusActor_TCP::read_input_registers(int const& address) {
  throw std::runtime_error("未实现");
  return 0;
}

void ModbusActor_TCP::write_single_register(int const& address, int const& data) {
  if (auto tcp = tcp_.lock()) {
    std::vector<uint8_t> vec;
    vec.reserve(2);

    vec[0] = data >> 8;
    vec[1] = data & 0xFF;

    tcp->write_holding_registers(address, vec);

  } else {
    ERROR("tcp已销毁，接口失效");
  }
}

void ModbusActor_TCP::write_multiple_registers(int const& address, std::vector<int> const& data) {
  if (auto tcp = tcp_.lock()) {
    std::vector<uint8_t> vec;
    vec.reserve(data.size() * 2);

    for (int value : data) {
      vec.push_back((value >> 8) & 0xFF);
      vec.push_back(value & 0xFF);
    }

    tcp->write_holding_registers(address, vec);
  } else {
    ERROR("tcp已销毁，接口失效");
  }
}
}  // namespace Hsu
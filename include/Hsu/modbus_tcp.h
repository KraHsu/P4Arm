#pragma once

#include "Hsu/arm.h"
#include <bits/stdint-uintn.h>
#include <boost/asio.hpp>
#include <cstddef>
#include <iostream>
#include <memory>
#include <vector>

namespace Hsu {
class ModbusActor_TCP;

class ModbusTCP {
 private:
  std::shared_ptr<ModbusActor_TCP> ma_{nullptr};

  std::vector<uint8_t> read_holding_registers_(uint16_t address, size_t len);

  std::vector<uint8_t> write_holding_registers_(uint16_t address, const uint8_t* const data, size_t N);

 public:
  ModbusTCP(const std::string& host, uint16_t port);

  ~ModbusTCP();

  // 连接到 Modbus 服务器
  bool connect();

  // 从 Modbus 服务器断开连接
  void disconnect();

  // 发送 Modbus 请求并接收响应
  std::vector<uint8_t> sendRequest(const std::vector<uint8_t>& request, std::chrono::milliseconds timeout);

  std::vector<uint8_t> read_holding_registers(uint16_t address, size_t len);

  void write_holding_registers(uint16_t address, std::vector<uint8_t> const& data);

  std::shared_ptr<ModbusActor_TCP> to_actor();

 private:
  std::string host_;
  uint16_t port_;
  boost::asio::io_context io_context_;
  boost::asio::ip::tcp::socket socket_;

  // 使用类成员缓冲区，避免重复分配内存
  std::vector<uint8_t> buffer_;
};

class ModbusActor_TCP : public ModbusActorBase {
 private:
  ModbusTCP* handle_{nullptr};

  ModbusActor_TCP(ModbusTCP* handle);

  friend class ModbusTCP;

 public:
  ModbusActor_TCP(ModbusActor_TCP const&) = delete;

  int read_holding_registers(int address, int device) override;

  std::vector<int> read_multiple_holding_registers(int address, int device, int len) override;

  int read_input_registers(int address, int device) override;

  void write_single_register(int address, int device, int data) override;

  void write_multiple_registers(int address, int device, std::vector<int> const& data) override;
};
}  // namespace Hsu
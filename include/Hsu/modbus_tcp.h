#pragma once

#include <Hsu/arm.h>
#include <bits/stdint-uintn.h>
#include <boost/asio.hpp>
#include <cstddef>
#include <memory>
#include <vector>

namespace Hsu {
class ModbusActor_TCP;

class ModbusTCP : public std::enable_shared_from_this<ModbusTCP> {
 private:
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

 private:
  std::string host_;
  uint16_t port_;
  boost::asio::io_context io_context_;
  boost::asio::ip::tcp::socket socket_;

  // 使用类成员缓冲区，避免重复分配内存
  std::vector<uint8_t> buffer_;

 private:
  std::shared_ptr<ModbusActor_TCP> modbus_actor_;

 public:
  std::weak_ptr<ModbusActor_TCP> produce_modbus_actor();
};

class ModbusActor_TCP : public ModbusActorBase {
 private:
  struct Passkey {
    explicit Passkey() {}
  };

  std::weak_ptr<ModbusTCP> tcp_;

  friend class ModbusTCP;

 public:
  ModbusActor_TCP operator=(ModbusActor_TCP const&) = delete;
  ModbusActor_TCP(ModbusActor_TCP const&) = delete;

  ModbusActor_TCP(Passkey, std::weak_ptr<ModbusTCP> tcp);

  int read_holding_registers(int const& address) override;

  std::vector<int> read_multiple_holding_registers(int const& address, int const& len) override;

  int read_input_registers(int const& address) override;

  void write_single_register(int const& address, int const& data) override;

  void write_multiple_registers(int const& address, std::vector<int> const& data) override;
};
}  // namespace Hsu
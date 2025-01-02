#pragma once

// ---- Hsu ----
#include <Hsu/grip.h>
// ---- RMArm ----
#include <RMArm/rm_define.h>
#include <RMArm/rm_service.h>
// ---- third ----
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/spdlog.h>
#include <units.h>
// ---- standard ----
#include <mutex>
#include <variant>

namespace Hsu {

namespace detail {
void api_log(const char* message, va_list args);

class init_helper {
 private:
  init_helper();

  ~init_helper();

  RM_Service service{};

  friend RM_Service& service_ins();
};

RM_Service& service_ins();

std::shared_ptr<spdlog::logger> arm_logger();

void throw_modbus_err(std::string msg, int res);
}  // namespace detail

class Arm;

class ModbusActorBase {
 public:
  virtual int read_holding_registers(int address, int device) = 0;

  virtual std::vector<int> read_multiple_holding_registers(int address, int device, int len) = 0;

  virtual int read_input_registers(int address, int device) = 0;

  virtual void write_single_register(int address, int device, int data) = 0;

  virtual void write_multiple_registers(int address, int device, std::vector<int> const& data) = 0;
};

class ModbusActor_RTU : public ModbusActorBase {
 private:
  rm_robot_handle* handle_{nullptr};

  int port_;

  ModbusActor_RTU(rm_robot_handle* handle, int port);

  friend class Arm;

 public:
  ModbusActor_RTU(ModbusActor_RTU const&) = delete;

  int read_holding_registers(int address, int device) override;

  std::vector<int> read_multiple_holding_registers(int address, int device, int len) override;

  int read_input_registers(int address, int device) override;

  void write_single_register(int address, int device, int data) override;

  void write_multiple_registers(int address, int device, std::vector<int> const& data) override;
};

class Arm {
 private:
  rm_robot_handle* handle_{nullptr};
  rm_position_t offset_{0, 0, 0};
  std::mutex mutex_;  // 添加互斥锁
  bool paused_{false};
  bool stop_{false};
  double hand_len_{0};

 public:
  Arm(std::string const& ip, int const& port);

  ~Arm();

  int move_common(const std::string& command, rm_position_t const& position,
                  std::variant<rm_quat_t, rm_euler_t> const& posture, int v, int r, int trajectory_connect, int block);

  int move_jp(rm_position_t const& position, std::variant<rm_quat_t, rm_euler_t> const& posture, int v, int r = 0,
              int trajectory_connect = 0, int block = 0);

  int move_jp_force(rm_position_t const& position, std::variant<rm_quat_t, rm_euler_t> const& posture, int v, int r = 0,
                    int trajectory_connect = 0, int block = 0);

  int move_l(rm_position_t const& position, std::variant<rm_quat_t, rm_euler_t> const& posture, int v, int r = 0,
             int trajectory_connect = 0, int block = 0);

  int move_s(rm_position_t const& position, std::variant<rm_quat_t, rm_euler_t> const& posture, int v, int r = 0,
             int trajectory_connect = 0, int block = 0);

  int follow(rm_position_t const& position, std::variant<rm_quat_t, rm_euler_t> const& posture);

  int set_mode(int mode);

  void set_hand_len(double len);

  int get_id();

  rm_current_arm_state_t get_state();

  void set_offset(rm_position_t offset);

  int pause();

  int continu();

  int restart();

  int stop();

  std::shared_ptr<ModbusActor_RTU> connect_modbus_actor(int port, int baudrate, int timeout);

  //  public:
  //   void set_grip_position(uint16_t pose);

  //   void set_grip_force(uint16_t force);

  //   void set_grip_speed(uint16_t speed);
};
}  // namespace Hsu

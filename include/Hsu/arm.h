#pragma once

// ---- Hsu ----
#include <Hsu/hand.h>
// ---- RMArm ----
#include <RMArm/rm_define.h>
#include <RMArm/rm_service.h>
// ---- third ----
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/spdlog.h>
#include <units.h>
// ---- standard ----
#include <memory>
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

class ModbusActor_RTU : public ModbusActorBase {
  friend class Arm;

 private:
  std::weak_ptr<Arm> arm_;

  rm_peripheral_read_write_params_t params_;

 private:
  ModbusActor_RTU(std::weak_ptr<Arm> arm, int port, int device) noexcept;

 public:
  ModbusActor_RTU(ModbusActor_RTU const&) = delete;

  int read_holding_registers(int const& address) override;

  std::vector<int> read_multiple_holding_registers(int const& address, int const& len) override;

  int read_input_registers(int const& address) override;

  void write_single_register(int const& address, int const& data) override;

  void write_multiple_registers(int const& address, std::vector<int> const& data) override;
};

class Arm : public std::enable_shared_from_this<Arm> {
 private:
  rm_robot_handle* handle_{nullptr};
  rm_position_t offset_{0, 0, 0};
  std::mutex mutex_;
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

 private:
  std::shared_ptr<ModbusActor_RTU> modbus_actor_;

 public:
  int read_holding_registers(rm_peripheral_read_write_params_t const& params);

  std::vector<int> read_multiple_holding_registers(rm_peripheral_read_write_params_t params, int const& len);

  int read_input_registers(rm_peripheral_read_write_params_t const& params);

  void write_single_register(rm_peripheral_read_write_params_t const& params, int const& data);

  void write_multiple_registers(rm_peripheral_read_write_params_t params, std::vector<int> const& data);

  std::weak_ptr<ModbusActor_RTU> produce_modbus_actor(int port, int baudrate, int device, int timeout);
};
}  // namespace Hsu

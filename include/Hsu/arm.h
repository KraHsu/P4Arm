#pragma once

// ---- Hsu ----
#include <Hsu/frame.h>
#include <Hsu/hand.h>
// ---- RMArm ----
#include <RMArm/rm_define.h>
#include <RMArm/rm_service.h>
// ---- third ----
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/spdlog.h>
#include <units.h>
// ---- standard ----
#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
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
 public:
  struct Frames {
    std::shared_ptr<Frame> links[8];
    std::array<Types::HomogeneousM, 8> data;

    Frames();

    std::array<Types::HomogeneousM, 8> get_data();

    Frames& set_base_offset(Types::HomogeneousM const& offset);

    Frames& set_joint_angle(uint32_t i, units::angle::radian_t rad);
  };

  Frames const& read_frames() { return frame_; }

 private:
  Frames frame_;

  std::shared_ptr<Frame> base_;

  std::shared_ptr<Frame> actor_;

  // ----

 public:
  Arm(std::string const& ip, int const& port);

  ~Arm();

  int move_jp(rm_pose_t pose, int v) noexcept;

  [[deprecated("暂时弃用")]] int move_common(const std::string& command, rm_position_t const& position,
                                             std::variant<rm_quat_t, rm_euler_t> const& posture, int v, int r,
                                             int trajectory_connect, int block);

  [[deprecated("暂时弃用")]] int move_jp(rm_position_t const& position,
                                         std::variant<rm_quat_t, rm_euler_t> const& posture, int v, int r = 0,
                                         int trajectory_connect = 0, int block = 0);

  [[deprecated("暂时弃用")]] int move_jp_force(rm_position_t const& position,
                                               std::variant<rm_quat_t, rm_euler_t> const& posture, int v, int r = 0,
                                               int trajectory_connect = 0, int block = 0);

  [[deprecated("暂时弃用")]] int move_l(rm_position_t const& position,
                                        std::variant<rm_quat_t, rm_euler_t> const& posture, int v, int r = 0,
                                        int trajectory_connect = 0, int block = 0);

  [[deprecated("暂时弃用")]] int move_s(rm_position_t const& position,
                                        std::variant<rm_quat_t, rm_euler_t> const& posture, int v, int r = 0,
                                        int trajectory_connect = 0, int block = 0);

  [[deprecated("暂时弃用")]] int follow(rm_position_t const& position,
                                        std::variant<rm_quat_t, rm_euler_t> const& posture);

  int set_mode(int mode);

  int get_id();

  rm_current_arm_state_t get_state();

  void set_offset(rm_position_t offset);

  int pause();

  int continu();

  int restart();

  int stop();

 public:
  struct Posture {
    std::array<units::length::meter_t, 3> Position;
    std::array<units::angle::radian_t, 3> Euler;

    Posture(std::array<units::length::meter_t, 3> position, std::array<units::angle::radian_t, 3> euler);

    Posture(units::length::meter_t x, units::length::meter_t y, units::length::meter_t z, units::angle::radian_t rx,
            units::angle::radian_t ry, units::angle::radian_t rz);

    Types::HomogeneousM to_homogeneous();
  };

  Posture read_posture(std::string const& actor, std::string const& frame);

  Posture read_target(std::string const& actor, std::string const& frame);

  std::array<units::angle::radian_t, 7> read_joint_angle();

  std::array<Types::HomogeneousM, 8> get_frames_data();

  void set_base_offset(Types::HomogeneousM const& offset);

  void set_actor_offset(Types::HomogeneousM const& offset);

  void move(Posture posture, int v, std::string const& actor, std::string const& frame);

 private:
  rm_robot_handle* handle_{nullptr};
  std::mutex mutex_;
  bool paused_{false};
  bool stop_{false};

  Posture target_;

  // ----

 public:
  int read_holding_registers(rm_peripheral_read_write_params_t const& params);

  std::vector<int> read_multiple_holding_registers(rm_peripheral_read_write_params_t params, int const& len);

  int read_input_registers(rm_peripheral_read_write_params_t const& params);

  void write_single_register(rm_peripheral_read_write_params_t const& params, int const& data);

  void write_multiple_registers(rm_peripheral_read_write_params_t params, std::vector<int> const& data);

  std::weak_ptr<ModbusActor_RTU> produce_modbus_actor(int port, int baudrate, int device, int timeout);

 private:
  std::shared_ptr<ModbusActor_RTU> modbus_actor_;
};
}  // namespace Hsu

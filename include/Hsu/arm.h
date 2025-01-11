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

    Frames() {
      const double D = std::sqrt(2) / 2;
      const double Angle90 = 3.14159265358979323846 / 2;
      const auto World = Hsu::Frame::WORLD_FRAME();

      Hsu::Types::TranslationM T;
      Hsu::Types::RotationM RxN90;
      Hsu::Types::RotationM Rx90;

      RxN90.value.row(0) << 1, 0, 0;
      RxN90.value.row(1) << 0, 0, 1;
      RxN90.value.row(2) << 0, -1, 0;

      Rx90.value.row(0) << 1, 0, 0;
      Rx90.value.row(1) << 0, 0, -1;
      Rx90.value.row(2) << 0, 1, 0;

      links[0] = World->define_frame("BaseLink", {});

      T.value << 0, 0, 0.2405;
      links[1] = links[0]->define_frame("Link 1", {RxN90, T});

      T.value << 0, 0, 0;
      links[2] = links[1]->define_frame("Link 2", {Rx90, T});

      T.value << 0, 0, 0.256;
      links[3] = links[2]->define_frame("Link 3", {RxN90, T});

      T.value << 0, 0, 0;
      links[4] = links[3]->define_frame("Link 4", {Rx90, T});

      T.value << 0, 0, 0.21;
      links[5] = links[4]->define_frame("Link 5", {RxN90, T});

      T.value << 0, 0, 0;
      links[6] = links[5]->define_frame("Link 6", {Rx90, T});

      T.value << 0, 0, 0.144;
      links[7] = links[6]->define_frame("Link 7", {T});
    }

    std::array<Types::HomogeneousM, 8> get_data() {
      auto& World = Frame::WORLD_FRAME();
      for (int i = 0; i < 8; i++) {
        data[i] = links[i]->get_homegeneous_relative_to(World);
      }
      return data;
    }

    Frames& set_base_offset(Types::HomogeneousM offset) {
      links[0]->set_homegeneous(offset);
      return *this;
    }

    Frames& set_joint_angle(uint32_t i, units::angle::radian_t rad) {
      if (i < 1 or i > 7) {
        throw std::runtime_error("关节应为 Joint 1-7");
      }

      double c = units::math::cos(rad);
      double s = units::math::sin(rad);

      Hsu::Types::RotationM R;

      switch (i) {
        case 1:
        case 3:
        case 5:
          // Rz @ Rx(-90)
          R.value.row(0) << +c, +0, -s;
          R.value.row(1) << +s, +0, +c;
          R.value.row(2) << +0, -1, +0;
          break;
        case 2:
        case 4:
        case 6:
          // Rz @ Rx(90)
          R.value.row(0) << +c, +0, +s;
          R.value.row(1) << +s, +0, -c;
          R.value.row(2) << +0, +1, +0;
          break;
        case 7:
          // Rz
          R.value.row(0) << +c, -s, +0;
          R.value.row(1) << +s, +c, +0;
          R.value.row(2) << +0, +0, +1;
          break;
      }

      links[i]->set_rotation(R);
      return *this;
    }
  };

  Frames const& read_frames() { return frame_; }

 private:
  Frames frame_{};

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

  std::array<units::angle::radian_t, 7> read_joint_angle();

  std::array<Types::HomogeneousM, 8> get_frames_data();

  void set_base_offset(Types::HomogeneousM offset);

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

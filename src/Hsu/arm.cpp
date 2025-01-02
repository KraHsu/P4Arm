// ---- Hsu ----
#include "RMArm/rm_define.h"
#include <Hsu/arm.h>
#include <Hsu/matrix_ops.h>
// ---- stdandard ----
#include <chrono>
#include <exception>
#include <memory>
#include <stdexcept>
#include <thread>
#include <sys/socket.h>
// ---- third ----
#include <bits/stdint-intn.h>
#include <fmt/format.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/spdlog.h>

#define DEBUG(...) Hsu::detail::arm_logger()->debug(__VA_ARGS__)
#define INFO(...) Hsu::detail::arm_logger()->info(__VA_ARGS__)
#define WARN(...) Hsu::detail::arm_logger()->warn(__VA_ARGS__)
#define ERROR(...) Hsu::detail::arm_logger()->error(__VA_ARGS__)

namespace Hsu {
ModbusActor_RTU::ModbusActor_RTU(rm_robot_handle* handle, int port) : handle_(handle), port_(port) {}

int ModbusActor_RTU::read_holding_registers(int address, int device) {
  rm_peripheral_read_write_params_t params;
  params.port = port_;
  params.address = address;
  params.device = device;

  int data;

  auto res = detail::service_ins().rm_read_holding_registers(handle_, params, &data);

  detail::throw_modbus_err(fmt::format("{}号机械臂读保持寄存器失败：", handle_->id), res);

  return data;
}

std::vector<int> ModbusActor_RTU::read_multiple_holding_registers(int address, int device, int len) {
  if (len <= 1) {
    throw std::invalid_argument("读多寄存器时，长度必须大于1");
  }

  static const int max_registers_per_read = 12;

  std::vector<int> res;
  res.reserve(len);

  rm_peripheral_read_write_params_t params;
  params.device = device;
  params.address = address;
  params.port = port_;

  for (int remaining = len; remaining > 0;) {
    params.num = std::min(remaining, max_registers_per_read);

    std::vector<int> temp(params.num * 2);

    auto err = detail::service_ins().rm_read_multiple_holding_registers(handle_, params, temp.data());
    detail::throw_modbus_err(fmt::format("{}号机械臂读多保持寄存器失败：", handle_->id), err);

    for (int i = 0; i < params.num; ++i) {
      int high = temp[2 * i];
      int low = temp[2 * i + 1];
      res.push_back((high << 8) | low);
    }

    remaining -= params.num;
    params.address += params.num * 2;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  return res;
}

void ModbusActor_RTU::write_multiple_registers(int address, int device, const std::vector<int>& data) {
  if (data.size() <= 1) {
    throw std::invalid_argument("写多寄存器时，至少写两个寄存器");
  }

  static const int max_registers_per_write = 10;
  const int len = data.size();

  std::vector<int> data_bytes;
  data_bytes.reserve(len * 2);

  for (int value : data) {
    data_bytes.push_back((value >> 8) & 0xFF);
    data_bytes.push_back(value & 0xFF);
  }

  rm_peripheral_read_write_params_t params;
  params.device = device;
  params.address = address;
  params.port = port_;

  for (int i = 0; i < len; i += max_registers_per_write) {
    params.num = std::min(max_registers_per_write, len - i);

    auto err = detail::service_ins().rm_write_registers(handle_, params, &data_bytes[i * 2]);

    detail::throw_modbus_err(fmt::format("{}号机械臂写多寄存器失败：", handle_->id), err);

    params.address += params.num * 2;

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

int ModbusActor_RTU::read_input_registers(int address, int device) {
  rm_peripheral_read_write_params_t params;
  params.port = port_;
  params.address = address;
  params.device = device;

  int data;

  auto res = detail::service_ins().rm_read_input_registers(handle_, params, &data);

  detail::throw_modbus_err(fmt::format("{}号机械臂读输入寄存器失败：", handle_->id), res);

  return data;
}

void ModbusActor_RTU::write_single_register(int address, int device, int data) {
  rm_peripheral_read_write_params_t params;
  params.port = port_;
  params.address = address;
  params.device = device;

  auto res = detail::service_ins().rm_write_single_register(handle_, params, data);

  detail::throw_modbus_err(fmt::format("{}号机械臂写单个寄存器失败：", handle_->id), res);
}

}  // namespace Hsu

namespace Hsu {
Arm::Arm(std::string const& ip, int const& port) {
  auto& service = detail::service_ins();
  service.rm_set_log_call_back(detail::api_log, 3);
  handle_ = service.rm_create_robot_arm(ip.c_str(), port);
  if (!handle_ or handle_->id <= 0) {
    throw std::runtime_error("连接机械臂失败");
  } else {
    INFO("[{}]号机械臂连接成功", handle_->id);
  }

  // if (!with_grip) return;

  // service.rm_set_modbus_mode(handle_, 1, 115200, 2);
  // rm_peripheral_read_write_params_t params_coils;
  // // 初始化夹爪，最大值、最小值运动复位
  // int data = 0xA5;
  // params_coils.port = 1;
  // params_coils.address = 0x0100;
  // params_coils.device = 1;
  // int ret = service.rm_write_single_register(handle_, params_coils, data);
  // DEBUG("夹爪初始化指令: {}", ret);
  // sleep(8);

  // params_coils.port = 1;
  // params_coils.address = 0x0200;
  // params_coils.device = 1;
  // ret = service.rm_read_holding_registers(handle_, params_coils, &data);
  // DEBUG("夹爪初始化指令: {}", ret);
  // if (data == 1) {
  //   set_grip_force(20);
  //   sleep(1);
  //   set_grip_speed(20);
  //   sleep(1);
  // } else {
  //   throw std::runtime_error("夹爪初始化失败");
  // }
}

Arm::~Arm() {
  INFO("[{}]号机械臂连接关闭", handle_->id);
  detail::service_ins().rm_delete_robot_arm(handle_);
  return;
}

int Arm::move_common(const std::string& command, rm_position_t const& position,
                     std::variant<rm_quat_t, rm_euler_t> const& posture, int v, int r, int trajectory_connect,
                     int block) {
  if (stop_ or paused_) {
    return -1;
  }

  std::lock_guard<std::mutex> lock(mutex_);  // 加锁
  rm_pose_t pose = {0};

  pose.position.x = position.x;
  pose.position.y = position.y;
  pose.position.z = position.z;

  if (std::holds_alternative<rm_quat_t>(posture)) {
    pose.quaternion = std::get<rm_quat_t>(posture);
  } else {
    pose.euler = std::get<rm_euler_t>(posture);
  }

  int res = 0;

  if (command == "movej") {
    res = detail::service_ins().rm_movej_p(handle_, pose, v, r, trajectory_connect, block);
  } else if (command == "movel") {
    res = detail::service_ins().rm_movel(handle_, pose, v, r, trajectory_connect, block);
  } else if (command == "moves") {
    res = detail::service_ins().rm_moves(handle_, pose, v, r, trajectory_connect, block);
  }

  return res;
}

int Arm::move_jp(rm_position_t const& position, std::variant<rm_quat_t, rm_euler_t> const& posture, int v, int r,
                 int trajectory_connect, int block) {
  rm_position_t real_position;
  real_position.x = position.x + offset_.x;
  real_position.y = position.y + offset_.y;
  real_position.z = position.z + offset_.z;

  Eigen::Vector3d end_pos(real_position.x, real_position.y, real_position.z);
  Eigen::Vector3d euler_angles;
  if (std::holds_alternative<rm_quat_t>(posture)) {
    ERROR("暂时不支持四元数");
  } else {
    euler_angles = Eigen::Vector3d(std::get<rm_euler_t>(posture).rx, std::get<rm_euler_t>(posture).ry,
                                   std::get<rm_euler_t>(posture).rz);
  }
  Eigen::Vector3d real_pos = Hsu::compute_previous_joint_position(end_pos, euler_angles, hand_len_);
  real_position.x = real_pos[0];
  real_position.y = real_pos[1];
  real_position.z = real_pos[2];

  INFO("手腕的位置：[{}, {}, {}]", real_position.x, real_position.y, real_position.z);

  return move_common("movej", real_position, posture, v, r, trajectory_connect, block);
}

int Arm::move_jp_force(rm_position_t const& position, std::variant<rm_quat_t, rm_euler_t> const& posture, int v, int r,
                       int trajectory_connect, int block) {
  std::lock_guard<std::mutex> lock(mutex_);  // 加锁
  rm_pose_t pose;

  pose.position.x = position.x + offset_.x;
  pose.position.y = position.y + offset_.y;
  pose.position.z = position.z + offset_.z;

  if (std::holds_alternative<rm_quat_t>(posture)) {
    pose.quaternion = std::get<rm_quat_t>(posture);
  } else {
    pose.euler = std::get<rm_euler_t>(posture);
  }

  return detail::service_ins().rm_movej_p(handle_, pose, v, r, trajectory_connect, block);
}

int Arm::move_l(rm_position_t const& position, std::variant<rm_quat_t, rm_euler_t> const& posture, int v, int r,
                int trajectory_connect, int block) {
  return move_common("movel", position, posture, v, r, trajectory_connect, block);
}

int Arm::move_s(rm_position_t const& position, std::variant<rm_quat_t, rm_euler_t> const& posture, int v, int r,
                int trajectory_connect, int block) {
  return move_common("moves", position, posture, v, r, trajectory_connect, block);
}

int Arm::follow(rm_position_t const& position, std::variant<rm_quat_t, rm_euler_t> const& posture) {
  std::lock_guard<std::mutex> lock(mutex_);
  rm_pose_t pose;

  pose.position = position;
  if (std::holds_alternative<rm_quat_t>(posture)) {
    pose.quaternion = std::get<rm_quat_t>(posture);
  } else {
    pose.euler = std::get<rm_euler_t>(posture);
  }

  return detail::service_ins().rm_movep_follow(handle_, pose);
}

int Arm::set_mode(int mode) {
  std::lock_guard<std::mutex> lock(mutex_);
  return detail::service_ins().rm_set_arm_run_mode(handle_, mode);
}

int Arm::get_id() { return handle_->id; }

rm_current_arm_state_t Arm::get_state() {
  std::lock_guard<std::mutex> lock(mutex_);
  rm_current_arm_state_t state;

  auto res = detail::service_ins().rm_get_current_arm_state(handle_, &state);
  detail::throw_modbus_err("查询状态：", res);

  state.pose.position.x -= offset_.x;
  state.pose.position.y -= offset_.y;
  state.pose.position.z -= offset_.z;

  Eigen::Vector3d end_pos(state.pose.position.x, state.pose.position.y, state.pose.position.z);
  Eigen::Vector3d euler_angles(state.pose.euler.rx, state.pose.euler.ry, state.pose.euler.rz);

  Eigen::Vector3d real_pos = Hsu::compute_next_joint_position(end_pos, euler_angles, hand_len_);

  state.pose.position.x = real_pos[0];
  state.pose.position.y = real_pos[1];
  state.pose.position.z = real_pos[2];

  return state;
}

void Arm::set_offset(rm_position_t offset) {
  std::lock_guard<std::mutex> lock(mutex_);
  this->offset_ = offset;
}

std::shared_ptr<ModbusActor_RTU> Arm::connect_modbus_actor(int port, int baudrate, int timeout) {
  static bool _ = true;
  if (_) {
    _ = false;

    std::lock_guard<std::mutex> lock(mutex_);
    auto res = detail::service_ins().rm_set_modbus_mode(handle_, port, baudrate, timeout);

    detail::throw_modbus_err(fmt::format("{}号机械臂Modbus连接失败：", handle_->id), res);

    return std::shared_ptr<ModbusActor_RTU>(new ModbusActor_RTU(handle_, port));
  }
  throw std::runtime_error("只能生产一次");
}

// void Arm::set_grip_force(uint16_t force) {
//   std::lock_guard<std::mutex> lock(mutex_);
//   rm_peripheral_read_write_params_t params_coils;
//   int data = force;
//   params_coils.port = 1;
//   params_coils.address = 0x0101;
//   params_coils.device = 1;
//   int ret = detail::service_ins().rm_write_single_register(handle_, params_coils, data);
// }

// void Arm::set_grip_speed(uint16_t speed) {
//   std::lock_guard<std::mutex> lock(mutex_);
//   rm_peripheral_read_write_params_t params_coils;
//   int data = speed;
//   params_coils.port = 1;
//   params_coils.address = 0x0104;
//   params_coils.device = 1;
//   int ret = detail::service_ins().rm_write_single_register(handle_, params_coils, data);
// }

// void Arm::set_grip_position(uint16_t pose) {
//   std::lock_guard<std::mutex> lock(mutex_);
//   rm_peripheral_read_write_params_t params_coils;
//   int data = pose;
//   params_coils.port = 1;
//   params_coils.address = 0x0103;
//   params_coils.device = 1;
//   int ret = detail::service_ins().rm_write_single_register(handle_, params_coils, data);
// }

int Arm::pause() {
  std::lock_guard<std::mutex> lock(mutex_);
  this->paused_ = true;
  return detail::service_ins().rm_set_arm_pause(handle_);
}

int Arm::continu() {
  std::lock_guard<std::mutex> lock(mutex_);
  this->paused_ = false;
  return detail::service_ins().rm_set_arm_continue(handle_);
}

int Arm::restart() {
  std::lock_guard<std::mutex> lock(mutex_);
  this->stop_ = false;
  this->paused_ = false;
  detail::service_ins().rm_set_arm_delete_trajectory(handle_);
  return detail::service_ins().rm_set_arm_continue(handle_);
}

int Arm::stop() {
  std::lock_guard<std::mutex> lock(mutex_);
  this->stop_ = true;
  return detail::service_ins().rm_set_arm_pause(handle_);
}

void Arm::set_hand_len(double len) {
  std::lock_guard<std::mutex> lock(mutex_);
  this->hand_len_ = len;
}

namespace detail {

std::shared_ptr<spdlog::logger> arm_logger() {
  std::filesystem::create_directories("./log");
  static std::shared_ptr<spdlog::logger> LOGGER = spdlog::get("Hsu Arm Logger");
  if (!LOGGER) {
    LOGGER = spdlog::basic_logger_mt("Hsu Arm Logger", "./log/Hsu_arm.log");
    LOGGER->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [thread %t] [%^%l%$] %v");
    INFO("==== New ====");
  }
  return LOGGER;
}

void api_log(const char* message, va_list args) {
  if (!message) {
    ERROR("机械臂日志获取失败：空指针");
    return;
  }

  char buffer[1024];
  vsnprintf(buffer, sizeof(buffer), message, args);
  ERROR(buffer);
}

init_helper::init_helper() {
  auto code = this->service.rm_init(RM_TRIPLE_MODE_E);
  if (code) {
    throw std::runtime_error("init fail");
  }
  INFO("机械臂核心初始化成功");
}

init_helper::~init_helper() {
  this->service.rm_destory();
  INFO("机械臂核心已销毁");
}

RM_Service& service_ins() {
  static init_helper init{};
  return init.service;
}

void throw_modbus_err(std::string msg, int res) {
  if (!res) return;
  switch (res) {
    case 1:
      msg += "控制器返回false，传递参数错误或机械臂状态发生错误。";
      break;
    case -1:
      msg += "数据发送失败，通信过程中出现问题。";
      break;
    case -2:
      msg += "数据接收失败，通信过程中出现问题或者控制器超时没有返回。";
      break;
    case -3:
      msg += "返回值解析失败，接收到的数据格式不正确或不完整。";
      break;
  }
  ERROR(msg);
  throw std::runtime_error(msg);
}

}  // namespace detail

}  // namespace Hsu
// ---- Hsu ----
#include "Hsu/frame.h"
#include "units.h"
#include <Hsu/arm.h>
#include <Hsu/hsu_module_log.h>
#include <Hsu/matrix_ops.h>
#include <RMArm/rm_define.h>
// ---- stdandard ----
#include <memory>
#include <stdexcept>
#include <string>
#include <sys/socket.h>
// ---- third ----
#include <bits/stdint-intn.h>
#include <fmt/format.h>

GENERATE_LOGGER(arm)

using namespace units::literals;

namespace Hsu {
Arm::Frames::Frames() {
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

std::array<Types::HomogeneousM, 8> Arm::Frames::get_data() {
  auto& World = Frame::WORLD_FRAME();
  for (int i = 0; i < 8; i++) {
    data[i] = links[i]->get_homegeneous_relative_to(World);
  }
  return data;
}

Arm::Frames& Arm::Frames::set_base_offset(Types::HomogeneousM const& offset) {
  links[0]->set_homegeneous(offset);
  return *this;
}

Arm::Frames& Arm::Frames::set_joint_angle(uint32_t i, units::angle::radian_t rad) {
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
}  // namespace Hsu

namespace Hsu {
Arm::Arm(std::string const& ip, int const& port) : frame_() {
  auto& service = detail::service_ins();
  service.rm_set_log_call_back(detail::api_log, 3);
  handle_ = service.rm_create_robot_arm(ip.c_str(), port);
  if (!handle_ or handle_->id <= 0) {
    throw std::runtime_error("连接机械臂失败");
  } else {
    INFO("[{}]号机械臂连接成功", handle_->id);
  }

  base_ = Hsu::Frame::WORLD_FRAME()->define_frame("Base", {});
  actor_ = frame_.links[7]->define_frame("Actor", {});
}

Arm::~Arm() {
  INFO("[{}]号机械臂连接关闭", handle_->id);
  detail::service_ins().rm_delete_robot_arm(handle_);
  return;
}

int Arm::move_jp(rm_pose_t pose, int v) noexcept { return detail::service_ins().rm_movej_p(handle_, pose, v, 0, 0, 0); }

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
  // real_position.x = position.x + offset_.x;
  // real_position.y = position.y + offset_.y;
  // real_position.z = position.z + offset_.z;

  Eigen::Vector3d end_pos(real_position.x, real_position.y, real_position.z);
  Eigen::Vector3d euler_angles;
  if (std::holds_alternative<rm_quat_t>(posture)) {
    ERROR("暂时不支持四元数");
  } else {
    euler_angles = Eigen::Vector3d(std::get<rm_euler_t>(posture).rx, std::get<rm_euler_t>(posture).ry,
                                   std::get<rm_euler_t>(posture).rz);
  }
  // Eigen::Vector3d real_pos = Hsu::compute_previous_joint_position(end_pos, euler_angles, hand_len_);
  // real_position.x = real_pos[0];
  // real_position.y = real_pos[1];
  // real_position.z = real_pos[2];

  INFO("手腕的位置：[{}, {}, {}]", real_position.x, real_position.y, real_position.z);

  return move_common("movej", real_position, posture, v, r, trajectory_connect, block);
}

int Arm::move_jp_force(rm_position_t const& position, std::variant<rm_quat_t, rm_euler_t> const& posture, int v, int r,
                       int trajectory_connect, int block) {
  std::lock_guard<std::mutex> lock(mutex_);  // 加锁
  rm_pose_t pose;

  // pose.position.x = position.x + offset_.x;
  // pose.position.y = position.y + offset_.y;
  // pose.position.z = position.z + offset_.z;

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

  // state.pose.position.x -= offset_.x;
  // state.pose.position.y -= offset_.y;
  // state.pose.position.z -= offset_.z;

  Eigen::Vector3d end_pos(state.pose.position.x, state.pose.position.y, state.pose.position.z);
  Eigen::Vector3d euler_angles(state.pose.euler.rx, state.pose.euler.ry, state.pose.euler.rz);

  // Eigen::Vector3d real_pos = Hsu::compute_next_joint_position(end_pos, euler_angles, hand_len_);

  // state.pose.position.x = real_pos[0];
  // state.pose.position.y = real_pos[1];
  // state.pose.position.z = real_pos[2];

  return state;
}

std::array<units::angle::radian_t, 7> Arm::read_joint_angle() {
  std::lock_guard<std::mutex> lock(mutex_);

  float tmp[7];
  std::array<units::angle::radian_t, 7> result;

  auto res = detail::service_ins().rm_get_joint_degree(handle_, tmp);
  detail::throw_modbus_err("查询关节角：", res);

  for (int i = 0; i < 7; i++) {
    result[i] = tmp[i] * units::angle::degree_t(1);
    frame_.set_joint_angle(i + 1, result[i]);
  }

  return result;
}

Arm::Posture::Posture(std::array<units::length::meter_t, 3> position, std::array<units::angle::radian_t, 3> euler)
    : Position(position), Euler(euler) {}

Arm::Posture::Posture(units::length::meter_t x, units::length::meter_t y, units::length::meter_t z,
                      units::angle::radian_t rx, units::angle::radian_t ry, units::angle::radian_t rz)
    : Position{x, y, z}, Euler{rx, ry, rz} {}

Types::HomogeneousM Arm::Posture::to_homogeneous() {
  auto const& [x, y, z] = Position;
  auto const& [rx, ry, rz] = Euler;
  Types::RotationM R;
  R.value = Hsu::get_Rz(rz) * Hsu::get_Ry(ry) * Hsu::get_Rx(rx);
  Types::TranslationM T;
  T.value << x(), y(), z();
  return {R, T};
}

Arm::Posture Arm::read_posture(std::string const& actor, std::string const& frame) {
  std::lock_guard<std::mutex> lock(mutex_);
  rm_current_arm_state_t state;

  auto res = detail::service_ins().rm_get_current_arm_state(handle_, &state);
  detail::throw_modbus_err("查询状态：", res);

  for (int i = 0; i < 7; i++) {
    frame_.set_joint_angle(i + 1, state.joint[i] * 1_deg);
  }

  std::shared_ptr<Hsu::Frame> Actor, Frame;

  if (actor == "Actor") {
    Actor = actor_;
  } else if (actor == "Wrist") {
    Actor = frame_.links[7];
  } else {
    ERROR(R"(可选 Actor 仅支持 "Actor" "Wrist")");
  }

  if (frame == "World") {
    Frame = Hsu::Frame::WORLD_FRAME();
  } else if (frame == "Base") {
    Frame = base_;
  } else {
    ERROR(R"(可选 Frame 仅支持 "World" "Base")");
  }

  auto&& Ha_f = Actor->get_homegeneous_relative_to(Frame);

  auto const& x = Ha_f.value(0, 3);
  auto const& y = Ha_f.value(1, 3);
  auto const& z = Ha_f.value(2, 3);

  auto&& euler = Types::RotationM(Ha_f).to_euler();

  auto const& rx = euler[0];
  auto const& ry = euler[1];
  auto const& rz = euler[2];

  return {{x * 1_m, y * 1_m, z * 1_m}, {rx * 1_rad, ry * 1_rad, rz * 1_rad}};
}

std::array<Types::HomogeneousM, 8> Arm::get_frames_data() {
  read_joint_angle();

  return frame_.get_data();
}

void Arm::set_base_offset(Types::HomogeneousM const& offset) {
  base_->set_translation(offset);
  frame_.set_base_offset(offset);
  return;
}

void Arm::set_actor_offset(Types::HomogeneousM const& offset) {
  actor_ = frame_.links[7]->define_frame("Actor", offset);
  return;
}

void Arm::move(Arm::Posture posture, int v, std::string const& actor, std::string const& frame) {
  rm_pose_t pose;

  std::shared_ptr<Hsu::Frame> Actor, Frame;
  auto const& World = Hsu::Frame::WORLD_FRAME();
  auto const& Wrist = frame_.links[7];

  if (actor == "Actor") {
    Actor = actor_;
  } else if (actor == "Wrist") {
    Actor = frame_.links[7];
  } else {
    ERROR(R"(可选 Actor 仅支持 "Actor" "Wrist")");
  }

  if (frame == "World") {
    Frame = Hsu::Frame::WORLD_FRAME();
  } else if (frame == "Base") {
    Frame = base_;
  } else {
    ERROR(R"(可选 Frame 仅支持 "World" "Base")");
  }

  auto&& Hf_b = Frame->get_homegeneous_relative_to(base_);
  auto&& Ha_f = posture.to_homogeneous();
  auto&& Hw_a = Actor->get_homegeneous_relative_to(Wrist).inv();

  auto&& Hw_b = Hf_b * Ha_f * Hw_a;

  Types::RotationM R(Hw_b);

  auto euler = R.to_euler();

  pose.position.x = Hw_b.value(0, 3);
  pose.position.y = Hw_b.value(1, 3);
  pose.position.z = Hw_b.value(2, 3);

  pose.euler.rx = euler[0];
  pose.euler.ry = euler[1];
  pose.euler.rz = euler[2];

  // TODO: 错误处理
  move_jp(pose, v);
}

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

namespace detail {

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
  throw std::runtime_error(msg);
}

}  // namespace detail

}  // namespace Hsu

namespace Hsu {
int Arm::read_holding_registers(rm_peripheral_read_write_params_t const& params) {
  int data;

  auto res = detail::service_ins().rm_read_holding_registers(handle_, params, &data);

  detail::throw_modbus_err(fmt::format("{}号机械臂读保持寄存器失败：", handle_->id), res);

  return data;
}

std::vector<int> Arm::read_multiple_holding_registers(rm_peripheral_read_write_params_t params, int const& len) {
  if (len <= 1) {
    throw std::invalid_argument("读多寄存器时，长度必须大于1");
  }

  static const int max_registers_per_read = 12;

  std::vector<int> res;
  res.reserve(len);

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

int Arm::read_input_registers(rm_peripheral_read_write_params_t const& params) {
  int data;

  auto res = detail::service_ins().rm_read_input_registers(handle_, params, &data);

  detail::throw_modbus_err(fmt::format("{}号机械臂读输入寄存器失败：", handle_->id), res);

  return data;
}

void Arm::write_single_register(rm_peripheral_read_write_params_t const& params, int const& data) {
  auto res = detail::service_ins().rm_write_single_register(handle_, params, data);

  detail::throw_modbus_err(fmt::format("{}号机械臂写单个寄存器失败：", handle_->id), res);
}

void Arm::write_multiple_registers(rm_peripheral_read_write_params_t params, std::vector<int> const& data) {
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

  for (int i = 0; i < len; i += max_registers_per_write) {
    params.num = std::min(max_registers_per_write, len - i);

    auto err = detail::service_ins().rm_write_registers(handle_, params, &data_bytes[i * 2]);

    detail::throw_modbus_err(fmt::format("{}号机械臂写多寄存器失败：", handle_->id), err);

    params.address += params.num * 2;

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

std::weak_ptr<ModbusActor_RTU> Arm::produce_modbus_actor(int port, int baudrate, int device, int timeout) {
  if (modbus_actor_) {
    ERROR("每个机械臂只能生产一个Modbus接口");
  }

  std::lock_guard<std::mutex> lock(mutex_);
  auto res = detail::service_ins().rm_set_modbus_mode(handle_, port, baudrate, timeout);
  detail::throw_modbus_err(fmt::format("{}号机械臂Modbus连接失败：", handle_->id), res);
  modbus_actor_ = std::shared_ptr<ModbusActor_RTU>(new ModbusActor_RTU(shared_from_this(), port, 1));

  return modbus_actor_;
}
}  // namespace Hsu

namespace Hsu {
ModbusActor_RTU::ModbusActor_RTU(std::weak_ptr<Arm> arm, int port, int device) noexcept : arm_(arm) {
  params_.port = port;
  params_.device = device;
}

int ModbusActor_RTU::read_holding_registers(int const& address) {
  params_.address = address;

  if (auto arm = arm_.lock()) {
    return arm->read_holding_registers(params_);
  } else {
    throw std::runtime_error("机械臂已被销毁，无法访问RTU接口");
  }
}

std::vector<int> ModbusActor_RTU::read_multiple_holding_registers(int const& address, int const& len) {
  params_.address = address;

  if (auto arm = arm_.lock()) {
    return arm->read_multiple_holding_registers(params_, len);
  } else {
    throw std::runtime_error("机械臂已被销毁，无法访问RTU接口");
  }
}

void ModbusActor_RTU::write_multiple_registers(int const& address, std::vector<int> const& data) {
  params_.address = address;

  if (auto arm = arm_.lock()) {
    arm->write_multiple_registers(params_, data);
  } else {
    throw std::runtime_error("机械臂已被销毁，无法访问RTU接口");
  }
}

int ModbusActor_RTU::read_input_registers(int const& address) {
  params_.address = address;

  if (auto arm = arm_.lock()) {
    return arm->read_input_registers(params_);
  } else {
    throw std::runtime_error("机械臂已被销毁，无法访问RTU接口");
  }
}

void ModbusActor_RTU::write_single_register(int const& address, int const& data) {
  params_.address = address;

  if (auto arm = arm_.lock()) {
    arm->write_single_register(params_, data);
  } else {
    throw std::runtime_error("机械臂已被销毁，无法访问RTU接口");
  }
}

}  // namespace Hsu
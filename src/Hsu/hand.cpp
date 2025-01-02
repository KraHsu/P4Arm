#include "units.h"
#include <Hsu/arm.h>
#include <Hsu/hand.h>
#include <fmt/format.h>
#include <fmt/ranges.h>
#include <chrono>
#include <cstring>
#include <exception>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>

#define DEBUG(...) Hsu::detail::arm_logger()->debug(__VA_ARGS__)
#define INFO(...) Hsu::detail::arm_logger()->info(__VA_ARGS__)
#define WARN(...) Hsu::detail::arm_logger()->warn(__VA_ARGS__)
#define ERROR(...) Hsu::detail::arm_logger()->error(__VA_ARGS__)

using namespace units;
using namespace units::literals;

static units::angle::degree_t map_data_to_angle(int data, units::angle::degree_t min, units::angle::degree_t max) {
  // 如果 data 超出范围，限制到 [0, 1000]
  data = std::clamp(data, 0, 1000);
  return min + (max - min) * (static_cast<double>(data) / 1000.0);
}

// 将 angle 映射为 data
static int map_angle_to_data(units::angle::degree_t angle, units::angle::degree_t min, units::angle::degree_t max) {
  if (angle < min || angle > max) {
    // 超出范围时，返回 -1
    return -1;
  }
  return static_cast<int>((angle - min) / (max - min) * 1000.0);
}

namespace Hsu {
Hand::Angles::Angles()
    : little(Retention), ring(Retention), middle(Retention), index(Retention), thumb(Retention), thumb_r(Retention) {}

Hand::Angles::Angles(units::angle::degree_t little, units::angle::degree_t ring, units::angle::degree_t middle,
                     units::angle::degree_t index, units::angle::degree_t thumb, units::angle::degree_t thumb_r)
    : little(little), ring(ring), middle(middle), index(index), thumb(thumb), thumb_r(thumb_r) {}

Hand::Angles::Angles(AnglesData const& data) {
  little = map_data_to_angle(data.data[0], 20_deg, 176_deg);
  ring = map_data_to_angle(data.data[1], 20_deg, 176_deg);
  middle = map_data_to_angle(data.data[2], 20_deg, 176_deg);
  index = map_data_to_angle(data.data[3], 20_deg, 176_deg);
  thumb = map_data_to_angle(data.data[4], -13_deg, 70_deg);
  thumb_r = map_data_to_angle(data.data[5], 90_deg, 165_deg);
}

Hand::Angles::operator AnglesData() const {
  AnglesData data;
  data.data[0] = map_angle_to_data(little, 20_deg, 176_deg);
  data.data[1] = map_angle_to_data(ring, 20_deg, 176_deg);
  data.data[2] = map_angle_to_data(middle, 20_deg, 176_deg);
  data.data[3] = map_angle_to_data(index, 20_deg, 176_deg);
  data.data[4] = map_angle_to_data(thumb, -13_deg, 70_deg);
  data.data[5] = map_angle_to_data(thumb_r, 90_deg, 165_deg);
  return data;
}

Hand::AnglesData::AnglesData() : data{-1, -1, -1, -1, -1, -1} {}
Hsu::Hand::AnglesData::AnglesData(const Hsu::Hand::Angles& angles) {
  data[0] = map_angle_to_data(angles.little, 20_deg, 176_deg);
  data[1] = map_angle_to_data(angles.ring, 20_deg, 176_deg);
  data[2] = map_angle_to_data(angles.middle, 20_deg, 176_deg);
  data[3] = map_angle_to_data(angles.index, 20_deg, 176_deg);
  data[4] = map_angle_to_data(angles.thumb, -13_deg, 70_deg);
  data[5] = map_angle_to_data(angles.thumb_r, 90_deg, 165_deg);
}
Hsu::Hand::AnglesData::operator Hsu::Hand::Angles() const {
  Angles angles;
  angles.little = map_data_to_angle(data[0], 20_deg, 176_deg);
  angles.ring = map_data_to_angle(data[1], 20_deg, 176_deg);
  angles.middle = map_data_to_angle(data[2], 20_deg, 176_deg);
  angles.index = map_data_to_angle(data[3], 20_deg, 176_deg);
  angles.thumb = map_data_to_angle(data[4], -13_deg, 70_deg);
  angles.thumb_r = map_data_to_angle(data[5], 90_deg, 165_deg);
  return angles;
}
}  // namespace Hsu

namespace Hsu {
Hand::Hand(std::shared_ptr<ModbusActorBase> ma) : ma_(ma) {}
Hand::~Hand() {}

bool Hand::clear_err() {
  try {
    std::lock_guard<std::mutex> lock(mutex_);
    ma_->write_single_register(1004, 1, 1);
  } catch (std::exception& e) {
    ERROR(e.what());
    return false;
  }
  return true;
}

Hand::Angles Hand::read_angles() {
  Angles res;
  AnglesData data;

  try {
    std::lock_guard<std::mutex> lock(mutex_);

    auto vec = ma_->read_multiple_holding_registers(1546, 1, 6);

    memcpy(data.data, vec.data(), sizeof(data.data));

    res = Angles(data);
  } catch (std::exception& e) {
    auto msg = "读取手指角度失败";
    ERROR(msg);
    throw std::runtime_error(msg);
  }

  return res;
}

bool Hand::set_angles(const Angles& angles) {
  this->angles_ = angles;

  AnglesData data(angles);

  try {
    std::lock_guard<std::mutex> lock(mutex_);

    ma_->write_multiple_registers(1486, 1, std::vector<int>(data.data, data.data + 6));
  } catch (std::exception& e) {
    auto msg = "写入手指角度失败";
    ERROR(msg);
    return false;
  }

  return true;
}

void Hand::read_tactile_(const std::exception& e) {
  auto msg = "读取触觉信息失败";
  ERROR(msg);
  throw std::runtime_error(msg);
}
}  // namespace Hsu

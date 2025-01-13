#include <Hsu/hand.h>
#include <Hsu/hsu_module_log.h>
#include <fmt/format.h>
#include <fmt/ranges.h>
#include <cstring>
#include <exception>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <units.h>

GENERATE_LOGGER(hand)

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
Hand::Hand(std::weak_ptr<ModbusActorBase> modbus_actor) noexcept : modbus_actor_(modbus_actor) {}

Hand::~Hand() {}

void Hand::clear_err() {
  try {
    std::lock_guard<std::mutex> lock(mutex_);
    if (auto ma = modbus_actor_.lock()) {
      ma->write_single_register(1004, 1);
    } else {
      throw std::runtime_error("接口已失效，清除错误失败！");
    }
  } catch (std::exception& e) {
    ERROR(e.what());
  }
}

Hand::Angles Hand::read_angles() {
  Angles res;
  AnglesData data;

  try {
    std::lock_guard<std::mutex> lock(mutex_);

    if (auto ma = modbus_actor_.lock()) {
      auto vec = ma->read_multiple_holding_registers(1546, 6);
      memcpy(data.data, vec.data(), sizeof(data.data));
      res = Angles(data);
    } else {
      throw std::runtime_error("接口已失效！");
    }

  } catch (std::exception& e) {
    ERROR("读取手指角度失败：{}", e.what());
  }

  return res;
}

void Hand::set_angles(const Angles& angles) {
  AnglesData data(angles);

  try {
    std::lock_guard<std::mutex> lock(mutex_);

    if (auto ma = modbus_actor_.lock()) {
      ma->write_multiple_registers(1486, std::vector<int>(data.data, data.data + 6));
    } else {
      throw std::runtime_error("接口已失效！");
    }
  } catch (std::exception& e) {
    ERROR("写入手指角度失败：{}", e.what());
  }
}

std::string Hand::read_err() {
  try {
    std::lock_guard<std::mutex> lock(mutex_);

    if (auto ma = modbus_actor_.lock()) {
      auto res = ma->read_multiple_holding_registers(1606, 3);

      return fmt::format("{}", fmt::join(res, ", "));
    } else {
      throw std::runtime_error("接口已失效！");
    }
  } catch (std::exception& e) {
    ERROR("写入手指角度失败：{}", e.what());
  }
}

void Hand::read_tactile_error(const std::exception& e) const {
  ERROR("读取触觉信息失败：{}", e.what());
  return;
}
}  // namespace Hsu

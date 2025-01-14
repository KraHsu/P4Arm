#include "Hsu/arm.h"
#include <Hsu/hand.h>
#include <Hsu/matrix_ops.h>
#include <fmt/ostream.h>
#include <nlohmann/json-schema.hpp>
#include <spdlog/spdlog.h>
#include <array>
#include <exception>
#include <handles.h>

using json = nlohmann::json;
using namespace units::literals;

// ---- LOGGER ----
#define DEBUG(...) \
  spdlog::log(spdlog::source_loc{__FILE__, __LINE__, SPDLOG_FUNCTION}, spdlog::level::debug, __VA_ARGS__)
#define INFO(...) spdlog::log(spdlog::source_loc{__FILE__, __LINE__, SPDLOG_FUNCTION}, spdlog::level::info, __VA_ARGS__)
#define WARN(...) spdlog::log(spdlog::source_loc{__FILE__, __LINE__, SPDLOG_FUNCTION}, spdlog::level::warn, __VA_ARGS__)
#define ERROR(...) spdlog::log(spdlog::source_loc{__FILE__, __LINE__, SPDLOG_FUNCTION}, spdlog::level::err, __VA_ARGS__)

Hsu::TCPConnection::Response handle_arm(std::shared_ptr<Hsu::Arm> arm, int code, json const& payload) {
  Hsu::TCPConnection::Response res;
  res.code = 300;

  static json schema101 =
      R"({"type":"object","properties":{"Actor":{"type":"string","enum":["Actor","Wrist"]},"Target":{"type":"object",")"
      R"(properties":{"Frame":{"type":"string","enum":["World","Base"]},"Position":{"type":"array","items":{"type":"nu)"
      R"(mber"},"minItems":3,"maxItems":3},"Euler":{"type":"array","items":{"type":"number"},"minItems":3,"maxItems":3)"
      R"(}},"required":["Frame","Position","Euler"]},"Speed":{"type":"integer","minimum":0,"maximum":100}},"required":)"
      R"(["Actor","Target","Speed"]})"_json;
  static nlohmann::json_schema::json_validator validator101;
  static json schema102 =
      R"({"type":"object","properties":{"Actor":{"type":"string","enum":["Actor","Wrist"]},"Frame":{"type":"string","e)"
      R"(num":["World","Base"]}},"required":["Actor","Frame"]})"_json;
  static nlohmann::json_schema::json_validator validator102;

  static bool _ = [&]() {
    validator101.set_root_schema(schema101);
    validator102.set_root_schema(schema102);
    return true;
  }();

  try {
    switch (code) {
      case 101: {
        validator101.validate(payload);

        auto const& [x, y, z] = payload["Target"]["Position"].get<std::array<double, 3>>();
        auto const& [rx, ry, rz] = payload["Target"]["Euler"].get<std::array<double, 3>>();

        Hsu::Arm::Posture posture(x * 1_m, y * 1_m, z * 1_m, rx * 1_rad, ry * 1_rad, rz * 1_rad);

        arm->move(posture, payload["Speed"].get<int>(), payload["Actor"].get<std::string>(),
                  payload["Target"]["Frame"].get<std::string>());
        break;
      }
      case 102: {
        validator102.validate(payload);
        auto&& posture = arm->read_posture(payload["Actor"], payload["Frame"]);
        auto const& [x, y, z] = posture.Position;
        auto const& [rx, ry, rz] = posture.Euler;
        res.payload = json::parse(
            fmt::format(R"({{"Position":[{},{},{}],"Euler":[{},{},{}]}})", x(), y(), z(), rx(), ry(), rz()));
        break;
      }
      case 103: {
        auto&& joint_angles = arm->read_joint_angle();

        res.payload = json::parse(fmt::format(R"({{"Joints":[{},{},{},{},{},{},{}]}})", joint_angles[0](),
                                              joint_angles[1](), joint_angles[2](), joint_angles[3](),
                                              joint_angles[4](), joint_angles[5](), joint_angles[6]()));
        break;
      }
      case 104: {
        auto&& joint_angles = arm->stop();

        res.payload = json::parse("{}");
        break;
      }
      case 105: {
        validator102.validate(payload);
        auto&& posture = arm->read_target(payload["Actor"], payload["Frame"]);
        auto const& [x, y, z] = posture.Position;
        auto const& [rx, ry, rz] = posture.Euler;
        res.payload = json::parse(
            fmt::format(R"({{"Position":[{},{},{}],"Euler":[{},{},{}]}})", x(), y(), z(), rx(), ry(), rz()));
        break;
      }
      case 106: {
        if (payload.contains("mode")) {
          arm->set_mode(payload["mode"].get<int>());
        }

        res.payload = json::parse("{}");
        break;
      }
      default: {
        res.code = 400;
        res.payload = {{"message", "Error code."}};
        break;
      }
    }
  } catch (std::exception& e) {
    res.code = 400;
    res.payload = {{"message", e.what()}};
    ERROR(e.what());
  }

  return res;
}

Hsu::TCPConnection::Response handle_hand(std::shared_ptr<Hsu::Hand> hand, int code, json const& payload) {
  Hsu::TCPConnection::Response res;
  res.code = 300;

  if (!hand) {
    res.code = 400;
    res.payload = {{"message", "Null hand."}};
    return res;
  }

  try {
    switch (code) {
      case 101: {
        if (!payload.contains("little") and !payload.contains("ring") and !payload.contains("middle") and
            !payload.contains("index") and !payload.contains("thumb") and !payload.contains("thumb_r")) {
          res.code = 400;
          res.payload = {{"message", "Ivalid key."}};
          break;
        }
        Hsu::Hand::Angles angles;

        angles.little = payload.value("little", -1024.0) * 1_deg;
        angles.ring = payload.value("ring", -1024.0) * 1_deg;
        angles.middle = payload.value("middle", -1024.0) * 1_deg;
        angles.index = payload.value("index", -1024.0) * 1_deg;
        angles.thumb = payload.value("thumb", -1024.0) * 1_deg;
        angles.thumb_r = payload.value("thumb_r", -1024.0) * 1_deg;

        hand->set_angles(angles);

        break;
      }
      case 102: {
        if (!payload.contains("config")) {
          res.code = 400;
          res.payload = {{"message", "Ivalid key."}};
          break;
        }
        res.payload = read_tactile_data(hand, payload["config"]);
        break;
      }
      default: {
        res.code = 400;
        res.payload = {{"message", "Error code."}};
        break;
      }
    }
  } catch (std::exception& e) {
    res.code = 400;
    res.payload = {{"message", e.what()}};
    spdlog::error(e.what());
  }

  return res;
}

bool limited_move_jp(std::shared_ptr<Hsu::Arm> arm, rm_position_t position, rm_euler_t posture) {
  auto cd = collision_detected(position, posture);

  if (cd) {
    return false;
  }

  // arm->move_jp(position, posture, 10);

  return true;
}

bool collision_detected(rm_position_t position, rm_euler_t euler) {
  // Eigen::Vector3d end_pos(position.x, position.y, position.z);
  // Eigen::Vector3d euler_angles(euler.rx, euler.ry, euler.rz);

  // Eigen::Vector3d prev_pos = Hsu::compute_previous_joint_position(end_pos, euler_angles, L_ARM);
  // Eigen::Vector3d next_pos = Hsu::compute_next_joint_position(end_pos, euler_angles, L_HAND);

  // auto in_body = [](double const& x, double const& y, double const& z) -> bool {
  //   return x <= 0.2 && y <= 0.3 && y >= -0.3;
  // };

  // auto in_base = [](double const& x, double const& y, double const& z) -> bool { return z <= -0.32; };

  // if (in_body(prev_pos[0], prev_pos[1], prev_pos[2]) || in_body(next_pos[0], next_pos[1], next_pos[2])) {
  //   return true;
  // }

  // if (in_base(prev_pos[0], prev_pos[1], prev_pos[2]) || in_base(next_pos[0], next_pos[1], next_pos[2])) {
  //   return true;
  // }

  return false;
}

template <typename T, int Rows, int Cols>
json eigen_to_json_array(const Eigen::Matrix<T, Rows, Cols>& matrix) {
  json result = json::array();
  for (int i = 0; i < Rows; ++i) {
    json row = json::array();
    for (int j = 0; j < Cols; ++j) {
      row.push_back(matrix(i, j));
    }
    result.push_back(row);
  }
  return result;
}

json read_tactile_data(std::shared_ptr<Hsu::Hand> hand, const json& config) {
  using TT = Hsu::Hand::TactileType;

  json tactile_data;

  try {
    // Little finger
    if (config.contains("Little")) {
      if (config["Little"].contains("Tip") && config["Little"]["Tip"].get<bool>()) {
        auto res_LittleTip = hand->read_tactile<TT::LittleTip>();
        tactile_data["Little"]["Tip"] = eigen_to_json_array(res_LittleTip);
      }
      if (config["Little"].contains("Distal") && config["Little"]["Distal"].get<bool>()) {
        auto res_LittleDistal = hand->read_tactile<TT::LittleDistal>();
        tactile_data["Little"]["Distal"] = eigen_to_json_array(res_LittleDistal);
      }
      if (config["Little"].contains("Pad") && config["Little"]["Pad"].get<bool>()) {
        auto res_LittlePad = hand->read_tactile<TT::LittlePad>();
        tactile_data["Little"]["Pad"] = eigen_to_json_array(res_LittlePad);
      }
    }

    // Ring finger
    if (config.contains("Ring")) {
      if (config["Ring"].contains("Tip") && config["Ring"]["Tip"].get<bool>()) {
        auto res_RingTip = hand->read_tactile<TT::RingTip>();
        tactile_data["Ring"]["Tip"] = eigen_to_json_array(res_RingTip);
      }
      if (config["Ring"].contains("Distal") && config["Ring"]["Distal"].get<bool>()) {
        auto res_RingDistal = hand->read_tactile<TT::RingDistal>();
        tactile_data["Ring"]["Distal"] = eigen_to_json_array(res_RingDistal);
      }
      if (config["Ring"].contains("Pad") && config["Ring"]["Pad"].get<bool>()) {
        auto res_RingPad = hand->read_tactile<TT::RingPad>();
        tactile_data["Ring"]["Pad"] = eigen_to_json_array(res_RingPad);
      }
    }

    // Middle finger
    if (config.contains("Middle")) {
      if (config["Middle"].contains("Tip") && config["Middle"]["Tip"].get<bool>()) {
        auto res_MiddleTip = hand->read_tactile<TT::MiddleTip>();
        tactile_data["Middle"]["Tip"] = eigen_to_json_array(res_MiddleTip);
      }
      if (config["Middle"].contains("Distal") && config["Middle"]["Distal"].get<bool>()) {
        auto res_MiddleDistal = hand->read_tactile<TT::MiddleDistal>();
        tactile_data["Middle"]["Distal"] = eigen_to_json_array(res_MiddleDistal);
      }
      if (config["Middle"].contains("Pad") && config["Middle"]["Pad"].get<bool>()) {
        auto res_MiddlePad = hand->read_tactile<TT::MiddlePad>();
        tactile_data["Middle"]["Pad"] = eigen_to_json_array(res_MiddlePad);
      }
    }

    // Index finger
    if (config.contains("Index")) {
      if (config["Index"].contains("Tip") && config["Index"]["Tip"].get<bool>()) {
        auto res_IndexTip = hand->read_tactile<TT::IndexTip>();
        tactile_data["Index"]["Tip"] = eigen_to_json_array(res_IndexTip);
      }
      if (config["Index"].contains("Distal") && config["Index"]["Distal"].get<bool>()) {
        auto res_IndexDistal = hand->read_tactile<TT::IndexDistal>();
        tactile_data["Index"]["Distal"] = eigen_to_json_array(res_IndexDistal);
      }
      if (config["Index"].contains("Pad") && config["Index"]["Pad"].get<bool>()) {
        auto res_IndexPad = hand->read_tactile<TT::IndexPad>();
        tactile_data["Index"]["Pad"] = eigen_to_json_array(res_IndexPad);
      }
    }

    // Thumb
    if (config.contains("Thumb")) {
      if (config["Thumb"].contains("Tip") && config["Thumb"]["Tip"].get<bool>()) {
        auto res_ThumbTip = hand->read_tactile<TT::ThumbTip>();
        tactile_data["Thumb"]["Tip"] = eigen_to_json_array(res_ThumbTip);
      }
      if (config["Thumb"].contains("Distal") && config["Thumb"]["Distal"].get<bool>()) {
        auto res_ThumbDistal = hand->read_tactile<TT::ThumbDistal>();
        tactile_data["Thumb"]["Distal"] = eigen_to_json_array(res_ThumbDistal);
      }
      if (config["Thumb"].contains("Middle") && config["Thumb"]["Middle"].get<bool>()) {
        auto res_ThumbMiddle = hand->read_tactile<TT::ThumbMiddle>();
        tactile_data["Thumb"]["Middle"] = eigen_to_json_array(res_ThumbMiddle);
      }
      if (config["Thumb"].contains("Pad") && config["Thumb"]["Pad"].get<bool>()) {
        auto res_ThumbPad = hand->read_tactile<TT::ThumbPad>();
        tactile_data["Thumb"]["Pad"] = eigen_to_json_array(res_ThumbPad);
      }
    }

    // Palm
    if (config.contains("Palm") && config["Palm"].get<bool>()) {
      auto res_Palm = hand->read_tactile<TT::Palm>();
      tactile_data["Palm"] = eigen_to_json_array(res_Palm);
    }
  } catch (const std::exception& e) {
    ERROR(e.what());
  }

  return tactile_data;
}
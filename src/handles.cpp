#include <Hsu/hand.h>
#include <Hsu/matrix_ops.h>
#include <spdlog/spdlog.h>
#include <exception>
#include <handles.h>

using json = nlohmann::json;
using namespace units::literals;

// ---- LOGGER ----
#define DEBUG(...) spdlog::debug(__VA_ARGS__)
#define INFO(...) spdlog::info(__VA_ARGS__)
#define WARN(...) spdlog::warn(__VA_ARGS__)
#define ERROR(...) spdlog::error(__VA_ARGS__)

Hsu::TCPConnection::Response handle_move(std::shared_ptr<Hsu::Arm> left, std::shared_ptr<Hsu::Arm> right, int code,
                                         json const& payload) {
  std::shared_ptr<Hsu::Arm> arm;

  Hsu::TCPConnection::Response res;
  res.code = 0;

  if (code >= 50 and code <= 55) {
    arm = right;
    code -= 50;
  } else if (code >= 60 and code <= 65) {
    arm = left;
    code -= 60;
  } else {
    res.code = 100;
    res.payload = {{"message", "Error code."}};
    return res;
  }

  try {
    switch (code) {
      case 0: {
        if (!payload.contains("coor")) {
          res.code = 100;
          res.payload = {{"message", "Ivalid key."}};
          break;
        }
        json coor = payload["coor"];
        if (coor.is_array() && coor.size() == 6) {
          if (!limited_move_jp(arm, rm_position_t{coor[0], coor[1], coor[2]}, rm_euler_t{coor[3], coor[4], coor[5]})) {
            res.code = 100;
            res.payload = {{"message", "Out of limit."}};
          }
          break;
        }
        res.code = 100;
        res.payload = {{"message", "Ivalid value of coor."}};
        break;
      }
      case 1: {
        auto pose = json::array();
        auto state = arm->get_state();

        pose.push_back(state.pose.position.x);
        pose.push_back(state.pose.position.y);
        pose.push_back(state.pose.position.z);
        pose.push_back(state.pose.euler.rx);
        pose.push_back(state.pose.euler.ry);
        pose.push_back(state.pose.euler.rz);

        res.payload = {{"pose", pose}};
        break;
      }
      case 2: {
        auto pose = json::array();
        auto state = arm->get_state();

        for (auto const& theta : state.joint) {
          pose.push_back(theta);
        }

        res.payload = {{"pose", pose}};
        break;
      }
      case 3:
      case 4:
      case 5: {
        res.code = 100;
        res.payload = {{"message", "Unsupport now!"}};
        break;
      }
    }
  } catch (std::exception& e) {
    res.code = 100;
    res.payload = {{"message", e.what()}};
    spdlog::error(e.what());
  }

  return res;
}

Hsu::TCPConnection::Response handle_hand(std::shared_ptr<Hsu::Hand> left, std::shared_ptr<Hsu::Hand> right, int code,
                                         json const& payload) {
  std::shared_ptr<Hsu::Hand> hand;

  Hsu::TCPConnection::Response res;
  res.code = 0;

  if (code >= 50 and code <= 55) {
    hand = right;
    code -= 50;
  } else if (code >= 60 and code <= 65) {
    hand = left;
    code -= 60;
  } else {
    res.code = 100;
    res.payload = {{"message", "Error code."}};
    return res;
  }

  if (!hand) {
    res.code = 100;
    res.payload = {{"message", "Null hand."}};
    return res;
  }

  try {
    switch (code) {
      case 0: {
        if (!payload.contains("little") and !payload.contains("ring") and !payload.contains("middle") and
            !payload.contains("index") and !payload.contains("thumb") and !payload.contains("thumb_r")) {
          res.code = 100;
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
      case 1: {
        if (!payload.contains("config")) {
          res.code = 100;
          res.payload = {{"message", "Ivalid key."}};
          break;
        }
        res.payload = read_tactile_data(hand, payload["config"]);
        break;
      }
    }
  } catch (std::exception& e) {
    res.code = 100;
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

  arm->move_jp(position, posture, 10);

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
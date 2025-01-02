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

json get_all_data(std::shared_ptr<Hsu::Hand> hand, const json& config);

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
        res.payload = get_all_data(hand, payload["config"]);
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

// Hsu::TCPConnection::Response handle_grip(Hsu::Arm* left, Hsu::Arm* right, int code, json const& payload) {
//   Hsu::Arm* arm;

//   Hsu::TCPConnection::Response res;
//   res.code = 0;

//   if (code == 50) {
//     arm = right;
//   } else if (code == 60) {
//     arm = left;
//   } else {
//     res.code = 100;
//     res.payload = {{"message", "Error code."}};
//     return res;
//   }

//   if (!payload.contains("holding")) {
//     res.code = 100;
//     res.payload = {{"message", "Invalid key!"}};
//   }

//   arm->set_grip_position(payload["holding"]);

//   return res;
// }

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
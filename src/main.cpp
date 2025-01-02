// ---- Hsu ----
#include <Hsu/arm.h>
#include <Hsu/hand.h>
#include <Hsu/matrix_ops.h>
#include <Hsu/modbus_tcp.h>
#include <Hsu/tcp.h>
// ---- RMArm ----
#include <RMArm/rm_define.h>
#include <RMArm/rm_service.h>
// ---- standard ----
#include <atomic>
#include <chrono>
#include <exception>
#include <functional>
#include <future>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <sys/time.h>
#include <termios.h>
#include <unistd.h>
// ---- thrid ----
#include <fmt/base.h>
#include <fmt/core.h>
#include <fmt/ranges.h>
#include <nlohmann/json.hpp>
#include <spdlog/common.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
// ---- main ----
#include <handles.h>

using namespace Eigen;
using json = nlohmann::json;
using namespace units::literals;
using TT = Hsu::Hand::TactileType;

// ---- LOGGER ----
#define DEBUG(...) spdlog::debug(__VA_ARGS__)
#define INFO(...) spdlog::info(__VA_ARGS__)
#define WARN(...) spdlog::warn(__VA_ARGS__)
#define ERROR(...) spdlog::error(__VA_ARGS__)

// ---- DEBUG ----
#define IS_TEST false

// ---- MACRO ----
#define sleep_s(s) std::this_thread::sleep_for(std::chrono::seconds(s));
#define sleep_ms(ms) std::this_thread::sleep_for(std::chrono::milliseconds(ms));

bool STOP = false;

void set_up_main_logger();

std::mutex cout_lock{};

float r2d(float rad) { return rad * (180.0f / M_PI); }

float d2r(float deg) { return deg * (M_PI / 180.0f); }

void collision_detection_thread(std::shared_ptr<Hsu::Arm> arm, rm_position_t pos,
                                std::function<bool(rm_position_t, rm_euler_t)> detected) {
  try {
    while (!STOP) {
      cout_lock.lock();
      auto pose_now = arm->get_state().pose;
      if (detected(pose_now.position, pose_now.euler)) {
        spdlog::warn("机械臂[{}]碰撞电子围栏！", arm->get_id());
        arm->stop();

        arm->move_jp_force(pos, rm_euler_t{0, 0, 0}, 20, 0, 0, 1);

        arm->restart();
      }
      cout_lock.unlock();

      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  } catch (std::exception& e) {
    ERROR(e.what());
  }
}

void Hsu::Hand::test() {}

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

json get_all_data(std::shared_ptr<Hsu::Hand> hand, const json& config) {
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

int main() {
  std::shared_ptr<Hsu::Arm> left_arm, right_arm;

  std::shared_ptr<Hsu::TCPConnection> server;

  std::shared_ptr<Hsu::Hand> left_hand, right_hand;

  set_up_main_logger();
  auto test = Hsu::ModbusTCP("192.168.11.210", 6000);

  try {
    left_arm = std::make_shared<Hsu::Arm>("192.168.1.18", 8080);
    right_arm = std::make_shared<Hsu::Arm>("192.168.2.18", 8080);

    // right_hand = std::make_shared<Hsu::Hand>(right_arm->connect_modbus_actor(1, 115200, 10));
    right_hand = nullptr;
    // left_hand = std::make_shared<Hsu::Hand>(left_arm->connect_modbus_actor(1, 115200, 10));
    left_hand = std::make_shared<Hsu::Hand>(test.to_actor());

    server = std::make_shared<Hsu::TCPConnection>("127.0.0.1", 5000);

    server->connect("move", [&](int code, json const& payload) {
      auto res = handle_move(left_arm, right_arm, code, payload);
      return res;
    });

    server->connect("hand", [&](int code, json const& payload) {
      auto res = handle_hand(left_hand, right_hand, code, payload);
      return res;
    });

    // server->connect("grip", [&](int code, json const& payload) {
    //   auto res = handle_grip(left_arm, right_arm, code, payload);
    //   return res;
    // });

  } catch (std::exception const& e) {
    ERROR(e.what());
    return -1;
  }

  left_arm->set_hand_len(0.14);
  right_arm->set_hand_len(0.14);

  left_arm->set_offset(rm_position_t{0, -0.209, 0});
  right_arm->set_offset(rm_position_t{0, 0.209, 0});

  right_arm->set_mode(1);
  left_arm->set_mode(1);

  limited_move_jp(left_arm, rm_position_t{0.7, 0.40, -0.15}, rm_euler_t{d2r(-90), d2r(25), d2r(-90)});
  limited_move_jp(right_arm, rm_position_t{0.7, -0.40, -0.15}, rm_euler_t{d2r(90), d2r(25), d2r(90)});

  Hsu::Hand::Angles open_angles(176_deg, 176_deg, 176_deg, 176_deg, 70_deg, 120_deg);
  Hsu::Hand::Angles close_angles(20_deg, 20_deg, 20_deg, 20_deg, 50_deg, 90_deg);

  // for (int i = 0; i < 5; i++) {
  // 读取当前状态
  auto res_left = left_hand->read_angles();
  INFO("左手当前状态: {}", res_left);
  // auto res_right = right_hand->read_angles();
  // INFO("右手当前状态: {}", res_right);

  // 设置为张开状态
  INFO("设置为张开状态...");
  left_hand->set_angles(open_angles);
  INFO("左手状态: {}", open_angles);
  // right_hand->set_angles(open_angles);
  // INFO("右手状态: {}", open_angles);

  // 等待 1 秒
  sleep_s(2);

  // 设置为合拢状态
  // INFO("设置为合拢状态...");
  // left_hand->set_angles(close_angles);
  // INFO("左手状态: {}", close_angles);
  // right_hand->set_angles(close_angles);
  // INFO("右手状态: {}", close_angles);

  // // 再次等待 1 秒
  // sleep_s(2);
  // }

  // try {
  //   get_all_data(left_hand);
  // } catch (std::exception& e) {
  //   ERROR("TEST ERR: {}", e.what());
  //   exit(-1);
  // }

  std::this_thread::sleep_for(std::chrono::seconds(1));

  std::thread end([]() {
    std::string str;
    std::cin >> str;
  });

  server->start_server();

  std::thread left_cd(collision_detection_thread, left_arm, rm_position_t{0.6, 0.159, -0.2}, collision_detected);
  std::thread right_cd(collision_detection_thread, right_arm, rm_position_t{0.6, -0.159, -0.2}, collision_detected);

  end.join();

  STOP = true;

  server->stop_server();

  // left_cd.join();
  // right_cd.join();

  return 0;
}

void set_up_main_logger() {
  auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>("./log/main.log", true);
  auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();

  console_sink->set_pattern("[%H:%M:%S.%e] [thread %t] [%^%l%$] %v");
  file_sink->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [thread %t] [%^%l%$] %v");

  std::vector<spdlog::sink_ptr> sinks{file_sink, console_sink};
  auto logger = std::make_shared<spdlog::logger>("main_logger", sinks.begin(), sinks.end());

  spdlog::set_default_logger(logger);

  spdlog::set_level(spdlog::level::debug);
  spdlog::flush_on(spdlog::level::debug);

  INFO(" ==== MAIN ==== ");
}

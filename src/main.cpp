// ---- Hsu ----
#include <Hsu/arm.h>
#include <Hsu/frame.h>
#include <Hsu/hand.h>
#include <Hsu/matrix_ops.h>
#include <Hsu/modbus_tcp.h>
#include <Hsu/tcp.h>
// ---- RMArm ----
#include <RMArm/rm_define.h>
#include <RMArm/rm_service.h>
// ---- standard ----
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>
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

// ---- DEBUG ----
#define IS_TEST false

// ---- MACRO ----
#define DEBUG(...) spdlog::debug(__VA_ARGS__)
#define INFO(...) spdlog::info(__VA_ARGS__)
#define WARN(...) spdlog::warn(__VA_ARGS__)
#define ERROR(...) spdlog::error(__VA_ARGS__)
#define sleep_s(s) std::this_thread::sleep_for(std::chrono::seconds(s));
#define sleep_ms(ms) std::this_thread::sleep_for(std::chrono::milliseconds(ms));

#define WHILE_RUNNING while (MAIN_IS_RUNNING)

// ---- GLOBAL ----
bool MAIN_IS_RUNNING = true;

void set_up_main_logger();

void signalHandler(int signal);

void collision_detection_thread(std::shared_ptr<Hsu::Arm> arm, rm_position_t pos,
                                std::function<bool(rm_position_t, rm_euler_t)> detected) {
  try {
    WHILE_RUNNING {
      auto pose_now = arm->get_state().pose;
      if (detected(pose_now.position, pose_now.euler)) {
        spdlog::warn("机械臂[{}]碰撞电子围栏！", arm->get_id());
        arm->stop();

        arm->move_jp_force(pos, rm_euler_t{0, 0, 0}, 20, 0, 0, 1);

        arm->restart();
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  } catch (std::exception& e) {
    ERROR(e.what());
  }
}

#if defined(HSU_FRAME_VISUAL)
#include <pybind11/pytypes.h>
void VisualThread() {
  INFO("注意此时开启可视化");
  try {
    auto& sc = Hsu::Frame3DScene::instance();

    sc.begin();

    sc.start();

    std::this_thread::sleep_for(std::chrono::seconds(1));

    Hsu::Arm::Frames frames_l;
    Hsu::Arm::Frames frames_r;

    Hsu::Types::RotationM R;
    Hsu::Types::TranslationM T;
    double d = std::sqrt(2) / 2;

    R.value.row(0) << +0, +1, +0;
    R.value.row(1) << +d, +0, +d;
    R.value.row(2) << +d, +0, -d;
    T.value << 0, 0.209, 0;

    frames_l.set_base_offset({R, T});

    R.value.row(0) << +0, -1, +0;
    R.value.row(1) << -d, +0, -d;
    R.value.row(2) << +d, +0, -d;
    T.value << 0, -0.209, 0;

    frames_r.set_base_offset({R, T});

    sc.set_arm_l_data(frames_l.get_data());
    sc.set_arm_r_data(frames_r.get_data());

    auto deg = 0_deg;

    frames_l.set_joint_angle(1, 90_deg);
    frames_r.set_joint_angle(1, -90_deg);

    double duration = 5.0;                             // 总时间
    double total_angle = 90.0;                         // 总角度
    double angular_velocity = total_angle / duration;  // 每秒旋转的角速度

    auto begin = std::chrono::steady_clock::now();

    while (true) {
      // 获取当前时间
      auto now = std::chrono::steady_clock::now();
      std::chrono::duration<double> elapsed = now - begin;  // 计算经过的时间

      // 如果超过5秒，停止
      if (elapsed.count() >= duration) {
        break;
      }

      // 计算当前角度
      auto angle = angular_velocity * elapsed.count() * 1_deg;  // 当前角度
      frames_r.set_joint_angle(2, angle);                       // 设置右臂关节角度
      frames_l.set_joint_angle(2, angle);

      // 更新数据
      sc.set_arm_l_data(frames_l.get_data());
      sc.set_arm_r_data(frames_r.get_data());
    }

    duration = 3.0;                             // 总时间
    total_angle = 60.0;                         // 总角度
    angular_velocity = total_angle / duration;  // 每秒旋转的角速度

    begin = std::chrono::steady_clock::now();

    while (true) {
      // 获取当前时间
      auto now = std::chrono::steady_clock::now();
      std::chrono::duration<double> elapsed = now - begin;  // 计算经过的时间

      // 如果超过5秒，停止
      if (elapsed.count() >= duration) {
        break;
      }

      // 计算当前角度
      auto angle = angular_velocity * elapsed.count() * 1_deg;  // 当前角度
      frames_r.set_joint_angle(7, -angle);                      // 设置右臂关节角度
      frames_l.set_joint_angle(7, angle);

      // 更新数据
      sc.set_arm_l_data(frames_l.get_data());
      sc.set_arm_r_data(frames_r.get_data());
    }

    // sc.add_frames(world)
    //     .add_frames(base_link_l)
    //     .add_frames(link1_l)
    //     .add_frames(link2_l)
    //     .add_frames(link3_l)
    //     .add_frames(link4_l)
    //     .add_frames(link5_l)
    //     .add_frames(link6_l)
    //     .add_frames(link7_l)
    //     .add_frames(base_link_r)
    //     .add_frames(link1_r)
    //     .add_frames(link2_r)
    //     .add_frames(link3_r)
    //     .add_frames(link4_r)
    //     .add_frames(link5_r)
    //     .add_frames(link6_r)
    //     .add_frames(link7_r);

    int x;

    std::cin >> x;

    sc.stop();

  } catch (const pybind11::error_already_set& e) {
    std::cerr << "Main Error: " << e.what() << std::endl;
  }
}
#else
void VisualThread() {}
#endif

int main() {
  // VisualThread();

  std::signal(SIGINT, signalHandler);

  set_up_main_logger();

  std::shared_ptr<Hsu::Arm> left_arm, right_arm;

  std::shared_ptr<Hsu::TCPConnection> server;

  std::shared_ptr<Hsu::Hand> left_hand, right_hand;

  std::shared_ptr<Hsu::ModbusTCP> left_tcp_modbus, right_tcp_modbus;

  try {
    left_arm = std::make_shared<Hsu::Arm>("192.168.1.18", 8080);
    right_arm = std::make_shared<Hsu::Arm>("192.168.2.18", 8080);

    left_tcp_modbus = std::make_shared<Hsu::ModbusTCP>("192.168.12.210", 6000);
    right_tcp_modbus = std::make_shared<Hsu::ModbusTCP>("192.168.11.210", 6000);

    left_hand = std::make_shared<Hsu::Hand>(left_tcp_modbus->produce_modbus_actor());
    right_hand = std::make_shared<Hsu::Hand>(right_tcp_modbus->produce_modbus_actor());

    server = std::make_shared<Hsu::TCPConnection>("127.0.0.1", 5000);

    server->connect("move", [&](int code, json const& payload) {
      auto res = handle_move(left_arm, right_arm, code, payload);
      return res;
    });

    server->connect("hand", [&](int code, json const& payload) {
      auto res = handle_hand(left_hand, right_hand, code, payload);
      return res;
    });

  } catch (std::exception const& e) {
    ERROR(e.what());
    return -1;
  }

  // left_arm->set_hand_len(0.0);
  // right_arm->set_hand_len(0.0);

  // left_arm->set_offset(rm_position_t{0, -0.209, 0});
  // right_arm->set_offset(rm_position_t{0, 0.209, 0});

  right_arm->set_mode(1);
  left_arm->set_mode(1);

  // limited_move_jp(left_arm, rm_position_t{0.7, 0.40, -0.15},
  //                 rm_euler_t{-90_deg / 1_rad, 25_deg / 1_rad, -90_deg / 1_rad});
  // limited_move_jp(right_arm, rm_position_t{0.7, -0.40, -0.15},
  //                 rm_euler_t{90_deg / 1_rad, 25_deg / 1_rad, 90_deg / 1_rad});

  // Hsu::Hand::Angles open_angles(176_deg, 176_deg, 176_deg, 176_deg, 70_deg, 120_deg);
  // Hsu::Hand::Angles close_angles(20_deg, 20_deg, 20_deg, 20_deg, 50_deg, 90_deg);

  // left_hand->set_angles(close_angles);
  // right_hand->set_angles(close_angles);

  // sleep_s(2);

  // left_hand->set_angles(open_angles);
  // right_hand->set_angles(open_angles);

  server->start_server();

  // std::thread left_cd(collision_detection_thread, left_arm, rm_position_t{0.6, 0.159, -0.2}, collision_detected);
  // std::thread right_cd(collision_detection_thread, right_arm, rm_position_t{0.6, -0.159, -0.2}, collision_detected);
#if defined(HSU_FRAME_VISUAL)
  try {
    auto& sc = Hsu::Frame3DScene::instance();

    sc.begin();

    sc.start();

    std::this_thread::sleep_for(std::chrono::seconds(1));

    Hsu::Types::RotationM R;
    Hsu::Types::TranslationM T;
    double d = std::sqrt(2) / 2;

    R.value.row(0) << +0, +1, +0;
    R.value.row(1) << +d, +0, +d;
    R.value.row(2) << +d, +0, -d;
    T.value << 0, 0.209, 0;

    left_arm->set_base_offset({R, T});

    R.value.row(0) << +0, -1, +0;
    R.value.row(1) << -d, +0, -d;
    R.value.row(2) << +d, +0, -d;
    T.value << 0, -0.209, 0;

    right_arm->set_base_offset({R, T});

    WHILE_RUNNING {
      sc.set_arm_l_data(left_arm->get_frames_data());
      sc.set_arm_r_data(right_arm->get_frames_data());

      sleep_ms(100);
    }

    sc.stop();
  } catch (const pybind11::error_already_set& e) {
    std::cerr << "Main Error: " << e.what() << std::endl;
  }
#else
  WHILE_RUNNING { sleep_ms(100); }
#endif
  INFO("主程序已停止");

  server->stop_server();

  // // left_cd.join();
  // // right_cd.join();

  return 0;
}

void set_up_main_logger() {
  auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>("./log/main.log", false);
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

void signalHandler(int signal) {
  if (signal == SIGINT) {
    static auto first = true;
    if (first) {
      WARN("捕获到 Ctrl+C (SIGINT) 信号，程序正在停止...");

      MAIN_IS_RUNNING = false;

      first = false;
    } else {
      ERROR("无法自然停止，已强制程序停止...");
      exit(1);
    }
  }
}

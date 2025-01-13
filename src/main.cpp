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
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <functional>
#include <memory>
#include <stdexcept>
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
#define DEBUG(...) \
  spdlog::log(spdlog::source_loc{__FILE__, __LINE__, SPDLOG_FUNCTION}, spdlog::level::debug, __VA_ARGS__)
#define INFO(...) spdlog::log(spdlog::source_loc{__FILE__, __LINE__, SPDLOG_FUNCTION}, spdlog::level::info, __VA_ARGS__)
#define WARN(...) spdlog::log(spdlog::source_loc{__FILE__, __LINE__, SPDLOG_FUNCTION}, spdlog::level::warn, __VA_ARGS__)
#define ERROR(...) spdlog::log(spdlog::source_loc{__FILE__, __LINE__, SPDLOG_FUNCTION}, spdlog::level::err, __VA_ARGS__)

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

// int main() {
//   std::signal(SIGINT, signalHandler);

//   set_up_main_logger();

//   std::shared_ptr<Hsu::Arm> left_arm, right_arm;

//   std::shared_ptr<Hsu::TCPConnection> server;

//   std::shared_ptr<Hsu::Hand> left_hand, right_hand;

//   std::shared_ptr<Hsu::ModbusTCP> left_tcp_modbus, right_tcp_modbus;

//   try {
//     left_arm = std::make_shared<Hsu::Arm>("192.168.1.18", 8080);
//     right_arm = std::make_shared<Hsu::Arm>("192.168.2.18", 8080);

//     left_tcp_modbus = std::make_shared<Hsu::ModbusTCP>("192.168.12.210", 6000);
//     right_tcp_modbus = std::make_shared<Hsu::ModbusTCP>("192.168.11.210", 6000);

//     left_hand = std::make_shared<Hsu::Hand>(left_tcp_modbus->produce_modbus_actor());
//     right_hand = std::make_shared<Hsu::Hand>(right_tcp_modbus->produce_modbus_actor());

//     server = std::make_shared<Hsu::TCPConnection>("127.0.0.1", 5000);

//     server->connect("move", [&](int code, json const& payload) {
//       auto res = handle_move(left_arm, right_arm, code, payload);
//       return res;
//     });

//     server->connect("hand", [&](int code, json const& payload) {
//       auto res = handle_hand(left_hand, right_hand, code, payload);
//       return res;
//     });

//   } catch (std::exception const& e) {
//     ERROR(e.what());
//     return -1;
//   }

//   // left_arm->set_hand_len(0.0);
//   // right_arm->set_hand_len(0.0);

//   // left_arm->set_offset(rm_position_t{0, -0.209, 0});
//   // right_arm->set_offset(rm_position_t{0, 0.209, 0});

//   right_arm->set_mode(1);
//   left_arm->set_mode(1);

//   // limited_move_jp(left_arm, rm_position_t{0.7, 0.40, -0.15},
//   //                 rm_euler_t{-90_deg / 1_rad, 25_deg / 1_rad, -90_deg / 1_rad});
//   // limited_move_jp(right_arm, rm_position_t{0.7, -0.40, -0.15},
//   //                 rm_euler_t{90_deg / 1_rad, 25_deg / 1_rad, 90_deg / 1_rad});

//   // Hsu::Hand::Angles open_angles(176_deg, 176_deg, 176_deg, 176_deg, 70_deg, 120_deg);
//   // Hsu::Hand::Angles close_angles(20_deg, 20_deg, 20_deg, 20_deg, 50_deg, 90_deg);

//   // left_hand->set_angles(close_angles);
//   // right_hand->set_angles(close_angles);

//   // sleep_s(2);

//   // left_hand->set_angles(open_angles);
//   // right_hand->set_angles(open_angles);

//   server->start_server();

//   // std::thread left_cd(collision_detection_thread, left_arm, rm_position_t{0.6, 0.159, -0.2}, collision_detected);
//   // std::thread right_cd(collision_detection_thread, right_arm, rm_position_t{0.6, -0.159, -0.2},
//   collision_detected);
// #if defined(HSU_FRAME_VISUAL)
//   try {
//     auto& sc = Hsu::Frame3DScene::instance();

//     sc.begin();

//     sc.start();

//     std::this_thread::sleep_for(std::chrono::seconds(1));

//     Hsu::Types::RotationM R;
//     Hsu::Types::TranslationM T;
//     double d = std::sqrt(2) / 2;

//     R.value.row(0) << +0, +1, +0;
//     R.value.row(1) << +d, +0, +d;
//     R.value.row(2) << +d, +0, -d;
//     T.value << 0, 0.209, 0;

//     left_arm->set_base_offset({R, T});

//     R.value.row(0) << +0, -1, +0;
//     R.value.row(1) << -d, +0, -d;
//     R.value.row(2) << +d, +0, -d;
//     T.value << 0, -0.209, 0;

//     right_arm->set_base_offset({R, T});

//     WHILE_RUNNING {
//       sc.set_arm_l_data(left_arm->get_frames_data());
//       sc.set_arm_r_data(right_arm->get_frames_data());

//       sleep_ms(100);
//     }

//     sc.stop();
//   } catch (const pybind11::error_already_set& e) {
//     std::cerr << "Main Error: " << e.what() << std::endl;
//   }
// #else
//   WHILE_RUNNING { sleep_ms(100); }
// #endif
//   INFO("主程序已停止");

//   server->stop_server();

//   // // left_cd.join();
//   // // right_cd.join();

//   return 0;
// }

#include <read_config.h>

int main() {
  // ---- 初始化 ----
  std::signal(SIGINT, signalHandler);

  set_up_main_logger();
  // ---- 全局对象 ----

  std::shared_ptr<Hsu::Arm> left_arm, right_arm;

  std::shared_ptr<Hsu::TCPConnection> tcp_server;

  std::shared_ptr<Hsu::ModbusTCP> left_hand_tcp, right_hand_tcp;
  std::shared_ptr<Hsu::Hand> left_hand, right_hand;

  // ---- 全局对象初始化 ----

  try {
    auto config = read_config("config.yaml");

    INFO("==== 连接左臂 ====");
    {
      auto const& [ip, port] = parse_ip_port(config["Address"]["ArmL"]);
      left_arm = std::make_shared<Hsu::Arm>(ip, port);
    }

    INFO("==== 连接右臂 ====");
    {
      auto const& [ip, port] = parse_ip_port(config["Address"]["ArmR"]);
      right_arm = std::make_shared<Hsu::Arm>(ip, port);
    }

    if (config["Actor"]["ArmL"] == "Hand") {
      if (config["Address"]["HandL"] == "485") {
        // TODO: 实现485灵巧手
        throw std::runtime_error("暂时不支持使用485控制灵巧手");
      } else {
        auto const& [ip, port] = parse_ip_port(config["Address"]["HandL"]);
        left_hand_tcp = std::make_shared<Hsu::ModbusTCP>(ip, port);
        left_hand = std::make_shared<Hsu::Hand>(left_hand_tcp->produce_modbus_actor());
      }
    } else {
      // TODO: 实现夹爪
      throw std::runtime_error("暂时不支持使用夹爪");
    }

    if (config["Actor"]["ArmR"] == "Hand") {
      if (config["Address"]["HandR"] == "485") {
        // TODO: 实现485灵巧手
        throw std::runtime_error("暂时不支持使用485控制灵巧手");
      } else {
        auto const& [ip, port] = parse_ip_port(config["Address"]["HandR"]);
        right_hand_tcp = std::make_shared<Hsu::ModbusTCP>(ip, port);
        right_hand = std::make_shared<Hsu::Hand>(right_hand_tcp->produce_modbus_actor());
      }
    } else {
      // TODO: 实现夹爪
      throw std::runtime_error("暂时不支持使用夹爪");
    }

    {
      auto const& [ip, port] = parse_ip_port(config["Address"]["Server"]);
      tcp_server = std::make_shared<Hsu::TCPConnection>(ip, port);
    }

    {
      Hsu::Types::RotationM R;
      Hsu::Types::TranslationM T;
      double d = std::sqrt(2) / 2;

      R.value.row(0) << +0, +1, +0;
      R.value.row(1) << +d, +0, +d;
      R.value.row(2) << +d, +0, -d;
      T.value << 0, 0.211, 0;

      left_arm->set_base_offset({R, T});

      R.value.row(0) << +0, -1, +0;
      R.value.row(1) << -d, +0, -d;
      R.value.row(2) << +d, +0, -d;
      T.value << 0, -0.211, 0;

      right_arm->set_base_offset({R, T});
    }

    {
      auto const& [x, y, z] = config["Actor"]["OffsetL"]["Position"].get<std::array<double, 3>>();
      auto const& [rx, ry, rz] = config["Actor"]["OffsetL"]["Euler"].get<std::array<double, 3>>();

      Hsu::Types::RotationM R;
      R.value = Hsu::get_Rz(rz * 1_rad) * Hsu::get_Ry(ry * 1_rad) * Hsu::get_Rx(rx * 1_rad);
      Hsu::Types::TranslationM T;
      T.value << x, y, z;

      left_arm->set_actor_offset({R, T});
    }

    {
      auto const& [x, y, z] = config["Actor"]["OffsetR"]["Position"].get<std::array<double, 3>>();
      auto const& [rx, ry, rz] = config["Actor"]["OffsetR"]["Euler"].get<std::array<double, 3>>();

      Hsu::Types::RotationM R;
      R.value = Hsu::get_Rz(rz * 1_rad) * Hsu::get_Ry(ry * 1_rad) * Hsu::get_Rx(rx * 1_rad);
      Hsu::Types::TranslationM T;
      T.value << x, y, z;

      right_arm->set_actor_offset({R, T});
    }

  } catch (std::exception const& e) {
    ERROR(e.what());
  }

  try {
    right_arm->set_mode(1);
    left_arm->set_mode(1);

    Hsu::Hand::Angles open_angles(176_deg, 176_deg, 176_deg, 176_deg, 70_deg, 120_deg);
    Hsu::Hand::Angles close_angles(20_deg, 20_deg, 20_deg, 20_deg, 50_deg, 90_deg);

    left_hand->clear_err();
    right_hand->clear_err();

    auto angles = right_hand->read_angles();

    WARN("{} {} {} {} {} {}", angles.index(), angles.little(), angles.middle(), angles.ring(), angles.thumb(),
         angles.thumb_r());

    left_hand->set_angles(close_angles);
    right_hand->set_angles(close_angles);

    angles = right_hand->read_angles();

    sleep_s(2);

    left_hand->set_angles(open_angles);
    right_hand->set_angles(open_angles);

    tcp_server->start_server();

  } catch (std::exception const& e) {
    ERROR(e.what());
  }

#if defined(HSU_FRAME_VISUAL)
  auto x = std::thread([&]() {
    try {
      auto& sc = Hsu::Frame3DScene::instance();

      sc.begin();

      sc.start();

      std::this_thread::sleep_for(std::chrono::seconds(1));

      WHILE_RUNNING {
        sc.set_arm_l_data(left_arm->get_frames_data());
        sc.set_arm_r_data(right_arm->get_frames_data());

        sleep_ms(30);
      }

      sc.stop();
    } catch (const pybind11::error_already_set& e) {
      std::cerr << "Main Error: " << e.what() << std::endl;
    }
  });

  sleep_s(4);

  INFO("可视化已启动");
#endif

  {
    Hsu::Arm::Posture pose(0.35_m, 0_m, 0.2_m, 0_deg, 0_deg, 0_deg);
    left_arm->move(pose, 2, "Wrist", "Base");

    right_arm->move(pose, 2, "Wrist", "Base");
  }

  {
    auto&& pose = left_arm->read_posture("Actor", "World");
    auto const& [x, y, z] = pose.Position;
    auto const& [rx, ry, rz] = pose.Euler;
    INFO("Actor in World: {}, {}, {}, {}, {}, {}", x(), y(), z(), rx(), ry(), rz());
  }

  {
    auto&& pose = left_arm->read_posture("Wrist", "World");
    auto const& [x, y, z] = pose.Position;
    auto const& [rx, ry, rz] = pose.Euler;
    INFO("Wrist in World: {}, {}, {}, {}, {}, {}", x(), y(), z(), rx(), ry(), rz());
  }

  WHILE_RUNNING { sleep_ms(100); }

  sleep_s(1);

  INFO("主程序已停止");

  return 0;
}

void set_up_main_logger() {
  auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>("./log/main.log", true);
  auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();

  console_sink->set_pattern("[%H:%M:%S.%e] [thread %t] [%^%l%$] %s:%# %! %v");
  file_sink->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [thread %t] [%^%l%$] %s:%# %! %v");

  std::vector<spdlog::sink_ptr> sinks{file_sink, console_sink};
  auto logger = std::make_shared<spdlog::logger>("main_logger", sinks.begin(), sinks.end());

  spdlog::set_default_logger(logger);

  spdlog::set_level(spdlog::level::debug);
  spdlog::flush_on(spdlog::level::debug);

  INFO("==== Main ====");
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

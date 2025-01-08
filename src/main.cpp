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
#include <cstdlib>
#include <exception>
#include <functional>
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

int main() {
  try {
    auto& sc = Hsu::Frame3DScene::instance();

    sc.begin();

    auto world = Hsu::Frame::WORLD_FRAME();

    Hsu::Types::RotationM ArmLR;
    Hsu::Types::TranslationM ArmLT;
    ArmLT.value << 0, 0.209, 0;
    Hsu::Types::HomogeneousM ArmLH(ArmLR, ArmLT);

    auto ArmL = world->define_frame("ArmL", ArmLH);

    Hsu::Types::RotationM ArmRR;
    Hsu::Types::TranslationM ArmRT;
    ArmRT.value << 0, -0.209, 0;
    Hsu::Types::HomogeneousM ArmRH(ArmRR, ArmRT);

    auto ArmR = world->define_frame("ArmR", ArmRH);

    sc.start();

    std::this_thread::sleep_for(std::chrono::seconds(1));

    sc.add_obj(world).add_obj(ArmL).add_obj(ArmR);

    std::this_thread::sleep_for(std::chrono::seconds(1));

    for (int i = 0; i < 50; i++) {
      ArmL->transform(ArmLT, ArmL);
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }

    for (int i = 0; i < 50; i++) {
      ArmR->transform(ArmRT, ArmL);
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }

    sc.stop();

  } catch (const pybind11::error_already_set& e) {
    std::cerr << "Main Error: " << e.what() << std::endl;
    return 1;
  }

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

  left_arm->set_hand_len(0.0);
  right_arm->set_hand_len(0.0);

  left_arm->set_offset(rm_position_t{0, -0.209, 0});
  right_arm->set_offset(rm_position_t{0, 0.209, 0});

  right_arm->set_mode(1);
  left_arm->set_mode(1);

  limited_move_jp(left_arm, rm_position_t{0.7, 0.40, -0.15},
                  rm_euler_t{-90_deg / 1_rad, 25_deg / 1_rad, -90_deg / 1_rad});
  limited_move_jp(right_arm, rm_position_t{0.7, -0.40, -0.15},
                  rm_euler_t{90_deg / 1_rad, 25_deg / 1_rad, 90_deg / 1_rad});

  Hsu::Hand::Angles open_angles(176_deg, 176_deg, 176_deg, 176_deg, 70_deg, 120_deg);
  Hsu::Hand::Angles close_angles(20_deg, 20_deg, 20_deg, 20_deg, 50_deg, 90_deg);

  left_hand->set_angles(close_angles);
  right_hand->set_angles(close_angles);

  sleep_s(2);

  left_hand->set_angles(open_angles);
  right_hand->set_angles(open_angles);

  server->start_server();

  // std::thread left_cd(collision_detection_thread, left_arm, rm_position_t{0.6, 0.159, -0.2}, collision_detected);
  // std::thread right_cd(collision_detection_thread, right_arm, rm_position_t{0.6, -0.159, -0.2}, collision_detected);

  WHILE_RUNNING { sleep_ms(500); }

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

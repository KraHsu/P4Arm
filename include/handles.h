#pragma once

#include <Hsu/arm.h>
#include <Hsu/hand.h>
#include <Hsu/tcp.h>

#define L_ARM 0.14
#define L_HAND 0.14

Hsu::TCPConnection::Response handle_arm(std::shared_ptr<Hsu::Arm> arm, int code, nlohmann::json const& payload);
Hsu::TCPConnection::Response handle_hand(std::shared_ptr<Hsu::Hand> hand, int code, nlohmann::json const& payload);

bool collision_detected(rm_position_t position, rm_euler_t euler);
bool limited_move_jp(std::shared_ptr<Hsu::Arm> arm, rm_position_t position, rm_euler_t posture);
nlohmann::json read_tactile_data(std::shared_ptr<Hsu::Hand> hand, const nlohmann::json& config);
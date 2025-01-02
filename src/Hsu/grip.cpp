#include <Hsu/grip.h>
#include <nlohmann/json.hpp>
#include <iostream>
#include <unistd.h>

using json = nlohmann::json;

RM_Grip ele_grip;

void RM_Grip::RM_GripInit() {
  rm_set_modbus_mode(handle, 1, 115200, 2);
  rm_peripheral_read_write_params_t params_coils;
  // 初始化夹爪，最大值、最小值运动复位
  int data = 0xA5;
  params_coils.port = 1;
  params_coils.address = 0x0100;
  params_coils.device = 1;
  int ret = rm_write_single_register(handle, params_coils, data);
  std::cout << "Grip initial cmd send state: " << ret << std::endl;
  sleep(6);

  params_coils.port = 1;
  params_coils.address = 0x0200;
  params_coils.device = 1;
  ret = rm_read_holding_registers(handle, params_coils, &data);
  sleep(1);
  // 判断夹爪是否初始化完毕
  if (data == 1) {
    RM_SetGripForce(20);
    sleep(1);
    RM_SetGripSpeed(20);
    sleep(1);
    std::cout << "Grip initial succeed" << std::endl;
  } else
    std::cout << "Grip initial failed" << std::endl;
}

void RM_Grip::RM_SetGripForce(uint16_t force) {
  rm_peripheral_read_write_params_t params_coils;
  int data = force;
  params_coils.port = 1;
  params_coils.address = 0x0101;
  params_coils.device = 1;
  int ret = rm_write_single_register(handle, params_coils, data);
  std::cout << "write force result : " << ret << std::endl;
}

void RM_Grip::RM_SetGripSpeed(uint16_t speed) {
  rm_peripheral_read_write_params_t params_coils;
  int data = speed;
  params_coils.port = 1;
  params_coils.address = 0x0104;
  params_coils.device = 1;
  int ret = rm_write_single_register(handle, params_coils, data);
  std::cout << "write speed result : " << ret << std::endl;
}

void RM_Grip::RM_SetGripPose(float pose, json& response) {
  rm_peripheral_read_write_params_t params_coils;
  int data = pose;
  params_coils.port = 1;
  params_coils.address = 0x0103;
  params_coils.device = 1;
  int ret = rm_write_single_register(handle, params_coils, data);

  uint16_t Fdb_Pose = RM_GetGripPose();
  uint16_t Fdb_Force = RM_GetGripForce();

  std::cout << "write pose result : " << ret << std::endl;

  if (ret == 0) {
    response = {{"cmd", "grip_res"}, {"code"}, {"payload", {"holding", Fdb_Force}}};
    response["code"] = 0;
    response["payload"]["holding"] = Fdb_Pose;
  } else {
    response = {{"cmd", "grip_res"}, {"code"}, {"payload", {{"message", "grip error"}}}};
    response["code"] = 100;
  }
}

uint16_t RM_Grip::RM_GetGripForce() {
  rm_peripheral_read_write_params_t params_coils;
  int data;
  params_coils.port = 1;
  params_coils.address = 0x0201;
  params_coils.device = 1;
  int ret = rm_read_holding_registers(handle, params_coils, &data);
  std::cout << "current force : " << data << std::endl;
  Fdb_Pose = data;
  return (uint16_t)data;
}

uint16_t RM_Grip::RM_GetGripPose() {
  rm_peripheral_read_write_params_t params_coils;
  int data;
  params_coils.port = 1;
  params_coils.address = 0x0202;
  params_coils.device = 1;
  int ret = rm_read_holding_registers(handle, params_coils, &data);
  printf("current pose : %d\n", data);
  Fdb_Pose = data;
  return (uint16_t)data;
}

Grip_StateEnum RM_Grip::RM_GetGripCaughtState() {
  rm_peripheral_read_write_params_t params_coils;
  int data;
  params_coils.port = 1;
  params_coils.address = 0x0201;
  params_coils.device = 1;
  int ret = rm_read_holding_registers(handle, params_coils, &data);
  printf("current state : %d\n", data);
  return (Grip_StateEnum)data;
}
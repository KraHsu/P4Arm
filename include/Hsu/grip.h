#pragma once

#include <RMArm/rm_define.h>
#include <RMArm/rm_service.h>
#include <nlohmann/json.hpp>
#include <stdio.h>
#include <unistd.h>

typedef enum { Grip_Running = 0, Grip_Arrived = 1, Grip_Caught = 2, Grip_Lost = 3 } Grip_StateEnum;

class RM_Grip {
  using json = nlohmann::json;

 public:
  rm_robot_handle* handle;

  void RM_GripInit();
  void RM_SetGripForce(uint16_t force);             // 20-100 percent
  void RM_SetGripSpeed(uint16_t speed);             // 1-100 percent
  void RM_SetGripPose(float pose, json& response);  // per thousand
  uint16_t RM_GetGripForce();
  Grip_StateEnum RM_GetGripCaughtState();
  uint16_t RM_GetGripPose();

  uint16_t Fdb_Pose;
  uint16_t Fdb_Force;
};

extern RM_Grip ele_grip;
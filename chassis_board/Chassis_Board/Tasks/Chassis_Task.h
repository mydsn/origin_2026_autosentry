#ifndef _CHASSIS_TASK
#define _CHASSIS_TASK

#include "struct_typedef.h"
#include "pid.h"

typedef struct // 底盘控制参数结构体
{
  fp32 vx;
  fp32 vy;
  fp32 wz;

  fp32 chassis_follow_gimbal_angle;
  fp32 chassis_power_limit;
  fp32 init_chassis_power;
  bool_t chassis_follow_gimbal_zerochange;

  pid_type_def chassis_follow_gimbal_pid;
} chassis_control_t;

extern chassis_control_t chassis_control;

void Chassis_Task(void const *argument);

#endif

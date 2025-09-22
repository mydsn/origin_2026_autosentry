/***************************************************************************************************
 * @file: motor.c
 * @author: Shiki
 * @date: 2025.9.17
 * @brief: 哨兵电机库（上下C板公用）
 * @attention:
 ***************************************************************************************************/

#ifndef MOTOR_H
#define MOTOR_H

#include "struct_typedef.h"
#include "pid.h"

typedef struct // 大疆电机反馈值结构体
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
    fp32 code;

} motor_measure_t; 

typedef struct // 底盘轮电机结构体
{
    int16_t speed_now;  //rpm
    fp32 speed_set;
    int16_t give_current;

    pid_type_def speed_pid;
} chassis_wheel_motor_t;

typedef struct // 底盘舵电机结构体
{
    int16_t speed_now;  //rpm
    fp32 speed_set;
    uint16_t ENC_angle_now;
    int32_t ENC_angle_set;
    int16_t give_current;

    pid_type_def speed_pid;
    pid_type_def angle_pid;
} chassis_steer_motor_t;

typedef struct
{
    fp32 INS_speed_now;
    fp32 INS_speed_last;
    fp32 INS_speed_set;
    fp32 INS_speed_set_last;
    fp32 INS_angle_now;
    fp32 INS_angle_set;
    uint16_t ENC_angle_now;
    int16_t ENC_speed_now;
    int16_t give_current;

    pid_type_def speed_pid;
    pid_type_def angle_pid;
    pid_type_def auto_aim_pid;
} gimbal_motor_t; // 云台yaw轴6020结构体


#endif
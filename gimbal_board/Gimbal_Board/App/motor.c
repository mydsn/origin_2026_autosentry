/***************************************************************************************************
 * @file: motor.c
 * @author: Shiki
 * @date: 2025.9.17
 * @brief: 哨兵电机库（上下C板公用）
 * @attention:
 ***************************************************************************************************/

 #include "moror.h"

#define get_motor_measure(ptr, data)                                   \
    {                                                                  \
        (ptr)->last_ecd = (ptr)->ecd;                                  \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);           \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);     \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]); \
        (ptr)->temperate = (data)[6];                                  \
    }

motor_measure_t motor_measure_steer[4];  //底盘舵电机6020
motor_measure_t motor_measure_wheel[4];  //底盘轮电机3508
motor_measure_t motor_measure_small_yaw;  //小yaw电机6020
motor_measure_t motor_measure_pitch;      //pitch 6020
motor_measure_t motor_measure_shoot[2];    //摩擦轮电机3508

chassis_steer_motor_t chassis_steer_motor[4];
chassis_wheel_motor_t chassis_wheel_motor[4];
gimbal_motor_t gimbal_motor_small_yaw;
gimbal_motor_t gimbal_motor_pitch;
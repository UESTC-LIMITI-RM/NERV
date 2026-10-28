/**
  * @file    nerv_motor_driver_dji.h
  * @brief   DJI CAN 电调驱动声明.
  * @version 1.0.0
  * @date    2026-10-25
  * @copyright Copyright (c) 2026 NERV Contributors
  *
  * SPDX-License-Identifier: MIT
  */

#ifndef __NERV_MOTOR_DRIVER_DJI_H
#define __NERV_MOTOR_DRIVER_DJI_H

#include "nerv_motor_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

#if (NERV_USE_MOTOR_DJI == NERV_ENABLE)

/**
  * @brief DJI M3508 电机 (C620 电调) 驱动.
  * @note  由 module/motor/driver/DJI/nerv_motor_driver_dji.c 提供.
  */
extern const NERV_Motor_DriverTypeDef NERV_Motor_Driver_Dji3508;

/**
  * @brief DJI M2006 电机 (C610 电调) 驱动.
  * @note  由 module/motor/driver/DJI/nerv_motor_driver_dji.c 提供.
  */
extern const NERV_Motor_DriverTypeDef NERV_Motor_Driver_Dji2006;

/**
  * @brief DJI GM6020 电机电流控制模式驱动.
  * @note  由 module/motor/driver/DJI/nerv_motor_driver_dji.c 提供.
  * @warning 反馈帧标识符区间 0x205 到 0x20B 与 C620, C610 电调 id 5 到 8 的反馈区间
  *           0x205 到 0x208 重叠, 同总线混编时会互相误收, 使用时必须错开设备 id
  *           或避开混编.
  */
extern const NERV_Motor_DriverTypeDef NERV_Motor_Driver_Dji6020Current;

/**
  * @brief DJI GM6020 电机电压控制模式驱动.
  * @note  由 module/motor/driver/DJI/nerv_motor_driver_dji.c 提供.
  * @warning 电压模式控制帧标识符 0x1FF 与 C620 电调高组指令标识符相同, 混编
  *           总线上会冲突, 使用时必须错开设备 id 或避开电压模式.
  * @warning 反馈帧标识符区间 0x205 到 0x20B 与 C620, C610 电调 id 5 到 8 的反馈区间
  *           0x205 到 0x208 重叠, 同总线混编时会互相误收, 使用时必须错开设备 id
  *           或避开混编.
  * @warning 本型号的控制量单位是协议给定值 (满量程 25000, 无量纲), 不是电流. 电机实例
  *           配置的 control_limit 按同单位解释, 按电流安培数配置会得到近乎为零的输出.
  */
extern const NERV_Motor_DriverTypeDef NERV_Motor_Driver_Dji6020Voltage;

#endif /* NERV_USE_MOTOR_DJI */

#ifdef __cplusplus
}
#endif

#endif /* __NERV_MOTOR_DRIVER_DJI_H */

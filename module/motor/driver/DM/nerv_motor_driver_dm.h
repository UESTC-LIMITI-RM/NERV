/**
  * @file    nerv_motor_driver_dm.h
  * @brief   达妙 DM-J4310-2EC 电机驱动声明.
  * @version 1.0.0
  * @date    2026-10-25
  * @copyright Copyright (c) 2026 NERV Contributors
  *
  * SPDX-License-Identifier: MIT
  */

#ifndef __NERV_MOTOR_DRIVER_DM_H
#define __NERV_MOTOR_DRIVER_DM_H

#include "nerv_motor_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

#if (NERV_USE_MOTOR_DM == NERV_ENABLE)

/**
  * @brief 达妙 DM-J4310-2EC MIT 模式驱动.
  * @note  由 module/motor/driver/DM/nerv_motor_driver_dm.c 提供.
  * @note  控制帧标识符为 0x000 加设备 id, 反馈帧标识符由 Master ID 配置.
  * @note  控制量为 NERV_MOTOR_CONTROL_MIT, 位置/速度/刚度/阻尼/前馈转矩位打包进 8 字节.
  */
extern const NERV_Motor_DriverTypeDef NERV_Motor_Driver_DmMit;

/**
  * @brief 达妙 DM-J4310-2EC 速度模式驱动.
  * @note  由 module/motor/driver/DM/nerv_motor_driver_dm.c 提供.
  * @note  控制帧标识符为 0x200 加设备 id, 控制量为 NERV_MOTOR_CONTROL_SCALAR, 单位 rad/s.
  */
extern const NERV_Motor_DriverTypeDef NERV_Motor_Driver_DmVelocity;

/**
  * @brief 达妙 DM-J4310-2EC 位置速度模式驱动.
  * @note  由 module/motor/driver/DM/nerv_motor_driver_dm.c 提供.
  * @note  控制帧标识符为 0x100 加设备 id, 控制量为 NERV_MOTOR_CONTROL_POSITION_VELOCITY.
  */
extern const NERV_Motor_DriverTypeDef NERV_Motor_Driver_DmPositionVelocity;

#endif /* NERV_USE_MOTOR_DM */

#ifdef __cplusplus
}
#endif

#endif /* __NERV_MOTOR_DRIVER_DM_H */

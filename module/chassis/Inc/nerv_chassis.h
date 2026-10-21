/**
  * @file    nerv_chassis.h
  * @brief   NERV 底盘模块公开接口.
  * @version 1.0.0
  * @date    2026-10-21
  * @copyright Copyright (c) 2026 NERV Contributors
  *
  * SPDX-License-Identifier: MIT
  */

#ifndef NERV_CHASSIS_H
#define NERV_CHASSIS_H

#include "nerv_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief  机体系速度.
  * @note   分量在机体坐标系下表达, 用于底盘速度指令与反馈.
  * @note   机体坐标系为右手系, x 轴指向机器人正前方, y 轴指向左方, z 轴指向上方, 与 REP-103 一致.
  * @note   遥控器给出的速度指令在云台系, 调用方必须先转到机体系再传入. 绕 z 轴的角速度不随参考系改变.
  */
typedef struct NERV_Chassis_Velocity
{
    float vx_mps;   /**< x 方向线速度, 单位 m/s, 机体前方为正 */
    float vy_mps;   /**< y 方向线速度, 单位 m/s, 机体左方为正 */
    float wz_radps; /**< 绕 z 轴角速度, 单位 rad/s, 逆时针为正 */
} NERV_Chassis_VelocityTypeDef;

/**
  * @brief  底盘里程计结果.
  * @note   位置在参考系中累积, 只有相对起点的意义. 姿态与速度在机体坐标系中表达.
  */
typedef struct NERV_Chassis_Odometry
{
    float x_m;                             /**< 位置 x, 单位 m, 参考系中向前为正 */
    float y_m;                             /**< 位置 y, 单位 m, 参考系中向左为正 */
    NERV_AttitudeTypeDef attitude;         /**< 姿态, 相对参考系 */
    NERV_Chassis_VelocityTypeDef velocity; /**< 速度, 在机体坐标系中 */
} NERV_Chassis_OdometryTypeDef;

#ifdef __cplusplus
}
#endif

#endif /* NERV_CHASSIS_H */

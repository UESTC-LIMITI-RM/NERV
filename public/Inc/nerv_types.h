/**
  * @file    nerv_types.h
  * @brief   NERV 通用数据类型定义.
  * @version 1.0.0
  * @date    2026-10-21
  * @copyright Copyright (c) 2026 NERV Contributors
  *
  * SPDX-License-Identifier: MIT
  */

#ifndef NERV_TYPES_H
#define NERV_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief  模块姿态.
  * @note   三个角度描述机体相对参考系的朝向.
  * @note   机体坐标系为右手系, x 轴指向机体正前方, y 轴指向左方, z 轴指向上方, 与 REP-103 一致.
  * @note   角度位于 [-π, π). 跨 ±π 的归一化必须调用 `NERV_Math_WrapPi`.
  * @note   底盘用它表达航向, 装有 IMU 的模块用它表达自身姿态.
  */
typedef struct NERV_Attitude
{
    float yaw_rad;   /**< 偏航角, 绕 z 轴, 逆时针为正, 范围 [-π, π) */
    float pitch_rad; /**< 俯仰角, 绕 y 轴, 抬头为正, 范围 [-π, π) */
    float roll_rad;  /**< 横滚角, 绕 x 轴, 右侧下沉为正, 范围 [-π, π) */
} NERV_AttitudeTypeDef;

#ifdef __cplusplus
}
#endif

#endif /* NERV_TYPES_H */

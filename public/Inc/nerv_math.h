/**
  * @file    nerv_math.h
  * @brief   NERV 数学工具.
  * @version 1.0.0
  * @date    2026-10-21
  * @copyright Copyright (c) 2026 NERV Contributors
  *
  * SPDX-License-Identifier: MIT
  */

#ifndef NERV_MATH_H
#define NERV_MATH_H

#include <stdbool.h>

#include "nerv_status.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief 圆周率, 单位 rad.
  */
#define NERV_MATH_PI 3.14159265358979323846f

/**
  * @brief 2π, 单位 rad.
  */
#define NERV_MATH_TWO_PI 6.28318530717958647692f

/**
  * @brief  把数值规整到半开区间 [min_value, max_value) 内, 超界则周期性环绕.
  * @note   ISR-safe. 纯函数, 不读写共享状态.
  */
float NERV_Math_Wrap(float value, float min_value, float max_value);

/**
  * @brief  判断两浮点数在给定容差内是否足够接近.
  * @note   ISR-safe. 纯函数, 通过输出参数返回结果, 不读写共享状态.
  */
NERV_StatusTypeDef NERV_Math_IsClose(float a, float b, float tolerance, bool *is_close);

/**
  * @brief  把数值限幅到 [min_value, max_value] 区间.
  * @note   ISR-safe. 纯函数, 不读写共享状态.
  */
float NERV_Math_Clamp(float value, float min_value, float max_value);

/**
  * @brief  死区处理, 输入绝对值小于阈值时输出 0.
  * @note   ISR-safe. 纯函数, 不读写共享状态.
  */
float NERV_Math_Deadband(float value, float threshold);

/**
  * @brief  绕 Z 轴旋转二维向量 (vx, vy), 原地修改.
  * @note   ISR-safe. 纯函数, 只修改调用方提供的输出参数, 不读写共享状态.
  */
NERV_StatusTypeDef NERV_Math_RotateZ(float *vx, float *vy, float angle_rad);

/**
  * @brief  解三阶线性方程组, Cramer 法则.
  * @note   ISR-safe. 纯函数, 只写入 solution, 不读写共享状态.
  */
NERV_StatusTypeDef NERV_Math_SolveLinear3(const float *matrix, const float *rhs, float *solution);

#ifdef __cplusplus
}
#endif

#endif /* NERV_MATH_H */

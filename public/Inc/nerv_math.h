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

/** @brief 每秒的毫秒数, 用于把毫秒间隔换算为秒. */
#define NERV_MATH_MS_PER_S 1000.0f

float NERV_Math_Wrap(float value, float min_value, float max_value);

NERV_StatusTypeDef NERV_Math_IsClose(float a, float b, float tolerance, bool *is_close);

float NERV_Math_Clamp(float value, float min_value, float max_value);

float NERV_Math_Deadband(float value, float threshold);

NERV_StatusTypeDef NERV_Math_RotateZ(float *vx, float *vy, float angle_rad);

NERV_StatusTypeDef NERV_Math_SolveLinear3(const float *matrix, const float *rhs, float *solution);

#ifdef __cplusplus
}
#endif

#endif /* NERV_MATH_H */

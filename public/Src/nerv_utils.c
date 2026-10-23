/**
  * @file    nerv_utils.c
  * @brief   NERV 通用工具实现.
  * @version 1.0.0
  * @date    2026-10-22
  * @copyright Copyright (c) 2026 NERV Contributors
  *
  * SPDX-License-Identifier: MIT
  */

#include "nerv_utils.h"
#include "nerv_math.h"

#include <float.h>
#include <math.h>

/** @cond INTERNAL */

#define __NERV_UTILS_SECONDS_PER_MINUTE 60.0f
#define __NERV_UTILS_RADPS_PER_RPM      (NERV_MATH_TWO_PI / __NERV_UTILS_SECONDS_PER_MINUTE)
#define __NERV_UTILS_RPM_PER_RADPS      (__NERV_UTILS_SECONDS_PER_MINUTE / NERV_MATH_TWO_PI)
#define __NERV_UTILS_MAX_RADPS          (FLT_MAX / __NERV_UTILS_RPM_PER_RADPS)

/** @endcond */

/**
  * @brief  把转速从 rpm 转换为 rad/s.
  * @param  rpm 转速, 单位 rpm. 正负号表示旋转方向.
  * @return 转速, 单位 rad/s. 非有限输入原样返回.
  * @note   1 rpm 等于 2π/60 rad/s.
  * @note   极小输入可能下溢为 0.
  * @note   极大输入可能上溢为无穷大.
  * @note   ISR-safe.
  */
float NERV_Utils_RpmToRadps(float rpm)
{
    if (!isfinite(rpm))
    {
        return rpm;
    }

    return rpm * __NERV_UTILS_RADPS_PER_RPM;
}

/**
  * @brief  把角速度从 rad/s 转换为 rpm.
  * @param  radps 角速度, 单位 rad/s. 正负号表示旋转方向.
  * @return 转速, 单位 rpm. 非有限输入原样返回.
  *         有限输入的结果超出 float 范围时返回带符号的无穷大.
  * @note   1 rad/s 等于 60/(2π) rpm.
  * @note   极大输入可能溢出, 此时返回带符号的无穷大.
  * @note   ISR-safe.
  */
float NERV_Utils_RadpsToRpm(float radps)
{
    if (!isfinite(radps))
    {
        return radps;
    }

    if (fabsf(radps) >= __NERV_UTILS_MAX_RADPS)
    {
        return copysignf(INFINITY, radps);
    }

    return radps * __NERV_UTILS_RPM_PER_RADPS;
}

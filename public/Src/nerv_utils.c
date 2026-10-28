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
/**
  * @brief  转速单位转换, 转每分转换为弧度每秒.
  * @note   ISR-safe. 纯函数, 不读写共享状态.
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
  *         有限输入的结果超出 float 范围时饱和到带符号的 FLT_MAX.
  * @note   1 rad/s 等于 60/(2π) rpm.
  * @note   极大输入可能溢出, 此时饱和到带符号的 FLT_MAX, 不返回无穷大, 以兼容
  *         Keil AC6 的 --fpmode=fast (该模式禁用 Inf/NaN).
  * @note   ISR-safe.
  */
/**
  * @brief  转速单位转换, 弧度每秒转换为转每分.
  * @note   ISR-safe. 纯函数, 不读写共享状态.
  */
float NERV_Utils_RadpsToRpm(float radps)
{
    if (!isfinite(radps))
    {
        return radps;
    }

    if (fabsf(radps) >= __NERV_UTILS_MAX_RADPS)
    {
        return copysignf(FLT_MAX, radps);
    }

    return radps * __NERV_UTILS_RPM_PER_RADPS;
}

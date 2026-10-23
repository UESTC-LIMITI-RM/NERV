/**
  * @file    nerv_utils.h
  * @brief   NERV 通用工具与单位转换.
  * @version 1.0.0
  * @date    2026-10-21
  * @copyright Copyright (c) 2026 NERV Contributors
  *
  * SPDX-License-Identifier: MIT
  */

#ifndef NERV_UTILS_H
#define NERV_UTILS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief  数组元素个数.
  * @param  _array 数组名. 必须指向真实数组, 传入指针时结果无意义.
  * @return 数组元素个数.
  * @note   展开体不求值实参, 传入空指针不会解引用, 但结果没有意义.
  * @warning 形参在展开体中出现两次. 传入带副作用的实参会产生多次求值.
  */
#define NERV_UTILS_ARRAY_SIZE(_array) (sizeof(_array) / sizeof((_array)[0]))

/**
  * @brief  毫秒超时值.
  * @param  _ms 超时时长, 单位 ms. 不足 1 ms 的部分截断.
  * @return 无符号整数表示的毫秒超时值.
  */
#define NERV_UTILS_TIMEOUT_MS(_ms) ((uint32_t)(_ms))

/**
  * @brief  秒超时值转换为毫秒.
  * @param  _s 超时时长, 单位 s. 不足 1 ms 的部分截断.
  * @return 无符号整数表示的毫秒超时值.
  */
#define NERV_UTILS_TIMEOUT_S_TO_MS(_s) ((uint32_t)((_s) * 1000U))

float NERV_Utils_RpmToRadps(float rpm);

float NERV_Utils_RadpsToRpm(float radps);

#ifdef __cplusplus
}
#endif

#endif /* NERV_UTILS_H */

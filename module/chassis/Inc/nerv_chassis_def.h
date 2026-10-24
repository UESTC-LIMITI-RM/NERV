/**
  * @file    nerv_chassis_def.h
  * @brief   NERV 底盘模块编译期定义.
  * @version 1.0.0
  * @date    2026-10-24
  * @copyright Copyright (c) 2026 NERV Contributors
  *
  * SPDX-License-Identifier: MIT
  */

#ifndef NERV_CHASSIS_DEF_H
#define NERV_CHASSIS_DEF_H

#include "config.h"

#if (NERV_USE_CHASSIS == NERV_ENABLE)

/* ========================================================================== */
/* 以下定义为底盘构型派生宏, 由 NERV_CHASSIS_TYPE 推导, 禁止使用者覆盖.        */
/* 作用域: NERV_CHASSIS_WHEEL_COUNT, 直至本定义区结束.                          */
/* ========================================================================== */

/* 轮组数量按构型推导. 同一次编译只保留选中的一种构型, 轮组数量随之确定.
 * 麦克纳姆轮与全向轮均为四轮布局; 舵轮为四轮, 每轮含转向与驱动两个执行器. */

#if defined(NERV_CHASSIS_WHEEL_COUNT)
#error "NERV_CHASSIS_WHEEL_COUNT is derived and must not be defined by the user."
#endif

#if (NERV_CHASSIS_TYPE == NERV_CHASSIS_TYPE_MECANUM)
#define NERV_CHASSIS_WHEEL_COUNT 4U
#elif (NERV_CHASSIS_TYPE == NERV_CHASSIS_TYPE_OMNI)
#define NERV_CHASSIS_WHEEL_COUNT 4U
#else
#define NERV_CHASSIS_WHEEL_COUNT 4U
#endif

/* ========================================================================== */
/* 底盘构型派生宏定义区结束                                                   */
/* ========================================================================== */

#endif /* NERV_USE_CHASSIS */

#endif /* NERV_CHASSIS_DEF_H */

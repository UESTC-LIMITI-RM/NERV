/**
  * @file    nerv_gimbal_def.h
  * @brief   NERV 云台模块轴索引与派生常量.
  * @version 1.0.0
  * @date    2026-10-31
  * @copyright Copyright (c) 2026 NERV Contributors
  *
  * SPDX-License-Identifier: MIT
  */

#ifndef NERV_GIMBAL_DEF_H
#define NERV_GIMBAL_DEF_H

#include "config.h"

#if (NERV_USE_GIMBAL == NERV_ENABLE)

/* 轴索引. 四轴结构按大 yaw, 小 yaw, pitch, roll 依次串联.
 * 两个 yaw 轴的轴线平行且都竖直, 安装位置不同, 总偏航角等于两者之和.
 * 索引直接用作电机数组下标, 顺序不得改动. */

/** @brief 大 yaw 轴索引, 安装在底盘与云台之间, 轴线竖直. */
#define NERV_GIMBAL_AXIS_YAW_BIG 0U

/** @brief 小 yaw 轴索引, 轴线与大 yaw 平行, 安装位置不同. */
#define NERV_GIMBAL_AXIS_YAW_SMALL 1U

/** @brief pitch 轴索引, 轴线水平, 抬头为正. */
#define NERV_GIMBAL_AXIS_PITCH 2U

/** @brief roll 轴索引, 轴线水平, 右侧下沉为正. */
#define NERV_GIMBAL_AXIS_ROLL 3U

/* 轴数由 config.h 选择, 四轴结构只接受四轴取值. */
#if (NERV_GIMBAL_AXIS_COUNT != NERV_GIMBAL_AXIS_COUNT_4)
#error "NERV_GIMBAL_AXIS_COUNT must be NERV_GIMBAL_AXIS_COUNT_4 for the four-axis gimbal."
#endif

#endif /* NERV_USE_GIMBAL */

#endif /* NERV_GIMBAL_DEF_H */

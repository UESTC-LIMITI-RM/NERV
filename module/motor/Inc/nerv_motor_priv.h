/**
  * @file    nerv_motor_priv.h
  * @brief   NERV 电机模块内部接口.
  * @version 1.0.0
  * @date    2026-10-23
  * @copyright Copyright (c) 2026 NERV Contributors
  *
  * SPDX-License-Identifier: MIT
  */

#ifndef NERV_MOTOR_PRIV_H
#define NERV_MOTOR_PRIV_H

#include <stdint.h>

#include "config.h"

#if (NERV_USE_MOTOR == NERV_ENABLE)

#include "nerv_motor.h"
#include "nerv_status.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/* 以下定义为电机模块内部实现常数, 禁止修改.                                   */
/* 作用域: 所有 __NERV_MOTOR_ 开头的宏, 直至本定义区结束.                      */
/* ========================================================================== */

/**
  * @brief 安装方向取反时指令与反馈的乘数.
  * @note  对应配置项 invert_direction 为 true. 不取反时乘以 1.0f, 该值是第 7.4 节
  *        的字面量例外, 不在本文件定义.
  */
#define __NERV_MOTOR_DIRECTION_SIGN_INVERTED (-1.0f)

/* ========================================================================== */
/* 电机模块内部实现常数定义区结束                                             */
/* ========================================================================== */

/* 内部辅助函数, 只供模块源文件使用, 不进入公开接口. */

static void NERV_Motor_ResetState(NERV_Motor_InstanceTypeDef *self);

static float NERV_Motor_DirectionSign(const NERV_Motor_ConfigTypeDef *config);

static NERV_StatusTypeDef NERV_Motor_ConvertFb(const NERV_Motor_ConfigTypeDef *config,
                                               const NERV_Motor_FbTypeDef *fb,
                                               NERV_Motor_MeasurementTypeDef *measurement);

static float NERV_Motor_EffectiveCommand(const NERV_Motor_InstanceTypeDef *self);

static NERV_StatusTypeDef NERV_Motor_EncodeInto(const NERV_Motor_InstanceTypeDef *self,
                                                uint8_t *frame);

static void NERV_Motor_Supervise(NERV_Motor_InstanceTypeDef *self, uint32_t now_ms);

static bool NERV_Motor_SameGroup(const NERV_Motor_InstanceTypeDef *lhs,
                                 const NERV_Motor_InstanceTypeDef *rhs);

static NERV_StatusTypeDef NERV_Motor_ValidateGroups(NERV_Motor_InstanceTypeDef *const *members,
                                                    uint8_t count);

static NERV_StatusTypeDef NERV_Motor_TransmitGroupFrame(NERV_Motor_InstanceTypeDef *const *members,
                                                        uint8_t count, uint8_t first);

#ifdef __cplusplus
}
#endif

#endif /* NERV_USE_MOTOR */

#endif /* NERV_MOTOR_PRIV_H */

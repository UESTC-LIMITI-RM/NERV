/**
  * @file    nerv_motor_driver.c
  * @brief   NERV 电机驱动层公共实现.
  * @version 1.0.0
  * @date    2026-10-28
  * @copyright Copyright (c) 2026 NERV Contributors
  *
  * SPDX-License-Identifier: MIT
  */

#include <stddef.h>

#include "config.h"
#include "nerv_motor_driver.h"

#if (NERV_USE_MOTOR == NERV_ENABLE)

/**
  * @brief  不支持某项能力时的统一实现.
  * @return NERV_ERROR, 表示该驱动不提供此能力.
  * @note   由驱动在操作表中填入, 保证可选指针永不为空, 核心层可直接调用.
  */
NERV_StatusTypeDef NERV_Motor_Driver_NotSupported(const NERV_FDCAN_InstanceTypeDef *bus,
                                                  const NERV_Motor_ModelTypeDef *model,
                                                  uint8_t device_id)
{
    (void)bus;
    (void)model;
    (void)device_id;

    return NERV_ERROR;
}

/**
  * @brief  不支持模式切换时的统一实现.
  * @return NERV_ERROR, 表示该驱动不提供此能力.
  */
NERV_StatusTypeDef NERV_Motor_Driver_NotSupportedSetMode(const NERV_FDCAN_InstanceTypeDef *bus,
                                                         const NERV_Motor_ModelTypeDef *model,
                                                         uint8_t device_id,
                                                         NERV_Motor_ControlKindTypeDef mode)
{
    (void)bus;
    (void)model;
    (void)device_id;
    (void)mode;

    return NERV_ERROR;
}

#endif /* NERV_USE_MOTOR */

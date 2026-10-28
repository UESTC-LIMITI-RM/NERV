/**
  * @file    nerv_status.h
  * @brief   NERV 基础状态码与错误码定义.
  * @version 1.2.0
  * @date    2026-10-21
  * @copyright Copyright (c) 2026 NERV Contributors
  *
  * SPDX-License-Identifier: MIT
  */

#ifndef NERV_STATUS_H
#define NERV_STATUS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/* 以下定义为 NERV 核心契约, 禁止修改.                                      */
/* 作用域: NERV_StatusTypeDef 与全部 NERV_ERR_ 开头的宏, 直至文件末尾.      */
/* 修改必须同步更新 AGENTS.md 与遥测协议定义.                               */
/* ========================================================================== */

/**
  * @brief  NERV 基础状态码.
  * @note   所有对外函数的默认返回类型, 与平台和 RTOS 无关.
  */
typedef enum
{
    NERV_OK      = 0x00U, /**< 操作成功 */
    NERV_ERROR   = 0x01U, /**< 通用错误 */
    NERV_BUSY    = 0x02U, /**< 资源忙, 稍后重试 */
    NERV_TIMEOUT = 0x03U, /**< 操作超时 */
} NERV_StatusTypeDef;

/** @brief 无错误, 作为错误寄存器的初始值. */
#define NERV_ERR_NONE 0x00000000U

/** @brief 全部错误位, 只用于清除操作, 不作为错误寄存器的返回值. */
#define NERV_ERR_ALL 0xFFFFFFFFU

/**
  * @brief  错误位掩码.
  * @note   错误位全局唯一, 各模块的 ReadError 直接返回本掩码内的位.
  */
#define NERV_ERR_BITS_MASK 0x00FFFFFFU

/**
  * @brief  由位号生成单比特掩码.
  * @param  _bit 位号, 取值 0 到 23.
  * @return 仅 _bit 位为 1 的掩码.
  */
#define NERV_ERR_BIT(_bit) ((uint32_t)1U << (_bit))

/* 错误位按模块分段:
 * bit 0..8   预留
 * bit 9..10  电机
 * bit 11..12 底盘
 * bit 13..14 云台
 * bit 15..16 发射
 * bit 17..18 安全
 * bit 19..23 预留
 * 每个错误码直接使用 NERV_ERR_BIT(全局位号).
 * 扩展现有模块时优先使用预留位, 并将新位并入该模块的 BITS_MASK.
 * 不移动已有错误位, 避免破坏已经确定的错误码与遥测映射.
 */

/**
  * @brief  电机错误位掩码.
  * @note   覆盖 bit 9, bit 10 与 bit 20.
  */
#define NERV_ERR_MOTOR_BITS_MASK (NERV_ERR_BIT(9U) | NERV_ERR_BIT(10U) | NERV_ERR_BIT(20U))

/**
  * @brief  底盘错误位掩码.
  * @note   覆盖 bit 11 与 bit 12.
  */
#define NERV_ERR_CHASSIS_BITS_MASK (NERV_ERR_BIT(11U) | NERV_ERR_BIT(12U))

/**
  * @brief  云台错误位掩码.
  * @note   覆盖 bit 13 到 bit 14.
  */
#define NERV_ERR_GIMBAL_BITS_MASK (NERV_ERR_BIT(13U) | NERV_ERR_BIT(14U))

/**
  * @brief  发射错误位掩码.
  * @note   覆盖 bit 15 到 bit 16.
  */
#define NERV_ERR_SHOOT_BITS_MASK (NERV_ERR_BIT(15U) | NERV_ERR_BIT(16U))

/**
  * @brief  安全错误位掩码.
  * @note   覆盖 bit 17 到 bit 18.
  */
#define NERV_ERR_SAFETY_BITS_MASK (NERV_ERR_BIT(17U) | NERV_ERR_BIT(18U))

/** @brief 电机离线. */
#define NERV_ERR_MOTOR_OFFLINE NERV_ERR_BIT(9U)

/** @brief 电机过温. */
#define NERV_ERR_MOTOR_OVER_TEMP NERV_ERR_BIT(10U)

/** @brief 电机指令发送失败. */
#define NERV_ERR_MOTOR_TX_FAIL NERV_ERR_BIT(20U)

/** @brief 运动学异常. */
#define NERV_ERR_CHASSIS_KINEMATICS NERV_ERR_BIT(11U)

/** @brief 轮系离线. */
#define NERV_ERR_CHASSIS_WHEEL_OFFLINE NERV_ERR_BIT(12U)

/** @brief 触发限位. */
#define NERV_ERR_GIMBAL_LIMIT NERV_ERR_BIT(13U)

/** @brief 云台离线. */
#define NERV_ERR_GIMBAL_OFFLINE NERV_ERR_BIT(14U)

/** @brief 卡弹. */
#define NERV_ERR_SHOOT_JAM NERV_ERR_BIT(15U)

/** @brief 发射机构离线. */
#define NERV_ERR_SHOOT_OFFLINE NERV_ERR_BIT(16U)

/** @brief 软件急停已触发并锁存, 等待显式复位. */
#define NERV_ERR_SAFETY_ESTOP NERV_ERR_BIT(17U)

/** @brief 安全状态已锁存, 需要显式复位. */
#define NERV_ERR_SAFETY_LATCHED NERV_ERR_BIT(18U)

/* ========================================================================== */
/* 核心契约定义区结束                                                       */
/* ========================================================================== */

#ifdef __cplusplus
}
#endif

#endif /* NERV_STATUS_H */

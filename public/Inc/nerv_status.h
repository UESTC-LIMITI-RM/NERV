/**
  * @file    nerv_status.h
  * @brief   NERV 基础状态码与扩展错误码定义.
  * @version 1.0.0
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

/** @brief 公共域, 供参数, 指针与状态类错误使用. */
#define NERV_ERR_DOMAIN_COMMON 0x00U

/** @brief FDCAN 域, 供通信链路错误使用. */
#define NERV_ERR_DOMAIN_FDCAN 0x01U

/** @brief 电机域, 供电机本体错误使用. */
#define NERV_ERR_DOMAIN_MOTOR 0x02U

/** @brief 底盘域, 供运动学与轮系错误使用. */
#define NERV_ERR_DOMAIN_CHASSIS 0x03U

/** @brief 云台域, 供限位与离线错误使用. */
#define NERV_ERR_DOMAIN_GIMBAL 0x04U

/** @brief 发射域, 供卡弹与离线错误使用. */
#define NERV_ERR_DOMAIN_SHOOT 0x05U

/** @brief 安全域, 供急停与锁存错误使用. */
#define NERV_ERR_DOMAIN_SAFETY 0x06U

/**
  * @brief  由域号生成错误码的域号段.
  * @param  _domain: 域号, 可取以下值:
  *                @arg @ref NERV_ERR_DOMAIN_COMMON 公共域.
  *                @arg @ref NERV_ERR_DOMAIN_FDCAN FDCAN 域.
  *                @arg @ref NERV_ERR_DOMAIN_MOTOR 电机域.
  *                @arg @ref NERV_ERR_DOMAIN_CHASSIS 底盘域.
  *                @arg @ref NERV_ERR_DOMAIN_GIMBAL 云台域.
  *                @arg @ref NERV_ERR_DOMAIN_SHOOT 发射域.
  *                @arg @ref NERV_ERR_DOMAIN_SAFETY 安全域.
  * @return 域号位于高 8 位, 低 24 位为 0 的掩码.
  */
#define NERV_ERR_DOMAIN(_domain) ((uint32_t)(_domain) << 24U)

/**
  * @brief  由位号生成单比特掩码.
  * @param  _bit: 位号, 取值 0 到 23.
  * @return 仅 _bit 位为 1 的掩码.
  */
#define NERV_ERR_BIT(_bit) ((uint32_t)1U << (_bit))

/** @brief 参数非法. */
#define NERV_ERR_COMMON_INVALID_PARAM (NERV_ERR_DOMAIN(NERV_ERR_DOMAIN_COMMON) | NERV_ERR_BIT(0U))

/** @brief 空指针. */
#define NERV_ERR_COMMON_NULL_PTR (NERV_ERR_DOMAIN(NERV_ERR_DOMAIN_COMMON) | NERV_ERR_BIT(1U))

/** @brief 未初始化. */
#define NERV_ERR_COMMON_NOT_INITIALIZED (NERV_ERR_DOMAIN(NERV_ERR_DOMAIN_COMMON) | NERV_ERR_BIT(2U))

/** @brief 内存不足. */
#define NERV_ERR_COMMON_NO_MEMORY (NERV_ERR_DOMAIN(NERV_ERR_DOMAIN_COMMON) | NERV_ERR_BIT(3U))

/** @brief 超出范围. */
#define NERV_ERR_COMMON_OUT_OF_RANGE (NERV_ERR_DOMAIN(NERV_ERR_DOMAIN_COMMON) | NERV_ERR_BIT(4U))

/** @brief 不支持. */
#define NERV_ERR_COMMON_NOT_SUPPORTED (NERV_ERR_DOMAIN(NERV_ERR_DOMAIN_COMMON) | NERV_ERR_BIT(5U))

/** @brief 过滤器配置失败. */
#define NERV_ERR_FDCAN_FILTER (NERV_ERR_DOMAIN(NERV_ERR_DOMAIN_FDCAN) | NERV_ERR_BIT(0U))

/** @brief 发送邮箱不可用. */
#define NERV_ERR_FDCAN_TX_MAILBOX (NERV_ERR_DOMAIN(NERV_ERR_DOMAIN_FDCAN) | NERV_ERR_BIT(1U))

/** @brief 接收溢出. */
#define NERV_ERR_FDCAN_RX_OVERRUN (NERV_ERR_DOMAIN(NERV_ERR_DOMAIN_FDCAN) | NERV_ERR_BIT(2U))

/** @brief 电机离线. */
#define NERV_ERR_MOTOR_OFFLINE (NERV_ERR_DOMAIN(NERV_ERR_DOMAIN_MOTOR) | NERV_ERR_BIT(0U))

/** @brief 电机过温. */
#define NERV_ERR_MOTOR_OVER_TEMP (NERV_ERR_DOMAIN(NERV_ERR_DOMAIN_MOTOR) | NERV_ERR_BIT(1U))

/** @brief 运动学异常. */
#define NERV_ERR_CHASSIS_KINEMATICS (NERV_ERR_DOMAIN(NERV_ERR_DOMAIN_CHASSIS) | NERV_ERR_BIT(0U))

/** @brief 轮系离线. */
#define NERV_ERR_CHASSIS_WHEEL_OFFLINE (NERV_ERR_DOMAIN(NERV_ERR_DOMAIN_CHASSIS) | NERV_ERR_BIT(1U))

/** @brief 触发限位. */
#define NERV_ERR_GIMBAL_LIMIT (NERV_ERR_DOMAIN(NERV_ERR_DOMAIN_GIMBAL) | NERV_ERR_BIT(0U))

/** @brief 云台离线. */
#define NERV_ERR_GIMBAL_OFFLINE (NERV_ERR_DOMAIN(NERV_ERR_DOMAIN_GIMBAL) | NERV_ERR_BIT(1U))

/** @brief 卡弹. */
#define NERV_ERR_SHOOT_JAM (NERV_ERR_DOMAIN(NERV_ERR_DOMAIN_SHOOT) | NERV_ERR_BIT(0U))

/** @brief 发射机构离线. */
#define NERV_ERR_SHOOT_OFFLINE (NERV_ERR_DOMAIN(NERV_ERR_DOMAIN_SHOOT) | NERV_ERR_BIT(1U))

/** @brief 软件急停已触发并锁存, 等待显式复位. */
#define NERV_ERR_SAFETY_ESTOP (NERV_ERR_DOMAIN(NERV_ERR_DOMAIN_SAFETY) | NERV_ERR_BIT(0U))

/** @brief 安全状态已锁存, 需要显式复位. */
#define NERV_ERR_SAFETY_LATCHED (NERV_ERR_DOMAIN(NERV_ERR_DOMAIN_SAFETY) | NERV_ERR_BIT(1U))

/* ========================================================================== */
/* 核心契约定义区结束                                                       */
/* ========================================================================== */

#ifdef __cplusplus
}
#endif

#endif /* NERV_STATUS_H */

/**
  * @file    config_check.c
  * @brief   NERV 编译期配置检查.
  * @version 1.1.0
  * @date    2026-10-22
  * @copyright Copyright (c) 2026 NERV Contributors
  *
  * SPDX-License-Identifier: MIT
  */

#include "config.h"

#include "nerv_status.h"
#include "nerv_types.h"

/* 平台编号与平台选择. */

_Static_assert(NERV_PLATFORM_NONE == 0U, "NERV_PLATFORM_NONE must be 0.");
_Static_assert(NERV_PLATFORM_STM32H7 == 1U, "NERV_PLATFORM_STM32H7 must be 1.");
_Static_assert(NERV_PLATFORM_CUSTOM == 2U, "NERV_PLATFORM_CUSTOM must be 2.");

_Static_assert((NERV_PLATFORM == NERV_PLATFORM_STM32H7) || (NERV_PLATFORM == NERV_PLATFORM_CUSTOM),
               "NERV_PLATFORM must select a supported platform.");

/* 功能开关取值. */

_Static_assert(NERV_ENABLE == 1U, "NERV_ENABLE must be 1.");
_Static_assert(NERV_DISABLE == 0U, "NERV_DISABLE must be 0.");

/* 模块裁剪开关. */

_Static_assert((NERV_USE_LOG == NERV_ENABLE) || (NERV_USE_LOG == NERV_DISABLE),
               "NERV_USE_LOG must be NERV_ENABLE or NERV_DISABLE.");
_Static_assert((NERV_USE_CHASSIS == NERV_ENABLE) || (NERV_USE_CHASSIS == NERV_DISABLE),
               "NERV_USE_CHASSIS must be NERV_ENABLE or NERV_DISABLE.");
_Static_assert((NERV_USE_GIMBAL == NERV_ENABLE) || (NERV_USE_GIMBAL == NERV_DISABLE),
               "NERV_USE_GIMBAL must be NERV_ENABLE or NERV_DISABLE.");
_Static_assert((NERV_USE_SHOOT == NERV_ENABLE) || (NERV_USE_SHOOT == NERV_DISABLE),
               "NERV_USE_SHOOT must be NERV_ENABLE or NERV_DISABLE.");
_Static_assert((NERV_USE_SAFETY == NERV_ENABLE) || (NERV_USE_SAFETY == NERV_DISABLE),
               "NERV_USE_SAFETY must be NERV_ENABLE or NERV_DISABLE.");
_Static_assert((NERV_USE_MOTOR == NERV_ENABLE) || (NERV_USE_MOTOR == NERV_DISABLE),
               "NERV_USE_MOTOR must be NERV_ENABLE or NERV_DISABLE.");

/* 电机协议族裁剪开关. 模块关闭时不检查其专属宏. */

#if (NERV_USE_MOTOR == NERV_ENABLE)

_Static_assert((NERV_USE_MOTOR_DJI == NERV_ENABLE) || (NERV_USE_MOTOR_DJI == NERV_DISABLE),
               "NERV_USE_MOTOR_DJI must be NERV_ENABLE or NERV_DISABLE.");

_Static_assert((NERV_USE_MOTOR_DM == NERV_ENABLE) || (NERV_USE_MOTOR_DM == NERV_DISABLE),
               "NERV_USE_MOTOR_DM must be NERV_ENABLE or NERV_DISABLE.");

#endif /* NERV_USE_MOTOR */

/* 底盘配置. 模块关闭时不检查其专属宏. */

#if (NERV_USE_CHASSIS == NERV_ENABLE)

_Static_assert((NERV_USE_CHASSIS_ODOMETRY == NERV_ENABLE) ||
                   (NERV_USE_CHASSIS_ODOMETRY == NERV_DISABLE),
               "NERV_USE_CHASSIS_ODOMETRY must be NERV_ENABLE or NERV_DISABLE.");

_Static_assert((NERV_CHASSIS_USE_SLOPE_FEEDFORWARD == NERV_ENABLE) ||
                   (NERV_CHASSIS_USE_SLOPE_FEEDFORWARD == NERV_DISABLE),
               "NERV_CHASSIS_USE_SLOPE_FEEDFORWARD must be NERV_ENABLE or NERV_DISABLE.");

_Static_assert((NERV_CHASSIS_USE_DRAG_FEEDFORWARD == NERV_ENABLE) ||
                   (NERV_CHASSIS_USE_DRAG_FEEDFORWARD == NERV_DISABLE),
               "NERV_CHASSIS_USE_DRAG_FEEDFORWARD must be NERV_ENABLE or NERV_DISABLE.");

_Static_assert((NERV_CHASSIS_USE_POWER_LIMIT == NERV_ENABLE) ||
                   (NERV_CHASSIS_USE_POWER_LIMIT == NERV_DISABLE),
               "NERV_CHASSIS_USE_POWER_LIMIT must be NERV_ENABLE or NERV_DISABLE.");

_Static_assert(NERV_CHASSIS_TYPE_MECANUM == 1U, "NERV_CHASSIS_TYPE_MECANUM must be 1.");
_Static_assert(NERV_CHASSIS_TYPE_OMNI == 2U, "NERV_CHASSIS_TYPE_OMNI must be 2.");
_Static_assert(NERV_CHASSIS_TYPE_STEERING == 3U, "NERV_CHASSIS_TYPE_STEERING must be 3.");

_Static_assert((NERV_CHASSIS_TYPE == NERV_CHASSIS_TYPE_MECANUM) ||
                   (NERV_CHASSIS_TYPE == NERV_CHASSIS_TYPE_OMNI) ||
                   (NERV_CHASSIS_TYPE == NERV_CHASSIS_TYPE_STEERING),
               "NERV_CHASSIS_TYPE must select a supported chassis.");

#endif /* NERV_USE_CHASSIS */

/* 云台配置. 模块关闭时不检查其专属宏. */

#if (NERV_USE_GIMBAL == NERV_ENABLE)

_Static_assert(NERV_GIMBAL_AXIS_COUNT_2 == 2U, "NERV_GIMBAL_AXIS_COUNT_2 must be 2.");
_Static_assert(NERV_GIMBAL_AXIS_COUNT_3 == 3U, "NERV_GIMBAL_AXIS_COUNT_3 must be 3.");

_Static_assert((NERV_GIMBAL_AXIS_COUNT == NERV_GIMBAL_AXIS_COUNT_2) ||
                   (NERV_GIMBAL_AXIS_COUNT == NERV_GIMBAL_AXIS_COUNT_3),
               "NERV_GIMBAL_AXIS_COUNT must be 2 or 3.");

#endif /* NERV_USE_GIMBAL */

/* 板间通信链路. */

_Static_assert(NERV_BOARD_LINK_FDCAN == 1U, "NERV_BOARD_LINK_FDCAN must be 1.");
_Static_assert(NERV_BOARD_LINK_UART == 2U, "NERV_BOARD_LINK_UART must be 2.");

_Static_assert((NERV_BOARD_LINK == NERV_BOARD_LINK_FDCAN) ||
                   (NERV_BOARD_LINK == NERV_BOARD_LINK_UART),
               "NERV_BOARD_LINK must select a supported link.");

/* 日志配置. 模块关闭时不检查其专属宏. */

#if (NERV_USE_LOG == NERV_ENABLE)

_Static_assert(NERV_LOG_LEVEL_ERROR == 0U, "NERV_LOG_LEVEL_ERROR must be 0.");
_Static_assert(NERV_LOG_LEVEL_WARN == 1U, "NERV_LOG_LEVEL_WARN must be 1.");
_Static_assert(NERV_LOG_LEVEL_INFO == 2U, "NERV_LOG_LEVEL_INFO must be 2.");
_Static_assert(NERV_LOG_LEVEL_DEBUG == 3U, "NERV_LOG_LEVEL_DEBUG must be 3.");

_Static_assert((NERV_LOG_LEVEL >= NERV_LOG_LEVEL_ERROR) && (NERV_LOG_LEVEL <= NERV_LOG_LEVEL_DEBUG),
               "NERV_LOG_LEVEL is invalid.");
_Static_assert((NERV_LOG_MSG_MAX >= 32U) && (NERV_LOG_MSG_MAX <= 512U),
               "NERV_LOG_MSG_MAX must be in range 32 to 512.");
_Static_assert((NERV_LOG_USE_FLOAT == NERV_ENABLE) || (NERV_LOG_USE_FLOAT == NERV_DISABLE),
               "NERV_LOG_USE_FLOAT must be NERV_ENABLE or NERV_DISABLE.");

#endif /* NERV_USE_LOG */

/* 基础状态码. */

_Static_assert((int)NERV_OK == 0, "NERV_OK must be 0.");
_Static_assert((int)NERV_ERROR == 1, "NERV_ERROR must be 1.");
_Static_assert((int)NERV_BUSY == 2, "NERV_BUSY must be 2.");
_Static_assert((int)NERV_TIMEOUT == 3, "NERV_TIMEOUT must be 3.");

/* 错误位契约. */

_Static_assert(NERV_ERR_NONE == 0U, "NERV_ERR_NONE must be 0.");
_Static_assert(NERV_ERR_ALL == 0xFFFFFFFFU, "NERV_ERR_ALL must be all ones.");
_Static_assert(NERV_ERR_BITS_MASK == 0x00FFFFFFU, "NERV_ERR_BITS_MASK must cover the low 24 bits.");
_Static_assert(NERV_ERR_BIT(0U) == 0x00000001U, "NERV_ERR_BIT(0) must set bit 0.");
_Static_assert(NERV_ERR_BIT(23U) == 0x00800000U, "NERV_ERR_BIT(23) must set bit 23.");

/* 错误位按模块分段. */

_Static_assert(NERV_ERR_MOTOR_BITS_MASK == 0x00100600U,
               "NERV_ERR_MOTOR_BITS_MASK must cover bits 9, 10 and 20.");
_Static_assert(NERV_ERR_CHASSIS_BITS_MASK == 0x00001800U,
               "NERV_ERR_CHASSIS_BITS_MASK must cover bits 11 and 12.");
_Static_assert(NERV_ERR_GIMBAL_BITS_MASK == 0x00006000U,
               "NERV_ERR_GIMBAL_BITS_MASK must cover bits 13 to 14.");
_Static_assert(NERV_ERR_SHOOT_BITS_MASK == 0x00018000U,
               "NERV_ERR_SHOOT_BITS_MASK must cover bits 15 to 16.");
_Static_assert(NERV_ERR_SAFETY_BITS_MASK == 0x00060000U,
               "NERV_ERR_SAFETY_BITS_MASK must cover bits 17 to 18.");

/** @cond INTERNAL */

#define __NERV_CONFIG_CHECK_ERROR_BITS_MASK                                              \
    (NERV_ERR_MOTOR_BITS_MASK | NERV_ERR_CHASSIS_BITS_MASK | NERV_ERR_GIMBAL_BITS_MASK | \
     NERV_ERR_SHOOT_BITS_MASK | NERV_ERR_SAFETY_BITS_MASK)

_Static_assert(__NERV_CONFIG_CHECK_ERROR_BITS_MASK == 0x0017FE00U,
               "Module error bits must cover bits 9 to 18 and 20, leaving bits 0 to 8, "
               "19, 21 to 23 spare.");

/* 错误码的位号与位范围必须与定义一致. */

#define __NERV_CONFIG_CHECK_ERROR(_code, _bit, _mask)           \
    _Static_assert(((uint32_t)(_code) == NERV_ERR_BIT(_bit)) && \
                       (((uint32_t)(_code) & ~(_mask)) == 0U),  \
                   #_code " must use its assigned bit and bit range.")

__NERV_CONFIG_CHECK_ERROR(NERV_ERR_MOTOR_OFFLINE, 9U, NERV_ERR_MOTOR_BITS_MASK);
__NERV_CONFIG_CHECK_ERROR(NERV_ERR_MOTOR_OVER_TEMP, 10U, NERV_ERR_MOTOR_BITS_MASK);
__NERV_CONFIG_CHECK_ERROR(NERV_ERR_MOTOR_TX_FAIL, 20U, NERV_ERR_MOTOR_BITS_MASK);

__NERV_CONFIG_CHECK_ERROR(NERV_ERR_CHASSIS_KINEMATICS, 11U, NERV_ERR_CHASSIS_BITS_MASK);
__NERV_CONFIG_CHECK_ERROR(NERV_ERR_CHASSIS_WHEEL_OFFLINE, 12U, NERV_ERR_CHASSIS_BITS_MASK);

__NERV_CONFIG_CHECK_ERROR(NERV_ERR_GIMBAL_LIMIT, 13U, NERV_ERR_GIMBAL_BITS_MASK);
__NERV_CONFIG_CHECK_ERROR(NERV_ERR_GIMBAL_OFFLINE, 14U, NERV_ERR_GIMBAL_BITS_MASK);

__NERV_CONFIG_CHECK_ERROR(NERV_ERR_SHOOT_JAM, 15U, NERV_ERR_SHOOT_BITS_MASK);
__NERV_CONFIG_CHECK_ERROR(NERV_ERR_SHOOT_OFFLINE, 16U, NERV_ERR_SHOOT_BITS_MASK);

__NERV_CONFIG_CHECK_ERROR(NERV_ERR_SAFETY_ESTOP, 17U, NERV_ERR_SAFETY_BITS_MASK);
__NERV_CONFIG_CHECK_ERROR(NERV_ERR_SAFETY_LATCHED, 18U, NERV_ERR_SAFETY_BITS_MASK);

/* 同一模块内的错误码不得重复. */

#define __NERV_CONFIG_CHECK_ERROR_DISTINCT(_a, _b) \
    _Static_assert((_a) != (_b), #_a " and " #_b " must be distinct.")

__NERV_CONFIG_CHECK_ERROR_DISTINCT(NERV_ERR_MOTOR_OFFLINE, NERV_ERR_MOTOR_OVER_TEMP);
__NERV_CONFIG_CHECK_ERROR_DISTINCT(NERV_ERR_CHASSIS_KINEMATICS, NERV_ERR_CHASSIS_WHEEL_OFFLINE);
__NERV_CONFIG_CHECK_ERROR_DISTINCT(NERV_ERR_GIMBAL_LIMIT, NERV_ERR_GIMBAL_OFFLINE);
__NERV_CONFIG_CHECK_ERROR_DISTINCT(NERV_ERR_SHOOT_JAM, NERV_ERR_SHOOT_OFFLINE);
__NERV_CONFIG_CHECK_ERROR_DISTINCT(NERV_ERR_SAFETY_ESTOP, NERV_ERR_SAFETY_LATCHED);

__NERV_CONFIG_CHECK_ERROR_DISTINCT(NERV_ERR_MOTOR_OFFLINE, NERV_ERR_MOTOR_OVER_TEMP);
__NERV_CONFIG_CHECK_ERROR_DISTINCT(NERV_ERR_CHASSIS_KINEMATICS, NERV_ERR_CHASSIS_WHEEL_OFFLINE);
__NERV_CONFIG_CHECK_ERROR_DISTINCT(NERV_ERR_GIMBAL_LIMIT, NERV_ERR_GIMBAL_OFFLINE);
__NERV_CONFIG_CHECK_ERROR_DISTINCT(NERV_ERR_SHOOT_JAM, NERV_ERR_SHOOT_OFFLINE);
__NERV_CONFIG_CHECK_ERROR_DISTINCT(NERV_ERR_SAFETY_ESTOP, NERV_ERR_SAFETY_LATCHED);

/* 公共结构体大小预算. */

#define __NERV_CONFIG_ATTITUDE_SIZE_MAX 12U

_Static_assert(sizeof(NERV_AttitudeTypeDef) <= __NERV_CONFIG_ATTITUDE_SIZE_MAX,
               "NERV_AttitudeTypeDef exceeds its size budget.");

/** @endcond */

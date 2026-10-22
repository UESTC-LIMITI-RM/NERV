/**
  * @file    config.h
  * @brief   NERV 编译期配置入口.
  * @version 1.0.0
  * @date    2026-10-22
  * @copyright Copyright (c) 2026 NERV Contributors
  *
  * SPDX-License-Identifier: MIT
  */

#ifndef NERV_CONFIG_H
#define NERV_CONFIG_H

/*
 * 本文件是 NERV 唯一的编译期配置入口.
 * 使用者只修改本文件的宏选择, 不在模块源文件中添加板级条件编译.
 * 模块裁剪为 0 时, 该模块的专属配置宏不定义.
 */

/* 平台选择. */

/**
  * @brief 未选择平台.
  * @note  只表示未选择, 不能作为实际平台使用.
  */
#define NERV_PLATFORM_NONE 0U

/**
  * @brief STM32H7 平台, 对应 STM32 HAL 参考 BSP.
  */
#define NERV_PLATFORM_STM32H7 1U

/**
  * @brief 自定义平台, 对应使用者实现的 BSP 适配层.
  */
#define NERV_PLATFORM_CUSTOM 2U

#ifndef NERV_PLATFORM
/**
  * @brief 当前选择的平台.
  * @note  必须取 NERV_PLATFORM_STM32H7 或 NERV_PLATFORM_CUSTOM.
  */
#define NERV_PLATFORM NERV_PLATFORM_NONE
#endif

#if (NERV_PLATFORM == NERV_PLATFORM_NONE)
#error "NERV_PLATFORM must be selected in config.h."
#endif

#if (NERV_PLATFORM != NERV_PLATFORM_STM32H7) && (NERV_PLATFORM != NERV_PLATFORM_CUSTOM)
#error "NERV_PLATFORM is not supported."
#endif

/* 模块裁剪开关. 0 表示不编译该模块, 1 表示编译. */

#ifndef NERV_USE_LOG
/**
  * @brief 日志模块裁剪开关.
  * @note  0 表示禁用, 1 表示启用.
  */
#define NERV_USE_LOG 1
#endif

#if (NERV_USE_LOG != 0) && (NERV_USE_LOG != 1)
#error "NERV_USE_LOG must be 0 or 1."
#endif

#ifndef NERV_USE_CHASSIS
/**
  * @brief 底盘模块裁剪开关.
  * @note  0 表示禁用, 1 表示启用.
  */
#define NERV_USE_CHASSIS 1
#endif

#if (NERV_USE_CHASSIS != 0) && (NERV_USE_CHASSIS != 1)
#error "NERV_USE_CHASSIS must be 0 or 1."
#endif

#ifndef NERV_USE_GIMBAL
/**
  * @brief 云台模块裁剪开关.
  * @note  0 表示禁用, 1 表示启用.
  */
#define NERV_USE_GIMBAL 1
#endif

#if (NERV_USE_GIMBAL != 0) && (NERV_USE_GIMBAL != 1)
#error "NERV_USE_GIMBAL must be 0 or 1."
#endif

#ifndef NERV_USE_SHOOT
/**
  * @brief 发射模块裁剪开关.
  * @note  0 表示禁用, 1 表示启用.
  */
#define NERV_USE_SHOOT 1
#endif

#if (NERV_USE_SHOOT != 0) && (NERV_USE_SHOOT != 1)
#error "NERV_USE_SHOOT must be 0 or 1."
#endif

#ifndef NERV_USE_SAFETY
/**
  * @brief 安全模块裁剪开关.
  * @note  0 表示禁用, 1 表示启用.
  */
#define NERV_USE_SAFETY 1
#endif

#if (NERV_USE_SAFETY != 0) && (NERV_USE_SAFETY != 1)
#error "NERV_USE_SAFETY must be 0 or 1."
#endif

#if (NERV_USE_CHASSIS != 0)

/* 底盘构型. 同一次编译只能选择一种. */

/**
  * @brief 麦克纳姆轮底盘.
  */
#define NERV_CHASSIS_TYPE_MECANUM 1U

/**
  * @brief 全向轮底盘.
  */
#define NERV_CHASSIS_TYPE_OMNI 2U

/**
  * @brief 舵轮底盘.
  */
#define NERV_CHASSIS_TYPE_STEERING 3U

#ifndef NERV_CHASSIS_TYPE
/**
  * @brief 当前选择的底盘构型.
  * @note  必须取 NERV_CHASSIS_TYPE_MECANUM, NERV_CHASSIS_TYPE_OMNI
  *        或 NERV_CHASSIS_TYPE_STEERING.
  */
#define NERV_CHASSIS_TYPE NERV_CHASSIS_TYPE_MECANUM
#endif

#if (NERV_CHASSIS_TYPE != NERV_CHASSIS_TYPE_MECANUM) && \
    (NERV_CHASSIS_TYPE != NERV_CHASSIS_TYPE_OMNI) &&    \
    (NERV_CHASSIS_TYPE != NERV_CHASSIS_TYPE_STEERING)
#error "NERV_CHASSIS_TYPE is not supported."
#endif

#endif /* NERV_USE_CHASSIS */

/* 电机协议族. 同一次编译只能选择一种. */

/**
  * @brief RoboMaster 系列 CAN 电调协议.
  */
#define NERV_MOTOR_PROTOCOL_RM_CAN 1U

/**
  * @brief 自定义电机协议.
  */
#define NERV_MOTOR_PROTOCOL_CUSTOM 2U

#ifndef NERV_MOTOR_PROTOCOL
/**
  * @brief 当前选择的电机协议族.
  * @note  必须取 NERV_MOTOR_PROTOCOL_RM_CAN 或 NERV_MOTOR_PROTOCOL_CUSTOM.
  */
#define NERV_MOTOR_PROTOCOL NERV_MOTOR_PROTOCOL_RM_CAN
#endif

#if (NERV_MOTOR_PROTOCOL != NERV_MOTOR_PROTOCOL_RM_CAN) && \
    (NERV_MOTOR_PROTOCOL != NERV_MOTOR_PROTOCOL_CUSTOM)
#error "NERV_MOTOR_PROTOCOL is not supported."
#endif

#if (NERV_USE_GIMBAL != 0)

/* 云台轴数. 同一次编译只能选择一种. */

/**
  * @brief 两轴云台.
  */
#define NERV_GIMBAL_AXIS_COUNT_2 2U

/**
  * @brief 三轴云台.
  */
#define NERV_GIMBAL_AXIS_COUNT_3 3U

#ifndef NERV_GIMBAL_AXIS_COUNT
/**
  * @brief 当前选择的云台轴数.
  * @note  必须取 NERV_GIMBAL_AXIS_COUNT_2 或 NERV_GIMBAL_AXIS_COUNT_3.
  */
#define NERV_GIMBAL_AXIS_COUNT NERV_GIMBAL_AXIS_COUNT_2
#endif

#if (NERV_GIMBAL_AXIS_COUNT != NERV_GIMBAL_AXIS_COUNT_2) && \
    (NERV_GIMBAL_AXIS_COUNT != NERV_GIMBAL_AXIS_COUNT_3)
#error "NERV_GIMBAL_AXIS_COUNT must be 2 or 3."
#endif

#endif /* NERV_USE_GIMBAL */

/* 板间通信链路. 同一次编译只能选择一种. */

/**
  * @brief 板间链路使用 FDCAN.
  */
#define NERV_BOARD_LINK_FDCAN 1U

/**
  * @brief 板间链路使用 UART.
  */
#define NERV_BOARD_LINK_UART 2U

#ifndef NERV_BOARD_LINK
/**
  * @brief 当前选择的板间通信链路.
  * @note  必须取 NERV_BOARD_LINK_FDCAN 或 NERV_BOARD_LINK_UART.
  */
#define NERV_BOARD_LINK NERV_BOARD_LINK_FDCAN
#endif

#if (NERV_BOARD_LINK != NERV_BOARD_LINK_FDCAN) && (NERV_BOARD_LINK != NERV_BOARD_LINK_UART)
#error "NERV_BOARD_LINK is not supported."
#endif

#if (NERV_USE_LOG != 0)

/* 日志配置. */

/**
  * @brief 错误级别.
  */
#define NERV_LOG_LEVEL_ERROR 0U

/**
  * @brief 警告级别.
  */
#define NERV_LOG_LEVEL_WARN 1U

/**
  * @brief 信息级别.
  */
#define NERV_LOG_LEVEL_INFO 2U

/**
  * @brief 调试级别.
  */
#define NERV_LOG_LEVEL_DEBUG 3U

#ifndef NERV_LOG_LEVEL
/**
  * @brief 当前日志级别, 数值越大输出越多.
  * @note  必须取 NERV_LOG_LEVEL_ERROR, NERV_LOG_LEVEL_WARN,
  *        NERV_LOG_LEVEL_INFO 或 NERV_LOG_LEVEL_DEBUG.
  */
#define NERV_LOG_LEVEL NERV_LOG_LEVEL_INFO
#endif

#if (NERV_LOG_LEVEL < NERV_LOG_LEVEL_ERROR) || (NERV_LOG_LEVEL > NERV_LOG_LEVEL_DEBUG)
#error "NERV_LOG_LEVEL is invalid."
#endif

#ifndef NERV_LOG_MSG_MAX
/**
  * @brief 日志缓冲区大小, 单位 byte, 包含结尾 NUL.
  * @note  取值范围 32 到 512.
  */
#define NERV_LOG_MSG_MAX 160U
#endif

#if (NERV_LOG_MSG_MAX < 32U) || (NERV_LOG_MSG_MAX > 512U)
#error "NERV_LOG_MSG_MAX must be in range 32 to 512."
#endif

#ifndef NERV_LOG_USE_FLOAT
/**
  * @brief 日志浮点格式化开关.
  * @note  0 表示禁用, 1 表示启用. 只在标准库支持浮点格式化时启用.
  */
#define NERV_LOG_USE_FLOAT 0
#endif

#if (NERV_LOG_USE_FLOAT != 0) && (NERV_LOG_USE_FLOAT != 1)
#error "NERV_LOG_USE_FLOAT must be 0 or 1."
#endif

#endif /* NERV_USE_LOG */

#endif /* NERV_CONFIG_H */

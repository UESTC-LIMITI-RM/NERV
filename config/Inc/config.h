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
 * 模块裁剪为 NERV_DISABLE 时, 该模块的专属配置宏不定义.
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

/* 功能开关取值. 所有表示是否启用的配置项统一取这两个值. */

#if defined(NERV_ENABLE)
#error "NERV_ENABLE is defined by the library and must not be defined by the user."
#endif

/**
  * @brief 启用取值.
  * @note  带 NERV_ 前缀是必须的. 裸 ENABLE 与裸 DISABLE 会与 STM32 HAL 的
  *        FunctionalState 枚举成员同名, 在同时包含 config.h 与 HAL 头文件的翻译
  *        单元中产生宏替换冲突.
  */
#define NERV_ENABLE 1U

#if defined(NERV_DISABLE)
#error "NERV_DISABLE is defined by the library and must not be defined by the user."
#endif

/**
  * @brief 禁用取值.
  * @note  编译期开关取本值表示不编译; 运行期限幅与阈值类配置字段取本值表示关闭该限制.
  */
#define NERV_DISABLE 0U

/* 模块裁剪开关. 赋 NERV_ENABLE 表示编译该模块, 赋 NERV_DISABLE 表示不编译. */

#ifndef NERV_USE_LOG
/**
  * @brief 日志模块裁剪开关.
  * @note  取 NERV_ENABLE 或 NERV_DISABLE.
  */
#define NERV_USE_LOG NERV_ENABLE
#endif

#if (NERV_USE_LOG != NERV_ENABLE) && (NERV_USE_LOG != NERV_DISABLE)
#error "NERV_USE_LOG must be NERV_ENABLE or NERV_DISABLE."
#endif

#ifndef NERV_USE_CHASSIS
/**
  * @brief 底盘模块裁剪开关.
  * @note  取 NERV_ENABLE 或 NERV_DISABLE.
  */
#define NERV_USE_CHASSIS NERV_ENABLE
#endif

#if (NERV_USE_CHASSIS != NERV_ENABLE) && (NERV_USE_CHASSIS != NERV_DISABLE)
#error "NERV_USE_CHASSIS must be NERV_ENABLE or NERV_DISABLE."
#endif

#ifndef NERV_USE_GIMBAL
/**
  * @brief 云台模块裁剪开关.
  * @note  取 NERV_ENABLE 或 NERV_DISABLE.
  */
#define NERV_USE_GIMBAL NERV_DISABLE
#endif

#if (NERV_USE_GIMBAL != NERV_ENABLE) && (NERV_USE_GIMBAL != NERV_DISABLE)
#error "NERV_USE_GIMBAL must be NERV_ENABLE or NERV_DISABLE."
#endif

#ifndef NERV_USE_SHOOT
/**
  * @brief 发射模块裁剪开关.
  * @note  取 NERV_ENABLE 或 NERV_DISABLE.
  */
#define NERV_USE_SHOOT NERV_DISABLE
#endif

#if (NERV_USE_SHOOT != NERV_ENABLE) && (NERV_USE_SHOOT != NERV_DISABLE)
#error "NERV_USE_SHOOT must be NERV_ENABLE or NERV_DISABLE."
#endif

#ifndef NERV_USE_SAFETY
/**
  * @brief 安全模块裁剪开关.
  * @note  取 NERV_ENABLE 或 NERV_DISABLE.
  */
#define NERV_USE_SAFETY NERV_DISABLE
#endif

#if (NERV_USE_SAFETY != NERV_ENABLE) && (NERV_USE_SAFETY != NERV_DISABLE)
#error "NERV_USE_SAFETY must be NERV_ENABLE or NERV_DISABLE."
#endif

#ifndef NERV_USE_MOTOR
/**
  * @brief 电机模块裁剪开关.
  * @note  取 NERV_ENABLE 或 NERV_DISABLE.
  */
#define NERV_USE_MOTOR NERV_ENABLE
#endif

#if (NERV_USE_MOTOR != NERV_ENABLE) && (NERV_USE_MOTOR != NERV_DISABLE)
#error "NERV_USE_MOTOR must be NERV_ENABLE or NERV_DISABLE."
#endif

#if (NERV_USE_MOTOR == NERV_ENABLE)

/* 电机协议族裁剪开关. 同一次编译允许启用多个协议族, 每个协议族的驱动独立编译. */

#ifndef NERV_USE_MOTOR_DJI
/**
  * @brief DJI CAN 电调协议族裁剪开关.
  * @note  取 NERV_ENABLE 或 NERV_DISABLE.
  * @note  关闭时 DJI 协议族的驱动源文件与型号符号不参与编译.
  */
#define NERV_USE_MOTOR_DJI NERV_ENABLE
#endif

#if (NERV_USE_MOTOR_DJI != NERV_ENABLE) && (NERV_USE_MOTOR_DJI != NERV_DISABLE)
#error "NERV_USE_MOTOR_DJI must be NERV_ENABLE or NERV_DISABLE."
#endif

#ifndef NERV_USE_MOTOR_DM
/**
  * @brief 达妙 CAN 电机协议族裁剪开关.
  * @note  取 NERV_ENABLE 或 NERV_DISABLE.
  * @note  关闭时达妙协议族的驱动源文件与型号符号不参与编译.
  */
#define NERV_USE_MOTOR_DM NERV_ENABLE
#endif

#if (NERV_USE_MOTOR_DM != NERV_ENABLE) && (NERV_USE_MOTOR_DM != NERV_DISABLE)
#error "NERV_USE_MOTOR_DM must be NERV_ENABLE or NERV_DISABLE."
#endif

#endif /* NERV_USE_MOTOR */

#if (NERV_USE_CHASSIS == NERV_ENABLE)

/* 底盘里程计功能裁剪开关. */

#ifndef NERV_USE_CHASSIS_ODOMETRY
/**
  * @brief 底盘里程计功能裁剪开关.
  * @note  取 NERV_ENABLE 或 NERV_DISABLE.
  * @note  关闭时实例不含里程计字段, Update 不积分位置, NERV_Chassis_GetOdometry 与
  *         NERV_Chassis_SetOdometry 不参与编译.
  */
#define NERV_USE_CHASSIS_ODOMETRY NERV_DISABLE
#endif

#if (NERV_USE_CHASSIS_ODOMETRY != NERV_ENABLE) && (NERV_USE_CHASSIS_ODOMETRY != NERV_DISABLE)
#error "NERV_USE_CHASSIS_ODOMETRY must be NERV_ENABLE or NERV_DISABLE."
#endif

/* 底盘构型. 同一次编译只能选择一种. */

/**
  * @brief 麦克纳姆轮底盘.
  */
#define NERV_CHASSIS_TYPE_MECANUM 1U

/**
  * @brief 全向轮底盘.
  */
#define NERV_CHASSIS_TYPE_OMNI 2U

#ifndef NERV_CHASSIS_TYPE
/**
  * @brief 当前选择的底盘构型.
  * @note  必须取 NERV_CHASSIS_TYPE_MECANUM 或 NERV_CHASSIS_TYPE_OMNI.
  */
#define NERV_CHASSIS_TYPE NERV_CHASSIS_TYPE_OMNI
#endif

#if (NERV_CHASSIS_TYPE != NERV_CHASSIS_TYPE_MECANUM) && \
    (NERV_CHASSIS_TYPE != NERV_CHASSIS_TYPE_OMNI)
#error "NERV_CHASSIS_TYPE is not supported."
#endif

#endif /* NERV_USE_CHASSIS */

#if (NERV_USE_GIMBAL == NERV_ENABLE)

/* 云台轴数. 同一次编译只能选择一种. */

/**
  * @brief 四轴云台, 大 yaw, 小 yaw, pitch, roll 依次串联.
  */
#define NERV_GIMBAL_AXIS_COUNT_4 4U

#ifndef NERV_GIMBAL_AXIS_COUNT
/**
  * @brief 当前选择的云台轴数.
  * @note  必须取 NERV_GIMBAL_AXIS_COUNT_4.
  */
#define NERV_GIMBAL_AXIS_COUNT NERV_GIMBAL_AXIS_COUNT_4
#endif

#if (NERV_GIMBAL_AXIS_COUNT != NERV_GIMBAL_AXIS_COUNT_4)
#error "NERV_GIMBAL_AXIS_COUNT must be NERV_GIMBAL_AXIS_COUNT_4."
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

#if (NERV_USE_LOG == NERV_ENABLE)

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
  * @note  取 NERV_ENABLE 或 NERV_DISABLE. 只在标准库支持浮点格式化时启用.
  */
#define NERV_LOG_USE_FLOAT NERV_DISABLE
#endif

#if (NERV_LOG_USE_FLOAT != NERV_ENABLE) && (NERV_LOG_USE_FLOAT != NERV_DISABLE)
#error "NERV_LOG_USE_FLOAT must be NERV_ENABLE or NERV_DISABLE."
#endif

#endif /* NERV_USE_LOG */

#endif /* NERV_CONFIG_H */

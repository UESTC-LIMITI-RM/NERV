/**
  * @file    nerv_log.h
  * @brief   NERV 日志模块公开接口.
  * @version 1.0.0
  * @date    2026-10-22
  * @copyright Copyright (c) 2026 NERV Contributors
  *
  * SPDX-License-Identifier: MIT
  */

#ifndef NERV_LOG_H
#define NERV_LOG_H

#include <stdint.h>

#include "config.h"
#include "nerv_status.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief  日志级别.
  * @note   成员数值必须与 NERV_LOG_LEVEL_ERROR, NERV_LOG_LEVEL_WARN,
  *         NERV_LOG_LEVEL_INFO 与 NERV_LOG_LEVEL_DEBUG 一致.
  * @note   只在 NERV_USE_LOG 非 0 时进行数值校验.
  */
typedef enum
{
    NERV_LOG_LEVEL_ERROR_RT = 0U, /**< 错误级别 */
    NERV_LOG_LEVEL_WARN_RT  = 1U, /**< 警告级别 */
    NERV_LOG_LEVEL_INFO_RT  = 2U, /**< 信息级别 */
    NERV_LOG_LEVEL_DEBUG_RT = 3U, /**< 调试级别 */
} NERV_Log_LevelTypeDef;

/**
  * @brief  日志输出回调.
  * @param  msg 日志内容起始地址, 长度为 len 字节. 不保证以 NUL 结尾.
  * @param  len 日志内容长度, 单位 byte, 不包含结尾 NUL.
  * @param  user 注册时提供的上下文指针, 由调用者保证生命周期.
  * @note   回调由 NERV_Log_Write 与 NERV_Log_Printf 调用, 默认在任务上下文执行.
  * @note   回调不得阻塞, 不得递归调用日志接口, 不得依赖日志缓冲区在回调返回后仍然有效.
  * @note   回调的线程安全由使用者保证. 库不创建互斥量, 不加锁.
  * @note   NOT ISR-safe.
  */
typedef void (*NERV_Log_OutputTypeDef)(const char *msg, uint16_t len, void *user);

NERV_StatusTypeDef NERV_Log_SetOutput(NERV_Log_OutputTypeDef output, void *user);

NERV_StatusTypeDef NERV_Log_Write(const char *msg, uint16_t len);

NERV_StatusTypeDef NERV_Log_Printf(NERV_Log_LevelTypeDef level, const char *file, uint32_t line,
                                   const char *fmt, ...);

#if (NERV_USE_LOG != 0)

/**
  * @brief  按级别格式化并输出日志.
  * @param  _level 日志级别, 取 NERV_LOG_LEVEL_ERROR, NERV_LOG_LEVEL_WARN,
  *                NERV_LOG_LEVEL_INFO 或 NERV_LOG_LEVEL_DEBUG.
  * @param  _file 源文件名, 通常传入 __FILE__.
  * @param  _line 源码行号, 通常传入 __LINE__.
  * @param  ... 格式串与可变参数, 至少包含一个格式串实参.
  * @note   展开体使用 do { ... } while (0), 不改变调用方控制流.
  * @note   _level, _file 与 _line 各求值一次.
  * @note   格式串中的浮点格式只在 NERV_LOG_USE_FLOAT 为 1 且标准库支持时使用.
  * @warning 实参中不得包含带副作用的表达式.
  * @note   NOT ISR-safe.
  */
#define NERV_LOG_FORMAT_AND_WRITE(_level, _file, _line, ...)                                   \
    do                                                                                         \
    {                                                                                          \
        (void)NERV_Log_Printf((NERV_Log_LevelTypeDef)(_level), (_file), (_line), __VA_ARGS__); \
    }                                                                                          \
    while (0)

/**
  * @brief  输出错误级别日志.
  * @param  ... 格式串与可变参数, 至少包含一个格式串实参.
  * @note   NERV_USE_LOG 为 0 时展开为空操作, 实参不求值.
  * @warning 实参中不得包含带副作用的表达式.
  * @note   NOT ISR-safe.
  */
#define NERV_LOG_ERROR(...) \
    NERV_LOG_FORMAT_AND_WRITE(NERV_LOG_LEVEL_ERROR, __FILE__, __LINE__, __VA_ARGS__)

/**
  * @brief  输出警告级别日志.
  * @param  ... 格式串与可变参数, 至少包含一个格式串实参.
  * @note   NERV_USE_LOG 为 0 时展开为空操作, 实参不求值.
  * @warning 实参中不得包含带副作用的表达式.
  * @note   NOT ISR-safe.
  */
#define NERV_LOG_WARN(...) \
    NERV_LOG_FORMAT_AND_WRITE(NERV_LOG_LEVEL_WARN, __FILE__, __LINE__, __VA_ARGS__)

/**
  * @brief  输出信息级别日志.
  * @param  ... 格式串与可变参数, 至少包含一个格式串实参.
  * @note   NERV_USE_LOG 为 0 时展开为空操作, 实参不求值.
  * @warning 实参中不得包含带副作用的表达式.
  * @note   NOT ISR-safe.
  */
#define NERV_LOG_INFO(...) \
    NERV_LOG_FORMAT_AND_WRITE(NERV_LOG_LEVEL_INFO, __FILE__, __LINE__, __VA_ARGS__)

/**
  * @brief  输出调试级别日志.
  * @param  ... 格式串与可变参数, 至少包含一个格式串实参.
  * @note   NERV_USE_LOG 为 0 时展开为空操作, 实参不求值.
  * @warning 实参中不得包含带副作用的表达式.
  * @note   NOT ISR-safe.
  */
#define NERV_LOG_DEBUG(...) \
    NERV_LOG_FORMAT_AND_WRITE(NERV_LOG_LEVEL_DEBUG, __FILE__, __LINE__, __VA_ARGS__)

#else

/**
  * @brief  按级别格式化并输出日志.
  * @param  _level 日志级别. 未使用.
  * @param  _file 源文件名. 未使用.
  * @param  _line 源码行号. 未使用.
  * @param  ... 格式串与可变参数. 未使用.
  * @note   NERV_USE_LOG 为 0 时展开为空操作, 实参不求值.
  */
#define NERV_LOG_FORMAT_AND_WRITE(_level, _file, _line, ...) ((void)0)

/**
  * @brief  输出错误级别日志.
  * @param  ... 格式串与可变参数. 未使用.
  * @note   NERV_USE_LOG 为 0 时展开为空操作, 实参不求值.
  */
#define NERV_LOG_ERROR(...)                                  ((void)0)

/**
  * @brief  输出警告级别日志.
  * @param  ... 格式串与可变参数. 未使用.
  * @note   NERV_USE_LOG 为 0 时展开为空操作, 实参不求值.
  */
#define NERV_LOG_WARN(...)                                   ((void)0)

/**
  * @brief  输出信息级别日志.
  * @param  ... 格式串与可变参数. 未使用.
  * @note   NERV_USE_LOG 为 0 时展开为空操作, 实参不求值.
  */
#define NERV_LOG_INFO(...)                                   ((void)0)

/**
  * @brief  输出调试级别日志.
  * @param  ... 格式串与可变参数. 未使用.
  * @note   NERV_USE_LOG 为 0 时展开为空操作, 实参不求值.
  */
#define NERV_LOG_DEBUG(...)                                  ((void)0)

#endif /* NERV_USE_LOG */

#ifdef __cplusplus
}
#endif

#endif /* NERV_LOG_H */

/**
  * @file    nerv_log.c
  * @brief   NERV 日志模块实现.
  * @version 1.0.0
  * @date    2026-10-22
  * @copyright Copyright (c) 2026 NERV Contributors
  *
  * SPDX-License-Identifier: MIT
  */

#include "nerv_log.h"

#include <stdarg.h>
#include <stdio.h>

#if (NERV_USE_LOG != 0)

_Static_assert((int)NERV_LOG_LEVEL_ERROR == (int)NERV_LOG_LEVEL_ERROR_RT,
               "NERV_LOG_LEVEL_ERROR and NERV_LOG_LEVEL_ERROR_RT must be equal.");
_Static_assert((int)NERV_LOG_LEVEL_WARN == (int)NERV_LOG_LEVEL_WARN_RT,
               "NERV_LOG_LEVEL_WARN and NERV_LOG_LEVEL_WARN_RT must be equal.");
_Static_assert((int)NERV_LOG_LEVEL_INFO == (int)NERV_LOG_LEVEL_INFO_RT,
               "NERV_LOG_LEVEL_INFO and NERV_LOG_LEVEL_INFO_RT must be equal.");
_Static_assert((int)NERV_LOG_LEVEL_DEBUG == (int)NERV_LOG_LEVEL_DEBUG_RT,
               "NERV_LOG_LEVEL_DEBUG and NERV_LOG_LEVEL_DEBUG_RT must be equal.");

/** @brief 当前输出回调, NULL 表示尚未注册. */
static NERV_Log_OutputTypeDef nerv_log_output;

/** @brief 当前输出回调上下文, 可以为 NULL. */
static void *nerv_log_user;

#endif /* NERV_USE_LOG */

/**
  * @brief  设置日志输出回调.
  * @param  output 输出回调, 不得为 NULL.
  * @param  user 回调上下文, 由调用者保证生命周期. 可以为 NULL.
  * @retval NERV_OK 输出回调与上下文已保存.
  * @retval NERV_ERROR output 为 NULL.
  * @note   不访问硬件, 不分配内存, 不创建任务或互斥量.
  * @note   启动时调用一次即完成初始化. 需要更换输出时再次调用, 后一次覆盖前一次.
  * @note   替换不得与日志输出并发: 输出回调与上下文作为一对读取, 替换瞬间的并发输出
  *         可能读到新回调与旧上下文的组合.
  * @note   NERV_USE_LOG 为 0 时, 本函数为同签名空实现, 总是返回 NERV_OK.
  * @note   NOT ISR-safe.
  * @note   caller-synchronized. 同一时刻只能有一个调用者修改输出回调.
  */
NERV_StatusTypeDef NERV_Log_SetOutput(NERV_Log_OutputTypeDef output, void *user)
{
#if (NERV_USE_LOG != 0)
    if (NULL == output)
    {
        return NERV_ERROR;
    }

    nerv_log_output = output;
    nerv_log_user   = user;

    return NERV_OK;
#else
    (void)output;
    (void)user;
    return NERV_OK;
#endif
}

/**
  * @brief  把已经格式化的字节序列交给输出回调.
  * @param  msg 日志内容起始地址, 不得为 NULL.
  * @param  len 日志内容长度, 单位 byte, 不得为 0.
  * @retval NERV_OK 内容已交给输出回调.
  * @retval NERV_ERROR msg 为 NULL, len 为 0, 或输出回调尚未注册.
  * @note   回调在调用者上下文执行, 不得阻塞, 不得递归调用日志接口.
  * @note   msg 只在回调执行期间有效, 回调不得保存该指针.
  * @note   NERV_USE_LOG 为 0 时, 本函数为同签名空实现, 总是返回 NERV_OK.
  * @note   NOT ISR-safe.
  * @note   caller-synchronized.
  */
NERV_StatusTypeDef NERV_Log_Write(const char *msg, uint16_t len)
{
#if (NERV_USE_LOG != 0)
    if ((NULL == msg) || (0U == len))
    {
        return NERV_ERROR;
    }

    if (NULL == nerv_log_output)
    {
        return NERV_ERROR;
    }

    nerv_log_output(msg, len, nerv_log_user);

    return NERV_OK;
#else
    (void)msg;
    (void)len;
    return NERV_OK;
#endif
}

/**
  * @brief  按级别格式化并输出日志.
  * @param  level 日志级别, 必须是 NERV_Log_LevelTypeDef 的成员值.
  * @param  file 源文件名, 不得为 NULL. 当前实现不写入消息.
  * @param  line 源码行号. 当前实现不写入消息.
  * @param  fmt 格式串, 不得为 NULL.
  * @param  ... fmt 对应的可变参数.
  * @retval NERV_OK 日志已输出, 或 level 高于当前配置级别而被过滤.
  * @retval NERV_ERROR file 或 fmt 为 NULL, level 非法, 格式化失败, 或输出失败.
  * @note   格式化缓冲区位于调用者栈上, 大小为 NERV_LOG_MSG_MAX byte.
  * @note   结果超过缓冲区时截断, 保留前 NERV_LOG_MSG_MAX - 1 个 byte.
  * @note   NERV_LOG_USE_FLOAT 为 0 时, fmt 不得包含浮点格式; 否则行为取决于标准库.
  * @note   NERV_LOG_USE_FLOAT 只描述工具链能力, 不改变 vsnprintf 的行为.
  * @note   NERV_USE_LOG 为 0 时, 本函数为同签名空实现, 总是返回 NERV_OK.
  * @note   不得在 ISR 中调用.
  * @note   NOT ISR-safe.
  * @note   caller-synchronized.
  */
NERV_StatusTypeDef NERV_Log_Printf(NERV_Log_LevelTypeDef level, const char *file, uint32_t line,
                                   const char *fmt, ...)
{
#if (NERV_USE_LOG != 0)
    char msg[NERV_LOG_MSG_MAX];
    int written;
    va_list args;
    uint16_t len;

    if ((NULL == file) || (NULL == fmt))
    {
        return NERV_ERROR;
    }

    if ((uint32_t)level > (uint32_t)NERV_LOG_LEVEL_DEBUG_RT)
    {
        return NERV_ERROR;
    }

    if ((uint32_t)level > (uint32_t)NERV_LOG_LEVEL)
    {
        return NERV_OK;
    }

    va_start(args, fmt);
    written = vsnprintf(msg, sizeof(msg), fmt, args);
    va_end(args);

    if (written < 0)
    {
        return NERV_ERROR;
    }

    if ((size_t)written >= sizeof(msg))
    {
        len = (uint16_t)(sizeof(msg) - 1U);
    }
    else
    {
        len = (uint16_t)written;
    }

    (void)file;
    (void)line;

    return NERV_Log_Write(msg, len);
#else
    (void)level;
    (void)file;
    (void)line;
    (void)fmt;
    return NERV_OK;
#endif
}

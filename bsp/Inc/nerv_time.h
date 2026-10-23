/**
  * @file    nerv_time.h
  * @brief   NERV 时间基准外设抽象接口.
  * @version 1.0.0
  * @date    2026-10-23
  * @copyright Copyright (c) 2026 NERV Contributors
  *
  * SPDX-License-Identifier: MIT
  */

#ifndef NERV_TIME_H
#define NERV_TIME_H

#include <stddef.h>
#include <stdint.h>

#include "nerv_status.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 实例类型前向声明, 完整定义在操作表之后. */

/**
  * @brief  时间基准实例类型, 完整定义见 NERV_Time_Instance.
  */
typedef struct NERV_Time_Instance NERV_Time_InstanceTypeDef;

/**
  * @brief  时间基准平台操作表.
  * @note   由平台适配层实现并绑定到实例, 绑定后不再更改. 模块层只调用通用接口,
  *         不直接调用操作表成员.
  * @note   成员契约:
  *         - init: 初始化时基并开始计数, 失败返回 NERV_ERROR. NOT ISR-safe.
  *         - get_tick_ms: 读取自时基启动以来的毫秒计数, 不阻塞, 不分配内存,
  *           不调用日志. ISR-safe.
  */
typedef struct
{
    NERV_StatusTypeDef (*init)(NERV_Time_InstanceTypeDef *self);    /**< 初始化并开始计数 */
    uint32_t (*get_tick_ms)(const NERV_Time_InstanceTypeDef *self); /**< 读取毫秒计数 */
} NERV_Time_OpsTypeDef;

/**
  * @brief  时间基准实例.
  * @note   实例由使用者分配并清零, 生命周期必须覆盖全部调用.
  */
struct NERV_Time_Instance
{
    const NERV_Time_OpsTypeDef *ops; /**< 平台操作表, 绑定后不再更改 */
    void *platform_data;             /**< 平台私有数据, 由平台适配层解释 */
};

/* 通用接口. 全部为静态内联, 只做参数检查与操作表分发, 行为契约见各函数文档.
 * NERV_Time_GetTickMs 需在 NERV_Time_Init 之后调用. */

/**
  * @brief  初始化时间基准.
  * @param  self 时间基准实例指针, 不得为 NULL.
  * @retval NERV_OK 时基已初始化并开始计数.
  * @retval NERV_ERROR self 为 NULL, 操作表未绑定, 或平台初始化失败.
  * @note   NOT ISR-safe.
  * @note   时基实现在外部定时器外围设备或 DCB 平台时基, 由平台适配层决定.
  *         多个实例共用同一时基时, 每个实例读取到的计数一致.
  */
static inline NERV_StatusTypeDef NERV_Time_Init(NERV_Time_InstanceTypeDef *self)
{
    if (NULL == self)
    {
        return NERV_ERROR;
    }

    if ((NULL == self->ops) || (NULL == self->ops->init))
    {
        return NERV_ERROR;
    }

    return self->ops->init(self);
}

/**
  * @brief  读取时基毫秒计数.
  * @param  self 时间基准实例指针, 不得为 NULL.
  * @return 自时基启动以来的毫秒计数, 按 32 位无符号整数回绕.
  * @note   需在 NERV_Time_Init 之后调用.
  * @note   调用方计算间隔时必须使用无符号减法, 例如 (tick_now - tick_start),
  *         回绕不会破坏该结果的正确性. 间隔值必须显著小于回绕周期, 约 49.7 天.
  * @note   self 为 NULL 或操作表未绑定时返回 0U.
  * @note   ISR-safe.
  */
static inline uint32_t NERV_Time_GetTickMs(const NERV_Time_InstanceTypeDef *self)
{
    if ((NULL == self) || (NULL == self->ops) || (NULL == self->ops->get_tick_ms))
    {
        return 0U;
    }

    return self->ops->get_tick_ms(self);
}

#ifdef __cplusplus
}
#endif

#endif /* NERV_TIME_H */

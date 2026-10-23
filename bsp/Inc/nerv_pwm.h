/**
  * @file    nerv_pwm.h
  * @brief   NERV PWM 外设抽象接口.
  * @version 1.0.0
  * @date    2026-10-23
  * @copyright Copyright (c) 2026 NERV Contributors
  *
  * SPDX-License-Identifier: MIT
  */

#ifndef NERV_PWM_H
#define NERV_PWM_H

#include <math.h>
#include <stddef.h>
#include <stdint.h>

#include "nerv_status.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 实例类型前向声明, 完整定义在操作表之后. */

/**
  * @brief  PWM 实例类型, 完整定义见 NERV_PWM_Instance.
  */
typedef struct NERV_PWM_Instance NERV_PWM_InstanceTypeDef;

/**
  * @brief  PWM 平台操作表.
  * @note   由平台适配层实现并绑定到实例, 绑定后不再更改. 模块层只调用通用接口,
  *         不直接调用操作表成员.
  * @note   成员契约:
  *         - init: 初始化输出并设置初始占空比, 失败返回 NERV_ERROR. NOT ISR-safe.
  *         - deinit: 释放 PWM 输出, 失败返回 NERV_ERROR. NOT ISR-safe.
  *         - set_duty: 写入占空比, 不阻塞, 不分配内存, 不调用日志. ISR-safe.
  */
typedef struct
{
    NERV_StatusTypeDef (*init)(NERV_PWM_InstanceTypeDef *self,
                               float init_duty);                  /**< 初始化并输出初始占空比 */
    NERV_StatusTypeDef (*deinit)(NERV_PWM_InstanceTypeDef *self); /**< 释放 PWM 输出 */
    NERV_StatusTypeDef (*set_duty)(NERV_PWM_InstanceTypeDef *self, float duty); /**< 写入占空比 */
} NERV_PWM_OpsTypeDef;

/**
  * @brief  PWM 实例.
  * @note   实例由使用者分配并清零, 生命周期必须覆盖全部调用.
  */
struct NERV_PWM_Instance
{
    const NERV_PWM_OpsTypeDef *ops; /**< 平台操作表, 绑定后不再更改 */
    void *platform_data;            /**< 平台私有数据, 由平台适配层解释 */
};

/* 通用接口. 全部为静态内联, 只做参数检查与操作表分发, 行为契约见各函数文档.
 * NERV_PWM_SetDuty 需在 NERV_PWM_Init 之后调用, NERV_PWM_DeInit 之后不应再调用. */

/**
  * @brief  初始化 PWM 输出.
  * @param  self PWM 实例指针, 不得为 NULL.
  * @param  init_duty 初始占空比, 无单位, 取值 0.0 到 1.0.
  * @retval NERV_OK PWM 输出已初始化并输出 init_duty.
  * @retval NERV_ERROR self 为 NULL, 操作表未绑定, init_duty 非有限或超出范围,
  *         或平台初始化失败.
  * @note   NOT ISR-safe.
  * @note   PWM 载波频率, 计数位宽等输出属性由平台配置决定, 本抽象只控制占空比.
  */
static inline NERV_StatusTypeDef NERV_PWM_Init(NERV_PWM_InstanceTypeDef *self, float init_duty)
{
    if (NULL == self)
    {
        return NERV_ERROR;
    }

    if ((NULL == self->ops) || (NULL == self->ops->init))
    {
        return NERV_ERROR;
    }

    if ((!isfinite(init_duty)) || (0.0f > init_duty) || (1.0f < init_duty))
    {
        return NERV_ERROR;
    }

    return self->ops->init(self, init_duty);
}

/**
  * @brief  释放 PWM 输出.
  * @param  self PWM 实例指针, 不得为 NULL.
  * @retval NERV_OK PWM 输出已释放.
  * @retval NERV_ERROR self 为 NULL, 操作表未绑定, 或平台释放失败.
  * @note   NOT ISR-safe.
  */
static inline NERV_StatusTypeDef NERV_PWM_DeInit(NERV_PWM_InstanceTypeDef *self)
{
    if (NULL == self)
    {
        return NERV_ERROR;
    }

    if ((NULL == self->ops) || (NULL == self->ops->deinit))
    {
        return NERV_ERROR;
    }

    return self->ops->deinit(self);
}

/**
  * @brief  设置占空比.
  * @param  self PWM 实例指针, 不得为 NULL.
  * @param  duty 目标占空比, 无单位, 取值 0.0 到 1.0.
  * @retval NERV_OK 占空比已写入.
  * @retval NERV_ERROR self 为 NULL, 操作表未绑定, duty 非有限或超出范围, 或平台写入失败.
  * @note   需在 NERV_PWM_Init 之后调用.
  * @note   进入安全状态时以 duty 0.0 调用本函数即可停止输出, 高电平有效;
  *         低电平有效输出的停止语义由平台配置决定.
  * @note   ISR-safe.
  */
static inline NERV_StatusTypeDef NERV_PWM_SetDuty(NERV_PWM_InstanceTypeDef *self, float duty)
{
    if (NULL == self)
    {
        return NERV_ERROR;
    }

    if ((NULL == self->ops) || (NULL == self->ops->set_duty))
    {
        return NERV_ERROR;
    }

    if ((!isfinite(duty)) || (0.0f > duty) || (1.0f < duty))
    {
        return NERV_ERROR;
    }

    return self->ops->set_duty(self, duty);
}

#ifdef __cplusplus
}
#endif

#endif /* NERV_PWM_H */

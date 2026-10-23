/**
  * @file    nerv_gpio.h
  * @brief   NERV GPIO 外设抽象接口.
  * @version 1.0.0
  * @date    2026-10-23
  * @copyright Copyright (c) 2026 NERV Contributors
  *
  * SPDX-License-Identifier: MIT
  */

#ifndef NERV_GPIO_H
#define NERV_GPIO_H

#include <stddef.h>
#include <stdint.h>

#include "nerv_status.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 实例类型前向声明, 完整定义在操作表之后. */

/**
  * @brief  GPIO 实例类型, 完整定义见 NERV_GPIO_Instance.
  */
typedef struct NERV_GPIO_Instance NERV_GPIO_InstanceTypeDef;

/**
  * @brief  GPIO 引脚方向.
  */
typedef enum
{
    NERV_GPIO_DIRECTION_INPUT  = 0U, /**< 输入, 引脚为高阻输入 */
    NERV_GPIO_DIRECTION_OUTPUT = 1U, /**< 输出, 推挽等电气属性由平台配置决定 */
} NERV_GPIO_DirectionTypeDef;

/**
  * @brief  GPIO 上下拉配置.
  */
typedef enum
{
    NERV_GPIO_PULL_NONE = 0U, /**< 无上下拉 */
    NERV_GPIO_PULL_UP   = 1U, /**< 内部上拉 */
    NERV_GPIO_PULL_DOWN = 2U, /**< 内部下拉 */
} NERV_GPIO_PullTypeDef;

/**
  * @brief  GPIO 引脚电平.
  * @note   按位掩码存放在电平寄存器中, 高电平为 1.
  */
typedef enum
{
    NERV_GPIO_LEVEL_LOW  = 0U, /**< 低电平 */
    NERV_GPIO_LEVEL_HIGH = 1U, /**< 高电平 */
} NERV_GPIO_LevelTypeDef;

/**
  * @brief  GPIO 引脚配置.
  * @note   本结构按值使用, 平台适配层在 init 中完成配置, 不保存指向本结构的指针,
  *         调用方无需保证生命周期覆盖实例.
  * @note   init_level 在 direction 为 NERV_GPIO_DIRECTION_OUTPUT 时作为初始化电平使用,
  *         direction 为 NERV_GPIO_DIRECTION_INPUT 时无效.
  */
typedef struct NERV_GPIO_Config
{
    NERV_GPIO_DirectionTypeDef direction; /**< 引脚方向 */
    NERV_GPIO_PullTypeDef pull;           /**< 上下拉选择 */
    NERV_GPIO_LevelTypeDef init_level;    /**< 输出方向时的初始化电平, 输入方向时无效 */
} NERV_GPIO_ConfigTypeDef;

/**
  * @brief  GPIO 平台操作表.
  * @note   由平台适配层实现并绑定到实例, 绑定后不再更改. 模块层只调用通用接口,
  *         不直接调用操作表成员.
  * @note   成员契约:
  *         - init: 按配置初始化引脚, 失败返回 NERV_ERROR. NOT ISR-safe.
  *         - deinit: 释放引脚配置, 失败返回 NERV_ERROR. NOT ISR-safe.
  *         - write: 写入输出电平, 不阻塞, 不分配内存, 不调用日志. ISR-safe.
  *         - read: 读取引脚电平, 不阻塞, 不分配内存, 不调用日志. ISR-safe.
  */
typedef struct
{
    NERV_StatusTypeDef (*init)(NERV_GPIO_InstanceTypeDef *self,
                               const NERV_GPIO_ConfigTypeDef *config); /**< 初始化引脚 */
    NERV_StatusTypeDef (*deinit)(NERV_GPIO_InstanceTypeDef *self);     /**< 释放引脚配置 */
    NERV_StatusTypeDef (*write)(NERV_GPIO_InstanceTypeDef *self,
                                NERV_GPIO_LevelTypeDef level); /**< 写入电平 */
    NERV_StatusTypeDef (*read)(const NERV_GPIO_InstanceTypeDef *self,
                               NERV_GPIO_LevelTypeDef *level); /**< 读取电平 */
} NERV_GPIO_OpsTypeDef;

/**
  * @brief  GPIO 实例.
  * @note   实例由使用者分配并清零, 生命周期必须覆盖全部调用.
  */
struct NERV_GPIO_Instance
{
    const NERV_GPIO_OpsTypeDef *ops; /**< 平台操作表, 绑定后不再更改 */
    void *platform_data;             /**< 平台私有数据, 由平台适配层解释 */
};

/* 通用接口. 全部为静态内联, 只做参数检查与操作表分发, 行为契约见各函数文档.
 * NERV_GPIO_Write 与 NERV_GPIO_Read 需在 NERV_GPIO_Init 之后调用,
 * NERV_GPIO_DeInit 之后不应再调用二者. */

/**
  * @brief  初始化 GPIO 引脚.
  * @param  self GPIO 实例指针, 不得为 NULL.
  * @param  config 引脚配置指针, 不得为 NULL, 按值使用, 不保存指向本结构的指针.
  * @retval NERV_OK 引脚已按配置初始化.
  * @retval NERV_ERROR self 或 config 为 NULL, 操作表未绑定, config 中枚举值非法,
  *         或平台初始化失败.
  * @note   NOT ISR-safe.
  */
static inline NERV_StatusTypeDef NERV_GPIO_Init(NERV_GPIO_InstanceTypeDef *self,
                                                const NERV_GPIO_ConfigTypeDef *config)
{
    if ((NULL == self) || (NULL == config))
    {
        return NERV_ERROR;
    }

    if ((NULL == self->ops) || (NULL == self->ops->init))
    {
        return NERV_ERROR;
    }

    switch (config->direction)
    {
    case NERV_GPIO_DIRECTION_INPUT:
    case NERV_GPIO_DIRECTION_OUTPUT:
        break;
    default:
        return NERV_ERROR;
    }

    switch (config->pull)
    {
    case NERV_GPIO_PULL_NONE:
    case NERV_GPIO_PULL_UP:
    case NERV_GPIO_PULL_DOWN:
        break;
    default:
        return NERV_ERROR;
    }

    switch (config->init_level)
    {
    case NERV_GPIO_LEVEL_LOW:
    case NERV_GPIO_LEVEL_HIGH:
        break;
    default:
        return NERV_ERROR;
    }

    return self->ops->init(self, config);
}

/**
  * @brief  释放 GPIO 引脚配置.
  * @param  self GPIO 实例指针, 不得为 NULL.
  * @retval NERV_OK 引脚配置已释放.
  * @retval NERV_ERROR self 为 NULL, 操作表未绑定, 或平台释放失败.
  * @note   NOT ISR-safe.
  */
static inline NERV_StatusTypeDef NERV_GPIO_DeInit(NERV_GPIO_InstanceTypeDef *self)
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
  * @brief  设置输出引脚电平.
  * @param  self GPIO 实例指针, 不得为 NULL.
  * @param  level 目标电平.
  * @retval NERV_OK 电平已写入.
  * @retval NERV_ERROR self 为 NULL, 操作表未绑定, level 值非法, 或平台写入失败.
  * @note   仅当引脚方向为 NERV_GPIO_DIRECTION_OUTPUT 时调用.
  * @note   ISR-safe.
  */
static inline NERV_StatusTypeDef NERV_GPIO_Write(NERV_GPIO_InstanceTypeDef *self,
                                                 NERV_GPIO_LevelTypeDef level)
{
    if (NULL == self)
    {
        return NERV_ERROR;
    }

    if ((NULL == self->ops) || (NULL == self->ops->write))
    {
        return NERV_ERROR;
    }

    switch (level)
    {
    case NERV_GPIO_LEVEL_LOW:
    case NERV_GPIO_LEVEL_HIGH:
        break;
    default:
        return NERV_ERROR;
    }

    return self->ops->write(self, level);
}

/**
  * @brief  读取引脚电平.
  * @param  self GPIO 实例指针, 不得为 NULL.
  * @param  level 电平结果输出指针, 不得为 NULL. 成功时写入, 失败时不修改.
  * @retval NERV_OK 电平结果已写入 level.
  * @retval NERV_ERROR self 或 level 为 NULL, 操作表未绑定, 或平台读取失败.
  * @note   ISR-safe.
  */
static inline NERV_StatusTypeDef NERV_GPIO_Read(const NERV_GPIO_InstanceTypeDef *self,
                                                NERV_GPIO_LevelTypeDef *level)
{
    if ((NULL == self) || (NULL == level))
    {
        return NERV_ERROR;
    }

    if ((NULL == self->ops) || (NULL == self->ops->read))
    {
        return NERV_ERROR;
    }

    return self->ops->read(self, level);
}

#ifdef __cplusplus
}
#endif

#endif /* NERV_GPIO_H */

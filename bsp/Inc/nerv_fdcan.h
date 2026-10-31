/**
  * @file    nerv_fdcan.h
  * @brief   NERV FDCAN 外设抽象接口.
  * @version 1.0.0
  * @date    2026-10-22
  * @copyright Copyright (c) 2026 NERV Contributors
  *
  * SPDX-License-Identifier: MIT
  */

#ifndef NERV_FDCAN_H
#define NERV_FDCAN_H

#include <stddef.h>
#include <stdint.h>

#include "nerv_status.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/* 以下定义为 CAN 协议固有常数, 禁止修改.                                    */
/* 作用域: NERV_FDCAN_CLASSIC_DATA_MAX, NERV_FDCAN_DATA_MAX,                 */
/*         NERV_FDCAN_ID_STANDARD_MAX 与 NERV_FDCAN_ID_EXTENDED_MAX.          */
/* ========================================================================== */

/** @brief 经典帧数据区长度上限, 单位 byte. */
#define NERV_FDCAN_CLASSIC_DATA_MAX 8U

/** @brief FDCAN 帧数据区最大长度, 单位 byte. */
#define NERV_FDCAN_DATA_MAX 64U

/** @brief 标准帧 11 位标识符的最大值. */
#define NERV_FDCAN_ID_STANDARD_MAX 0x7FFU

/** @brief 扩展帧 29 位标识符的最大值. */
#define NERV_FDCAN_ID_EXTENDED_MAX 0x1FFFFFFFU

/* ========================================================================== */
/* CAN 协议常数定义区结束                                                   */
/* ========================================================================== */

/* 实例类型前向声明, 完整定义在操作表之后. */

typedef struct NERV_FDCAN_Instance NERV_FDCAN_InstanceTypeDef;

/**
  * @brief  FDCAN 帧标志.
  * @note   取值按位掩码使用, 存放在帧与过滤器的 flags 成员中.
  */
typedef enum
{
    NERV_FDCAN_FLAG_NONE     = 0x00U, /**< 无标志, 经典 CAN 标准帧 */
    NERV_FDCAN_FLAG_EXTENDED = 0x01U, /**< 扩展帧, 标识符使用 29 位 */
    NERV_FDCAN_FLAG_FD       = 0x02U, /**< FDCAN 帧, 数据长度可达 64 字节 */
    NERV_FDCAN_FLAG_BRS      = 0x04U, /**< 位速率切换, 只能与 NERV_FDCAN_FLAG_FD 同时使用 */
} NERV_FDCAN_FlagTypeDef;

/**
  * @brief  FDCAN 帧.
  * @note   标识符位宽由 flags 中的 NERV_FDCAN_FLAG_EXTENDED 决定: 标准帧使用低 11 位,
  *         扩展帧使用低 29 位, 超出位宽的高位必须为 0.
  * @note   数据长度必须与帧类型一致: 经典帧 len 为 0 到 8, FDCAN 帧 len 为 0 到 64.
  * @note   data 区固定 64 字节, 有效长度由 len 指定, 超出 len 的字节内容未定义.
  * @note   作为接收回调参数时, 本结构只在回调执行期间有效, 回调不得保存该指针.
  */
typedef struct
{
    uint32_t id;                       /**< 标识符, 位宽由 flags 决定 */
    uint8_t data[NERV_FDCAN_DATA_MAX]; /**< 数据区, 有效长度由 len 指定 */
    uint8_t len;                       /**< 数据长度, 经典帧 0 到 8, FDCAN 帧 0 到 64 */
    uint8_t flags;                     /**< NERV_FDCAN_FLAG_* 位掩码 */
} NERV_FDCAN_FrameTypeDef;

/**
  * @brief  FDCAN 接收过滤器.
  * @note   过滤器按标识符匹配, 与帧格式无关. 标准帧过滤器只匹配标准帧, 扩展帧过滤器
  *         只匹配扩展帧, 由 flags 中的 NERV_FDCAN_FLAG_EXTENDED 决定.
  * @note   帧标识符与 mask 按位与等于 id 与 mask 按位与时, 该帧被接受. mask 中为 0
  *         的位不参与比较.
  */
typedef struct
{
    uint32_t id;   /**< 过滤器标识符, 标准帧使用低 11 位, 扩展帧使用低 29 位 */
    uint32_t mask; /**< 过滤器掩码, 为 0 的位表示任意值 */
    uint8_t flags; /**< NERV_FDCAN_FLAG_EXTENDED 等帧标志 */
} NERV_FDCAN_FilterTypeDef;

/**
  * @brief  FDCAN 接收回调.
  * @param  user 注册时提供的上下文指针, 由调用者保证生命周期.
  * @param  frame 接收到的帧, 只在本次调用期间有效, 回调不得保存该指针.
  * @note   由 NERV_FDCAN_OnRx 在平台适配层的接收路径中调用, 可能在中断上下文中执行.
  *         平台在中断中调用本回调时, 实现必须是 ISR-safe.
  * @note   回调不得阻塞, 不得分配内存, 不得递归调用日志或 FDCAN 接口.
  */
typedef void (*NERV_FDCAN_RxCallbackTypeDef)(void *user, const NERV_FDCAN_FrameTypeDef *frame);

/**
  * @brief  FDCAN 平台操作表.
  * @note   由平台适配层实现并绑定到实例, 绑定后不再更改. 模块层只调用通用接口,
  *         不直接调用操作表成员.
  * @note   成员契约:
  *         - init: 初始化外设与 platform_data 指向的平台数据, 失败返回 NERV_ERROR.
  *                 NOT ISR-safe.
  *         - start: 启动外设, 必须在 init 与 configure_filter 之后调用.
  *                  NOT ISR-safe.
  *         - stop: 停止外设, 停止时未完成的发送由平台处理. NOT ISR-safe.
  *         - configure_filter: 把过滤器数组转换为厂商过滤器配置, 必须在 start
  *                             之前调用. NOT ISR-safe.
  *         - transmit: 非阻塞发送, 不修改实例, self 为 const. 没有空闲发送邮箱时
  *                     返回 NERV_BUSY. 能否在中断中调用由平台适配层约定.
  */
typedef struct
{
    NERV_StatusTypeDef (*init)(NERV_FDCAN_InstanceTypeDef *self);
    NERV_StatusTypeDef (*start)(NERV_FDCAN_InstanceTypeDef *self);
    NERV_StatusTypeDef (*stop)(NERV_FDCAN_InstanceTypeDef *self);
    NERV_StatusTypeDef (*configure_filter)(NERV_FDCAN_InstanceTypeDef *self,
                                           const NERV_FDCAN_FilterTypeDef *filters, uint8_t count);
    NERV_StatusTypeDef (*transmit)(const NERV_FDCAN_InstanceTypeDef *self,
                                   const NERV_FDCAN_FrameTypeDef *frame);
} NERV_FDCAN_OpsTypeDef;

/**
  * @brief  FDCAN 实例.
  * @note   平台适配层的私有结构体以本结构体作为第一个成员, 实例指针即可在该结构体
  *         与平台私有数据之间转换.
  * @note   实例由使用者分配并清零, 生命周期必须覆盖全部调用.
  */
struct NERV_FDCAN_Instance
{
    const NERV_FDCAN_OpsTypeDef *ops;         /**< 平台操作表, 绑定后不再更改 */
    NERV_FDCAN_RxCallbackTypeDef rx_callback; /**< 接收回调, NULL 表示未注册 */
    void *rx_user;                            /**< 接收回调上下文, 可以为 NULL */
    void *platform_data;                      /**< 平台私有数据, 由平台适配层解释 */
};

/* 通用接口. 全部为静态内联, 只做参数检查与操作表分发, 行为契约见各函数文档.
 * 调用顺序: NERV_FDCAN_Init, NERV_FDCAN_ConfigureFilter, NERV_FDCAN_Start.
 * NERV_FDCAN_RegisterRxCallback 必须在 NERV_FDCAN_Start 之前调用. */

/**
  * @brief  初始化 FDCAN 实例.
  * @param  self 实例指针, 不得为 NULL. 调用前 ops 必须已绑定.
  * @retval NERV_OK 平台初始化成功.
  * @retval NERV_ERROR self, self->ops 或 self->ops->init 为 NULL, 或平台初始化失败.
  * @note   不分配内存, 不创建任务.
  * @note   NOT ISR-safe.
  * @note   必须在 NERV_FDCAN_Start 与 NERV_FDCAN_ConfigureFilter 之前调用.
  */
static inline NERV_StatusTypeDef NERV_FDCAN_Init(NERV_FDCAN_InstanceTypeDef *self)
{
    if ((NULL == self) || (NULL == self->ops) || (NULL == self->ops->init))
    {
        return NERV_ERROR;
    }

    return self->ops->init(self);
}

/**
  * @brief  启动 FDCAN 外设.
  * @param  self 实例指针, 不得为 NULL.
  * @retval NERV_OK 外设已启动.
  * @retval NERV_ERROR self, self->ops 或 self->ops->start 为 NULL, 或启动失败.
  * @note   NOT ISR-safe.
  * @note   必须在 NERV_FDCAN_Init 与 NERV_FDCAN_ConfigureFilter 之后调用.
  */
static inline NERV_StatusTypeDef NERV_FDCAN_Start(NERV_FDCAN_InstanceTypeDef *self)
{
    if ((NULL == self) || (NULL == self->ops) || (NULL == self->ops->start))
    {
        return NERV_ERROR;
    }

    return self->ops->start(self);
}

/**
  * @brief  停止 FDCAN 外设.
  * @param  self 实例指针, 不得为 NULL.
  * @retval NERV_OK 外设已停止.
  * @retval NERV_ERROR self, self->ops 或 self->ops->stop 为 NULL, 或停止失败.
  * @note   停止时未完成的发送由平台处理, 库不保证送达.
  * @note   NOT ISR-safe.
  */
static inline NERV_StatusTypeDef NERV_FDCAN_Stop(NERV_FDCAN_InstanceTypeDef *self)
{
    if ((NULL == self) || (NULL == self->ops) || (NULL == self->ops->stop))
    {
        return NERV_ERROR;
    }

    return self->ops->stop(self);
}

/**
  * @brief  配置接收过滤器.
  * @param  self 实例指针, 不得为 NULL.
  * @param  filters 过滤器数组, 不得为 NULL.
  * @param  count 过滤器个数, 必须大于 0.
  * @retval NERV_OK 过滤器已配置.
  * @retval NERV_ERROR 参数非法, 或平台配置失败.
  * @note   过滤语义由平台适配层转换为厂商配置, 模块层不接触寄存器.
  * @note   NOT ISR-safe.
  * @note   必须在 NERV_FDCAN_Init 之后, NERV_FDCAN_Start 之前调用.
  */
static inline NERV_StatusTypeDef NERV_FDCAN_ConfigureFilter(NERV_FDCAN_InstanceTypeDef *self,
                                                            const NERV_FDCAN_FilterTypeDef *filters,
                                                            uint8_t count)
{
    if ((NULL == self) || (NULL == filters))
    {
        return NERV_ERROR;
    }

    if (0U == count)
    {
        return NERV_ERROR;
    }

    if ((NULL == self->ops) || (NULL == self->ops->configure_filter))
    {
        return NERV_ERROR;
    }

    return self->ops->configure_filter(self, filters, count);
}

/**
  * @brief  发送一帧.
  * @param  self 实例指针, 不得为 NULL.
  * @param  frame 待发送的帧, 不得为 NULL. 内容在返回前读取, 返回后即可释放.
  * @retval NERV_OK 帧已交给外设发送.
  * @retval NERV_BUSY 没有空闲发送邮箱, 帧未发送, 调用方可重试.
  * @retval NERV_ERROR self 或 frame 为 NULL, 操作表未绑定, NERV_FDCAN_FLAG_BRS
  *         未与 NERV_FDCAN_FLAG_FD 同时使用, len 超出帧类型允许的长度, id 超出
  *         标志位允许的位宽, 或平台发送失败.
  * @note   非阻塞. 返回 NERV_OK 只表示帧进入发送流程, 不代表对端已接收.
  * @note   ISR-safe. 本函数只做参数检查与操作表分发, 不修改实例状态, 可在中断中调用,
  *         前提是平台适配层的 transmit 实现也 ISR-safe.
  */
static inline NERV_StatusTypeDef NERV_FDCAN_Transmit(const NERV_FDCAN_InstanceTypeDef *self,
                                                     const NERV_FDCAN_FrameTypeDef *frame)
{
    if ((NULL == self) || (NULL == frame))
    {
        return NERV_ERROR;
    }

    if ((0U != (frame->flags & NERV_FDCAN_FLAG_BRS)) && (0U == (frame->flags & NERV_FDCAN_FLAG_FD)))
    {
        return NERV_ERROR;
    }

    const uint8_t max_len = (0U != (frame->flags & NERV_FDCAN_FLAG_FD))
                                ? NERV_FDCAN_DATA_MAX
                                : NERV_FDCAN_CLASSIC_DATA_MAX;

    if (frame->len > max_len)
    {
        return NERV_ERROR;
    }

    const uint32_t id_limit = (0U != (frame->flags & NERV_FDCAN_FLAG_EXTENDED))
                                  ? NERV_FDCAN_ID_EXTENDED_MAX
                                  : NERV_FDCAN_ID_STANDARD_MAX;

    if (frame->id > id_limit)
    {
        return NERV_ERROR;
    }

    if ((NULL == self->ops) || (NULL == self->ops->transmit))
    {
        return NERV_ERROR;
    }

    return self->ops->transmit(self, frame);
}

/**
  * @brief  注册或替换接收回调.
  * @param  self 实例指针, 不得为 NULL.
  * @param  callback 接收回调. 为 NULL 时注销已注册的回调.
  * @param  user 回调上下文, 由调用者保证生命周期. 可以为 NULL.
  * @retval NERV_OK 回调与上下文已保存.
  * @retval NERV_ERROR self 为 NULL.
  * @note   NOT ISR-safe.
  * @note   注册与替换不得与接收路径并发. NERV_FDCAN_OnRx 分两次读取回调与上下文,
  *         并发替换可能把新回调与旧上下文的组合交给同一次接收.
  * @note   接收回调在中断上下文执行, 必须保持 O(1) 轻量: 只做拷贝, 置标志或入队,
  *         不得阻塞, 不得调用可能等待的外设接口, 不得做日志输出. 回调时长直接增加
  *         总线接收延迟, 可能造成后续帧溢出.
  * @note   回调在平台适配层的接收路径中执行, 可能是中断上下文. 若在中断中调用,
  *         回调实现必须 ISR-safe.
  */
static inline NERV_StatusTypeDef
NERV_FDCAN_RegisterRxCallback(NERV_FDCAN_InstanceTypeDef *self,
                              NERV_FDCAN_RxCallbackTypeDef callback, void *user)
{
    if (NULL == self)
    {
        return NERV_ERROR;
    }

    self->rx_callback = callback;
    self->rx_user     = user;

    return NERV_OK;
}

/**
  * @brief  把接收到的帧交给已注册的回调.
  * @param  self 实例指针, 不得为 NULL.
  * @param  frame 接收到的帧, 不得为 NULL.
  * @note   由平台适配层在接收中断中调用. 未注册回调时丢弃该帧.
  * @note   只读取回调与上下文并转发, 不阻塞, 不分配内存, 不调用日志.
  * @note   设计为在中断上下文中调用, 因此已注册的回调必须 ISR-safe.
  * @note   ISR-safe. 本函数自身只读取并转发回调, 不阻塞, 不分配内存, 不调用日志.
  */
static inline void NERV_FDCAN_OnRx(NERV_FDCAN_InstanceTypeDef *self,
                                   const NERV_FDCAN_FrameTypeDef *frame)
{
    if ((NULL == self) || (NULL == frame))
    {
        return;
    }

    const NERV_FDCAN_RxCallbackTypeDef callback = self->rx_callback;
    void *const user                            = self->rx_user;

    if (NULL != callback)
    {
        callback(user, frame);
    }
}

#ifdef __cplusplus
}
#endif

#endif /* NERV_FDCAN_H */

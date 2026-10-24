/**
  * @file    nerv_motor.c
  * @brief   NERV 电机模块实现.
  * @version 1.0.0
  * @date    2026-10-23
  * @copyright Copyright (c) 2026 NERV Contributors
  *
  * SPDX-License-Identifier: MIT
  */

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "config.h"
#include "nerv_math.h"
#include "nerv_motor.h"
#include "nerv_motor_priv.h"
#include "nerv_status.h"
#include "nerv_utils.h"

#if (NERV_USE_MOTOR == NERV_ENABLE)

/**
  * @brief  用配置建立电机实例: 校验配置, 核对设备 id, 建立运行期状态.
  * @param  self 实例指针, 不得为 NULL.
  * @param  config 配置指针, 不得为 NULL. 内容被复制, 调用方保留所有权.
  * @retval NERV_OK 配置已绑定, 帧几何信息已缓存, 实例进入就绪状态.
  * @retval NERV_ERROR self 或 config 为 NULL, config 的 driver, driver 的 model,
  *         driver 的 ops, ops 的 get_frame_geo, encode_tx, decode_fb 或 bus 为 NULL,
  *         gear_ratio 不为大于 0 的有限值, over_temperature_c 与 control_limit 为负
  *         或非有限值, device_id 超出描述符的 id_min 到 id_max 范围, 或帧几何信息
  *         的帧长为 0 或超过 NERV_FDCAN_DATA_MAX, 标识符超过 11 位.
  * @note   不直接访问外设: 电调无需初始化指令, 帧几何信息只经驱动操作表派生.
  *         调用点仍应按 FDCAN 启动之后安排.
  * @note   校验与几何派生全部在修改 self 之前完成, 失败时 self 保持原状.
  * @note   重复调用以新配置重建实例, 复位错误位, 接收计时, 反馈与目标.
  * @note   NOT ISR-safe.
  * @note   caller-synchronized.
  */
NERV_StatusTypeDef NERV_Motor_Init(NERV_Motor_InstanceTypeDef *self,
                                   const NERV_Motor_ConfigTypeDef *config)
{
    NERV_Motor_FrameGeoTypeDef geo;

    if ((NULL == self) || (NULL == config))
    {
        return NERV_ERROR;
    }

    if ((NULL == config->driver) || (NULL == config->driver->model) ||
        (NULL == config->driver->ops) || (NULL == config->driver->ops->get_frame_geo) ||
        (NULL == config->driver->ops->encode_tx) || (NULL == config->driver->ops->decode_fb) ||
        (NULL == config->bus))
    {
        return NERV_ERROR;
    }

    if (!isfinite(config->gear_ratio) || (config->gear_ratio <= 0.0f))
    {
        return NERV_ERROR;
    }

    if (!isfinite(config->over_temperature_c) || (config->over_temperature_c < 0.0f))
    {
        return NERV_ERROR;
    }

    if (!isfinite(config->control_limit) || (config->control_limit < 0.0f))
    {
        return NERV_ERROR;
    }

    if (NERV_OK !=
        config->driver->ops->get_frame_geo(config->driver->model, config->device_id, &geo))
    {
        return NERV_ERROR;
    }

    if ((0U == geo.len) || (geo.len > NERV_FDCAN_DATA_MAX) ||
        (geo.tx_id > NERV_FDCAN_ID_STANDARD_MAX) || (geo.rx_id > NERV_FDCAN_ID_STANDARD_MAX))
    {
        return NERV_ERROR;
    }

    self->config = *config;

    NERV_Motor_ResetState(self);

    self->geo         = geo;
    self->initialized = true;

    return NERV_OK;
}

/**
  * @brief  设置控制量目标.
  * @param  self 实例指针, 不得为 NULL.
  * @param  target 控制量目标, 单位与语义由型号描述符约定, 必须为有限值.
  * @retval NERV_OK 目标已保存.
  * @retval NERV_ERROR self 为 NULL, 实例未初始化, 或 target 为非有限值.
  * @note   本函数不限幅. 实例限幅与协议满量程限幅在发送时进行.
  * @note   NOT ISR-safe.
  * @note   caller-synchronized.
  */
NERV_StatusTypeDef NERV_Motor_SetTarget(NERV_Motor_InstanceTypeDef *self, float target)
{
    if (NULL == self)
    {
        return NERV_ERROR;
    }

    if (!self->initialized)
    {
        return NERV_ERROR;
    }

    if (!isfinite(target))
    {
        return NERV_ERROR;
    }

    self->target = target;

    return NERV_OK;
}

/**
  * @brief  刷新机构系测量值并执行离线与过温监督, 不发送指令.
  * @param  self 实例指针, 不得为 NULL.
  * @param  now_ms 当前时刻, 单位 ms. 由调用方提供, 库不读取硬件时钟.
  * @retval NERV_OK 状态已刷新, 监督已执行.
  * @retval NERV_ERROR self 为 NULL 或实例未初始化.
  * @note   监督结果写入错误位, 不通过返回值报告. 错误位为粘滞, 只由
  *         NERV_Motor_ClearError 清除.
  * @note   指令发送由 NERV_Motor_UpdateGroup 完成.
  * @note   NOT ISR-safe.
  * @note   caller-synchronized.
  */
NERV_StatusTypeDef NERV_Motor_Update(NERV_Motor_InstanceTypeDef *self, uint32_t now_ms)
{
    if (NULL == self)
    {
        return NERV_ERROR;
    }

    if (!self->initialized)
    {
        return NERV_ERROR;
    }

    if (NERV_OK != NERV_Motor_ConvertFb(&self->config, &self->fb, &self->measurement))
    {
        return NERV_ERROR;
    }

    NERV_Motor_Supervise(self, now_ms);

    return NERV_OK;
}

/**
  * @brief  更新一组电机: 逐成员刷新状态与监督, 按总线与指令帧标识符分组发送.
  * @param  members 实例指针数组, 不得为 NULL. 每个元素不得为 NULL 且必须已初始化.
  * @param  count 数组元素个数, 必须大于 0.
  * @param  now_ms 当前时刻, 单位 ms.
  * @retval NERV_OK 全部指令帧已交给 FDCAN 发送.
  * @retval NERV_BUSY 没有空闲发送邮箱, 本周期后续帧不再发送, 下周期按当前目标重建.
  * @retval NERV_ERROR members 为 NULL, count 为 0, 数组含 NULL 元素, 成员未初始化,
  *         同一总线与指令帧标识符的成员帧长不一致, 同一总线与指令帧标识符内设备 id
  *         重复, 或编码失败.
  * @note   参数与分组一致性错误在刷新与发送之前返回, 不会产生部分发送. 发送阶段的
  *         错误不回滚已处理成员的测量值刷新与已发送的指令帧.
  * @note   分组键为总线实例与指令帧标识符. 同一组每周期只发送一次, 帧内只包含该组
  *         成员的槽位, 其余槽位为 0. 同一指令帧标识符位于不同总线时分别发送.
  * @note   帧长取该组首个成员的值, 同一组的成员必须帧长一致.
  * @note   同一组内设备 id 必须唯一, 否则两个成员编码到同一槽位互相覆盖.
  * @note   NOT ISR-safe.
  * @note   caller-synchronized.
  */
NERV_StatusTypeDef NERV_Motor_UpdateGroup(NERV_Motor_InstanceTypeDef *const *members, uint8_t count,
                                          uint32_t now_ms)
{
    NERV_StatusTypeDef status;
    uint8_t first;
    uint8_t index;
    bool first_of_group;

    if ((NULL == members) || (0U == count))
    {
        return NERV_ERROR;
    }

    for (index = 0U; index < count; index++)
    {
        if ((NULL == members[index]) || (!members[index]->initialized))
        {
            return NERV_ERROR;
        }
    }

    if (NERV_OK != NERV_Motor_ValidateGroups(members, count))
    {
        return NERV_ERROR;
    }

    for (index = 0U; index < count; index++)
    {
        if (NERV_OK != NERV_Motor_Update(members[index], now_ms))
        {
            return NERV_ERROR;
        }
    }

    for (first = 0U; first < count; first++)
    {
        first_of_group = true;

        for (index = 0U; index < first; index++)
        {
            if (NERV_Motor_SameGroup(members[index], members[first]))
            {
                first_of_group = false;
                break;
            }
        }

        if (first_of_group)
        {
            status = NERV_Motor_TransmitGroupFrame(members, count, first);

            if (NERV_OK != status)
            {
                return status;
            }
        }
    }

    return NERV_OK;
}

/**
  * @brief  接收反馈帧入口, 由应用的总线接收回调调用.
  * @param  self 实例指针, 可以为 NULL.
  * @param  frame 接收到的帧, 可以为 NULL.
  * @note   帧标识符与实例缓存的反馈标识符不一致时忽略该帧. 扩展帧与标准帧的标识符
  *         位宽不同, 即使低位相同也不匹配, 扩展帧一律忽略. 解码失败时丢弃, 不更新
  *         反馈, 不置位 fb_fresh.
  * @note   只做解码, 存储与置位 fb_fresh, 不读取时钟, 不维护错误位.
  * @note   ISR-safe.
  */
void NERV_Motor_OnRx(NERV_Motor_InstanceTypeDef *self, const NERV_FDCAN_FrameTypeDef *frame)
{
    if ((NULL == self) || (NULL == frame))
    {
        return;
    }

    if (!self->initialized)
    {
        return;
    }

    if (0U != (((uint32_t)frame->flags) & (uint32_t)NERV_FDCAN_FLAG_EXTENDED))
    {
        return;
    }

    if (frame->id != self->geo.rx_id)
    {
        return;
    }

    if (NERV_OK != self->config.driver->ops->decode_fb(self->config.driver->model, frame->data,
                                                       frame->len, &self->fb))
    {
        return;
    }

    self->fb_fresh = true;
}

/**
  * @brief  读取机构系测量值.
  * @param  self 实例指针, 不得为 NULL.
  * @param  measurement 输出测量值, 不得为 NULL. 失败时不修改.
  * @retval NERV_OK 测量值已写入 measurement.
  * @retval NERV_ERROR self 或 measurement 为 NULL, 或实例未初始化.
  * @note   测量值由 NERV_Motor_Update 或 NERV_Motor_UpdateGroup 刷新, 读取发生在
  *         任务上下文. 收到第一次反馈之前为初始零值.
  * @note   NOT ISR-safe.
  * @note   caller-synchronized.
  */
NERV_StatusTypeDef NERV_Motor_GetMeasurement(const NERV_Motor_InstanceTypeDef *self,
                                             NERV_Motor_MeasurementTypeDef *measurement)
{
    if ((NULL == self) || (NULL == measurement))
    {
        return NERV_ERROR;
    }

    if (!self->initialized)
    {
        return NERV_ERROR;
    }

    *measurement = self->measurement;

    return NERV_OK;
}

/**
  * @brief  读取电机错误位.
  * @param  self 实例指针, 可以为 NULL.
  * @return 粘滞错误位掩码. 高 8 位为电机域号, 低 24 位可取 NERV_ERR_MOTOR_OFFLINE
  *         与 NERV_ERR_MOTOR_OVER_TEMP. self 为 NULL 时返回 NERV_ERR_NONE.
  * @note   读取不清除错误位. 错误位只在任务上下文维护, 无需临界区.
  * @note   NOT ISR-safe.
  * @note   caller-synchronized.
  */
uint32_t NERV_Motor_GetError(const NERV_Motor_InstanceTypeDef *self)
{
    uint32_t bits;
    uint32_t domain;

    if (NULL == self)
    {
        return NERV_ERR_NONE;
    }

    bits   = self->error & NERV_ERR_BITS_MASK;
    domain = 0U;

    if (0U != (bits & NERV_ERR_MOTOR_BITS_MASK))
    {
        domain |= NERV_ERR_DOMAIN_MOTOR;
    }

    return NERV_ERR_DOMAIN(domain) | bits;
}

/**
  * @brief  清除电机错误位.
  * @param  self 实例指针, 可以为 NULL.
  * @param  mask 待清除的错误位掩码, 只使用低 24 位, 高 8 位域号被忽略.
  * @note   只清除 mask 中置位的错误位. self 为 NULL 时为空操作.
  * @note   NOT ISR-safe.
  * @note   caller-synchronized.
  */
void NERV_Motor_ClearError(NERV_Motor_InstanceTypeDef *self, uint32_t mask)
{
    if (NULL == self)
    {
        return;
    }

    self->error &= ~(mask & NERV_ERR_BITS_MASK);
}

/**
  * @brief  进入安全状态: 清零控制量目标并复位离线监督计时.
  * @param  self 实例指针, 不得为 NULL.
  * @retval NERV_OK 目标已清零.
  * @retval NERV_ERROR self 为 NULL 或实例未初始化.
  * @note   控制输出由目标派生, 本函数不保存独立输出值. 零指令随下一个
  *         NERV_Motor_UpdateGroup 发送, 之间电调保持最后收到的指令. 需要立即输出时,
  *         调用本函数后立即执行一次 NERV_Motor_UpdateGroup.
  * @note   本函数不直接切断电调供电, 也不依赖总线发送成功. 离线监督在下一个
  *         NERV_Motor_Update 或 NERV_Motor_UpdateGroup 时重新建立计时起点.
  * @note   NOT ISR-safe.
  * @note   caller-synchronized.
  */
NERV_StatusTypeDef NERV_Motor_EnterSafeState(NERV_Motor_InstanceTypeDef *self)
{
    if (NULL == self)
    {
        return NERV_ERROR;
    }

    if (!self->initialized)
    {
        return NERV_ERROR;
    }

    self->target        = 0.0f;
    self->offline_armed = false;

    return NERV_OK;
}

/**
  * @brief  复位实例的软件状态, 不清除配置.
  * @param  self 实例指针, 由调用方保证不为 NULL.
  */
static void NERV_Motor_ResetState(NERV_Motor_InstanceTypeDef *self)
{
    self->fb.angle_rad              = 0.0f;
    self->fb.speed_radps            = 0.0f;
    self->fb.current_a              = 0.0f;
    self->fb.temperature_c          = 0;
    self->measurement.angle_rad     = 0.0f;
    self->measurement.speed_radps   = 0.0f;
    self->measurement.control       = 0.0f;
    self->measurement.temperature_c = 0;
    self->geo.tx_id                 = 0U;
    self->geo.rx_id                 = 0U;
    self->geo.len                   = 0U;
    self->error                     = 0U;
    self->target                    = 0.0f;
    self->last_rx_ms                = 0U;
    self->offline_armed             = false;
    self->initialized               = false;
    self->fb_fresh                  = false;
}

/**
  * @brief  给出安装方向对应的符号.
  * @param  config 实例配置, 由调用方保证不为 NULL.
  * @return invert_direction 为 true 时返回 -1.0f, 否则返回 1.0f.
  */
static float NERV_Motor_DirectionSign(const NERV_Motor_ConfigTypeDef *config)
{
    return config->invert_direction ? __NERV_MOTOR_DIRECTION_SIGN_INVERTED : 1.0f;
}

/**
  * @brief  把转子量反馈换算为机构系测量值.
  * @param  config 实例配置, 必须已通过 NERV_Motor_Init 校验, 不得为 NULL.
  * @param  fb 转子量反馈, 不得为 NULL.
  * @param  measurement 输出机构系测量值, 不得为 NULL.
  * @retval NERV_OK 换算完成, measurement 已写入.
  * @retval NERV_ERROR 参数为 NULL.
  * @note   角度与角速度除以 gear_ratio, 控制量透传, 全部乘安装方向符号. 角度结果
  *         归回 [0, 2π).
  */
static NERV_StatusTypeDef NERV_Motor_ConvertFb(const NERV_Motor_ConfigTypeDef *config,
                                               const NERV_Motor_FbTypeDef *fb,
                                               NERV_Motor_MeasurementTypeDef *measurement)
{
    float sign;

    if ((NULL == config) || (NULL == fb) || (NULL == measurement))
    {
        return NERV_ERROR;
    }

    sign = NERV_Motor_DirectionSign(config);

    measurement->angle_rad =
        NERV_Math_Wrap(fb->angle_rad / config->gear_ratio * sign, 0.0f, NERV_MATH_TWO_PI);
    measurement->speed_radps   = fb->speed_radps / config->gear_ratio * sign;
    measurement->control       = fb->current_a * sign;
    measurement->temperature_c = fb->temperature_c;

    return NERV_OK;
}

/**
  * @brief  给出本周期应发送的控制量.
  * @param  self 实例指针, 必须已初始化, 由调用方保证不为 NULL.
  * @return 乘安装方向符号并按 control_limit 实例限幅后的控制量.
  * @note   control_limit 为 0 时跳过实例限幅, 协议满量程限幅由驱动完成.
  */
static float NERV_Motor_EffectiveCommand(const NERV_Motor_InstanceTypeDef *self)
{
    float command;

    command = self->target * NERV_Motor_DirectionSign(&self->config);

    if (self->config.control_limit > 0.0f)
    {
        command = NERV_Math_Clamp(command, -self->config.control_limit, self->config.control_limit);
    }

    return command;
}

/**
  * @brief  把本实例的控制量编码到控制帧内该实例对应的位置.
  * @param  self 实例指针, 必须已初始化, 由调用方保证不为 NULL.
  * @param  frame 控制帧数据缓冲, 长度不小于实例的 frame_len, 调用方负责清零.
  * @retval NERV_OK 编码成功.
  * @retval NERV_ERROR 驱动编码失败.
  */
static NERV_StatusTypeDef NERV_Motor_EncodeInto(const NERV_Motor_InstanceTypeDef *self,
                                                uint8_t *frame)
{
    return self->config.driver->ops->encode_tx(self->config.driver->model, frame,
                                               self->config.device_id,
                                               NERV_Motor_EffectiveCommand(self));
}

/**
  * @brief  维护离线与过温错误位.
  * @param  self 实例指针, 必须已初始化, 由调用方保证不为 NULL.
  * @param  now_ms 当前时刻, 单位 ms.
  * @note   fb_fresh 置位时刷新 last_rx_ms 并清除该标志. 首次监督以 now_ms 建立计时
  *         起点, 避免实例初始化后尚未收到反馈就立即判定离线. 之后按无符号回绕减法
  *         计算间隔, 达到 NERV_UTILS_TIMEOUT_MS(offline_ms) 时置离线位. offline_ms
  *         为 0 时跳过离线判定.
  * @note   过温判定以 over_temperature_c 大于 0 为前提, 读 measurement 的温度字段,
  *         因此本函数必须在 NERV_Motor_ConvertFb 之后调用.
  * @note   错误位为粘滞, 只由 NERV_Motor_ClearError 清除.
  */
static void NERV_Motor_Supervise(NERV_Motor_InstanceTypeDef *self, uint32_t now_ms)
{
    if (self->fb_fresh)
    {
        self->fb_fresh      = false;
        self->last_rx_ms    = now_ms;
        self->offline_armed = true;
    }
    else if (!self->offline_armed)
    {
        self->last_rx_ms    = now_ms;
        self->offline_armed = true;
    }
    else if ((self->config.offline_ms > 0U) &&
             ((now_ms - self->last_rx_ms) >= NERV_UTILS_TIMEOUT_MS(self->config.offline_ms)))
    {
        self->error |= (NERV_ERR_MOTOR_OFFLINE & NERV_ERR_BITS_MASK);
    }

    if ((self->config.over_temperature_c > 0.0f) &&
        ((float)self->measurement.temperature_c >= self->config.over_temperature_c))
    {
        self->error |= (NERV_ERR_MOTOR_OVER_TEMP & NERV_ERR_BITS_MASK);
    }
}

/**
  * @brief  判断两个实例是否属于同一发送组.
  * @param  lhs 第一个实例, 不得为 NULL.
  * @param  rhs 第二个实例, 不得为 NULL.
  * @return 总线实例与指令帧标识符均相同时返回 true, 否则返回 false.
  * @note   同一标识符的控制帧只能由一条总线发出, 因此总线也是分组键.
  */
static bool NERV_Motor_SameGroup(const NERV_Motor_InstanceTypeDef *lhs,
                                 const NERV_Motor_InstanceTypeDef *rhs)
{
    return (lhs->config.bus == rhs->config.bus) && (lhs->geo.tx_id == rhs->geo.tx_id);
}

/**
  * @brief  校验同一发送组内成员的帧长与设备 id.
  * @param  members 实例指针数组, 元素由调用方保证不为 NULL.
  * @param  count 数组元素个数.
  * @retval NERV_OK 组内一致.
  * @retval NERV_ERROR 同一总线与指令帧标识符的成员帧长不一致或设备 id 重复.
  * @note   不同组之间允许帧长与设备 id 相同.
  */
static NERV_StatusTypeDef NERV_Motor_ValidateGroups(NERV_Motor_InstanceTypeDef *const *members,
                                                    uint8_t count)
{
    uint8_t first;
    uint8_t second;

    for (first = 0U; first < count; first++)
    {
        for (second = (uint8_t)(first + 1U); second < count; second++)
        {
            if (!NERV_Motor_SameGroup(members[first], members[second]))
            {
                continue;
            }

            if (members[first]->geo.len != members[second]->geo.len)
            {
                return NERV_ERROR;
            }

            if (members[first]->config.device_id == members[second]->config.device_id)
            {
                return NERV_ERROR;
            }
        }
    }

    return NERV_OK;
}

/**
  * @brief  为一个发送组组帧并发送.
  * @param  members 实例指针数组, 元素由调用方保证不为 NULL 且已初始化.
  * @param  count 数组元素个数.
  * @param  first 该组首个成员在数组中的下标.
  * @retval NERV_OK 帧已交给 FDCAN 发送.
  * @retval NERV_BUSY 没有空闲发送邮箱.
  * @retval NERV_ERROR 帧长超过 NERV_FDCAN_DATA_MAX 或编码失败.
  * @note   帧标识符与帧长取 members[first] 的值, 数据区按帧长清零, 只编码同一总线
  *         与标识符成员的槽位, 其余槽位为 0. 经 members[first] 的 bus 发送.
  */
static NERV_StatusTypeDef NERV_Motor_TransmitGroupFrame(NERV_Motor_InstanceTypeDef *const *members,
                                                        uint8_t count, uint8_t first)
{
    NERV_FDCAN_FrameTypeDef frame;
    uint8_t index;
    uint8_t slot;

    if (members[first]->geo.len > NERV_FDCAN_DATA_MAX)
    {
        return NERV_ERROR;
    }

    frame.id    = members[first]->geo.tx_id;
    frame.len   = members[first]->geo.len;
    frame.flags = (uint8_t)NERV_FDCAN_FLAG_NONE;

    for (slot = 0U; slot < frame.len; slot++)
    {
        frame.data[slot] = 0U;
    }

    for (index = 0U; index < count; index++)
    {
        if (!NERV_Motor_SameGroup(members[index], members[first]))
        {
            continue;
        }

        if (NERV_OK != NERV_Motor_EncodeInto(members[index], frame.data))
        {
            return NERV_ERROR;
        }
    }

    return NERV_FDCAN_Transmit(members[first]->config.bus, &frame);
}

#endif /* NERV_USE_MOTOR */

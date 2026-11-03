/**
  * @file    nerv_gimbal.h
  * @brief   NERV 云台模块公开接口.
  * @version 1.0.0
  * @date    2026-11-01
  * @copyright Copyright (c) 2026 NERV Contributors
  *
  * SPDX-License-Identifier: MIT
  */

#ifndef NERV_GIMBAL_H
#define NERV_GIMBAL_H

#include <stdbool.h>
#include <stdint.h>

#include "config.h"

#if (NERV_USE_GIMBAL == NERV_ENABLE)

#include "nerv_motor.h"
#include "nerv_pid.h"
#include "nerv_status.h"

/* 轴索引. 四轴结构按大 yaw, 小 yaw, pitch, roll 依次串联.
 * 两个 yaw 轴的轴线平行且都竖直, 安装位置不同, 总偏航角等于两者之和.
 * 索引直接用作电机数组下标, 顺序不得改动. */

/** @brief 大 yaw 轴索引, 安装在底盘与云台之间, 轴线竖直. */
#define NERV_GIMBAL_AXIS_YAW_BIG 0U

/** @brief 小 yaw 轴索引, 轴线与大 yaw 平行, 安装位置不同. */
#define NERV_GIMBAL_AXIS_YAW_SMALL 1U

/** @brief pitch 轴索引, 轴线水平, 抬头为正. */
#define NERV_GIMBAL_AXIS_PITCH 2U

/** @brief roll 轴索引, 轴线水平, 右侧下沉为正. */
#define NERV_GIMBAL_AXIS_ROLL 3U

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief  云台目标.
  * @note   所有角度与角速度均为输出轴量, 单位 rad 与 rad/s. 电机实例配置里的
  *         feedback_ratio 负责与反馈侧换算, 本模块的调用方不需要处理该换算.
  * @note   大 yaw 承担整台车朝向的粗对准, 以角速度目标驱动; 小 yaw 承担相对大
  *         yaw 的精瞄, 以角度目标驱动. 末端总偏航等于两者之和.
  */
typedef struct NERV_Gimbal_Target
{
    float yaw_big_speed_radps; /**< 大 yaw 目标角速度, 单位 rad/s, 逆时针为正 */
    float yaw_small_rad;       /**< 小 yaw 目标角, 单位 rad, 相对大 yaw, 范围 [-π, π) */
    float pitch_rad;           /**< pitch 目标角, 单位 rad, 抬头为正 */
} NERV_Gimbal_TargetTypeDef;

/**
  * @brief  云台当前姿态.
  */
typedef struct NERV_Gimbal_Attitude
{
    float yaw_big_rad;   /**< 大 yaw 当前角, 单位 rad */
    float yaw_small_rad; /**< 小 yaw 当前角, 单位 rad */
    float pitch_rad;     /**< pitch 当前角, 单位 rad */
    float roll_rad;      /**< roll 当前角, 单位 rad, 多圈累计, 不参与定位 */
    float yaw_total_rad; /**< 总偏航, 单位 rad, 等于大 yaw 与小 yaw 之和, 已归一到 [-π, π) */
} NERV_Gimbal_AttitudeTypeDef;

/**
  * @brief  roll 轴寻位状态.
  * @note   roll 轴的型号必须为标量电流模式. 寻位阶段速度环按给定角速度驱动电机, 电机
  *         抵住机械限位后转速无法跟随, 速度环积分饱和使输出电流上升到限幅, 电流与转速
  *         同时满足阈值并持续配置的时间即判定到位, 随后切换到位置环保持到位时的角度.
  *         两侧限位各调用一次 NERV_Gimbal_StartRollSeek, 传入相反的符号.
  */
typedef enum
{
    NERV_GIMBAL_ROLL_IDLE    = 0U, /**< 未寻位, roll 输出零电流(零转矩) */
    NERV_GIMBAL_ROLL_SEEKING = 1U, /**< 正在朝目标方向转, 尚未检测到堵转 */
    NERV_GIMBAL_ROLL_HOLDING = 2U, /**< 已到位, 位置环保持到位时的角度 */
} NERV_Gimbal_RollStateTypeDef;

/**
  * @brief  故障回调.
  * @param  user 注册时传入的上下文指针, 可以为 NULL.
  * @param  error 最新的错误位掩码, 与 NERV_Gimbal_ReadError 同值.
  * @note   只在本次新增错误位时调用, 同一组错误持续存在不会重复通知.
  *         回调在 NERV_Gimbal_UpdateGroup 末尾执行, 属于任务上下文, 不得阻塞,
  *         不得递归调用云台接口.
  * @note   注册后于下一次 NERV_Gimbal_UpdateGroup 起生效, 注销后立即失效.
  */
typedef void (*NERV_Gimbal_FaultCallbackTypeDef)(void *user, uint32_t error);

/**
  * @brief  云台配置.
  * @note   角度限位按输出轴给出, 单位为 rad. 限位只约束目标, 不约束电机自带的
  *         机械限位, 后者由电调或结构保证.
  */
typedef struct NERV_Gimbal_Config
{
    NERV_Motor_InstanceTypeDef
        *motors[NERV_GIMBAL_AXIS_COUNT]; /**< 各轴电机, 按轴索引排列. 指向的实例必须已初始化,
                                             生命周期覆盖云台实例, 且四个指针互不相同 */

    float limit_min_rad
        [NERV_GIMBAL_AXIS_COUNT]; /**< 各轴角度下限, 单位 rad, 仅当同下标 limit_enabled 为 true 时生效 */
    float limit_max_rad
        [NERV_GIMBAL_AXIS_COUNT]; /**< 各轴角度上限, 单位 rad, 仅当同下标 limit_enabled 为 true 时生效 */
    bool limit_enabled
        [NERV_GIMBAL_AXIS_COUNT]; /**< 各轴限位开关. true 表示启用限位, false 表示无机械限位.
                                                        memset 清零即为无限制, 此时 limit_min_rad/limit_max_rad 可填任意值 */

    float yaw_small_speed_limit_radps; /**< 小 yaw 限速, 单位 rad/s, 0 表示使用型号最大速度 */
    float pitch_speed_limit_radps;     /**< pitch 限速, 单位 rad/s, 0 表示使用型号最大速度 */

    const NERV_PID_ConfigTypeDef
        *roll_speed_pid; /**< roll 寻位速度环 PID 配置指针, 输入角速度误差, 输出标量
                              控制量, 单位由型号描述符的 scalar_quantity 决定.
                              生命周期须覆盖实例, 不得为 NULL */
    const NERV_PID_ConfigTypeDef
        *roll_position_pid; /**< roll 到位后位置环 PID 配置指针, 输入角度误差, 输出标量
                                 控制量, 单位由型号描述符的 scalar_quantity 决定.
                                 生命周期须覆盖实例, 不得为 NULL */

    float roll_seek_speed_radps; /**< roll 默认寻位角速度, 单位 rad/s, 必须大于 0 */

    float roll_stall_torque_n_m;  /**< roll 堵转判定输出轴转矩阈值, 单位 N·m, 必须大于 0 */
    float roll_stall_speed_radps; /**< roll 堵转判定速度阈值, 单位 rad/s, 必须大于 0.
                                       角速度绝对值低于本值与电流超过阈值同时成立才算堵转 */
    uint32_t roll_stall_ms;       /**< 堵转持续时间阈值, 单位 ms, 必须大于 0 */
} NERV_Gimbal_ConfigTypeDef;

/**
  * @brief  云台实例.
  * @note   由调用方以静态存储提供并清零, 生命周期覆盖全部调用.
  * @note   config 之后的字段由模块维护, 调用方只读.
  */
typedef struct NERV_Gimbal_Instance
{
    NERV_Gimbal_ConfigTypeDef config; /**< 配置, 由 NERV_Gimbal_Init 绑定, 之后只读 */

    NERV_Gimbal_TargetTypeDef target_attitude;       /**< 目标, 由 NERV_Gimbal_SetTarget 写入 */
    NERV_Gimbal_AttitudeTypeDef measured_attitude;   /**< 当前姿态, 由 UpdateGroup 刷新 */
    uint32_t error;                                  /**< 粘滞错误位 */
    NERV_Gimbal_FaultCallbackTypeDef fault_callback; /**< 故障回调, NULL 表示未注册 */
    void *fault_user;                                /**< 故障回调上下文, 可以为 NULL */
    uint32_t last_update_ms;                         /**< 上次 UpdateGroup 的时刻, 单位 ms */
    NERV_PID_InstanceTypeDef roll_speed_pid;         /**< roll 寻位速度环实例 */
    NERV_PID_InstanceTypeDef roll_position_pid;      /**< roll 到位后位置环实例 */
    NERV_Gimbal_RollStateTypeDef roll_state;         /**< roll 寻位状态 */
    uint32_t roll_stall_since_ms;                    /**< 本次堵转条件成立的起始时刻, 单位 ms */
    float roll_hold_angle_rad;    /**< 到位时记录的角度, 作为位置环目标, 单位 rad */
    float roll_seek_active_radps; /**< 本次寻位的角速度, 单位 rad/s, 带方向 */
    bool roll_stall_timing;       /**< 堵转计时是否有效 */
    bool target_valid;            /**< SetTarget 是否已设置过有效目标 */
    bool initialized;             /**< 初始化完成标志 */
} NERV_Gimbal_InstanceTypeDef;

NERV_StatusTypeDef NERV_Gimbal_Init(NERV_Gimbal_InstanceTypeDef *self,
                                    const NERV_Gimbal_ConfigTypeDef *config);

NERV_StatusTypeDef NERV_Gimbal_SetTarget(NERV_Gimbal_InstanceTypeDef *self,
                                         const NERV_Gimbal_TargetTypeDef *target);

NERV_StatusTypeDef NERV_Gimbal_UpdateGroup(NERV_Gimbal_InstanceTypeDef *self, uint32_t now_ms);

NERV_StatusTypeDef NERV_Gimbal_GetAttitude(const NERV_Gimbal_InstanceTypeDef *self,
                                           NERV_Gimbal_AttitudeTypeDef *attitude);

NERV_StatusTypeDef NERV_Gimbal_GetRollState(const NERV_Gimbal_InstanceTypeDef *self,
                                            NERV_Gimbal_RollStateTypeDef *state);

NERV_StatusTypeDef NERV_Gimbal_StartRollSeek(NERV_Gimbal_InstanceTypeDef *self, float speed_radps);

uint32_t NERV_Gimbal_ReadError(const NERV_Gimbal_InstanceTypeDef *self);

NERV_StatusTypeDef NERV_Gimbal_RegisterFaultCallback(NERV_Gimbal_InstanceTypeDef *self,
                                                     NERV_Gimbal_FaultCallbackTypeDef callback,
                                                     void *user);

void NERV_Gimbal_ClearError(NERV_Gimbal_InstanceTypeDef *self, uint32_t mask);

NERV_StatusTypeDef NERV_Gimbal_EnterSafeState(NERV_Gimbal_InstanceTypeDef *self, uint32_t now_ms);

NERV_StatusTypeDef NERV_Gimbal_Enable(NERV_Gimbal_InstanceTypeDef *self);

NERV_StatusTypeDef NERV_Gimbal_Disable(NERV_Gimbal_InstanceTypeDef *self);

#ifdef __cplusplus
}
#endif

#endif /* NERV_USE_GIMBAL */

#endif /* NERV_GIMBAL_H */

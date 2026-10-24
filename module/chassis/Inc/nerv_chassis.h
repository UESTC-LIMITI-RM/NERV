/**
  * @file    nerv_chassis.h
  * @brief   NERV 底盘模块公开接口.
  * @version 1.0.0
  * @date    2026-10-24
  * @copyright Copyright (c) 2026 NERV Contributors
  *
  * SPDX-License-Identifier: MIT
  */

#ifndef NERV_CHASSIS_H
#define NERV_CHASSIS_H

#include <stdbool.h>
#include <stdint.h>

#include "config.h"

#if (NERV_USE_CHASSIS == NERV_ENABLE)

#include "nerv_chassis_def.h"
#include "nerv_motor.h"
#include "nerv_status.h"
#include "nerv_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief  底盘构型枚举镜像.
  * @note   成员数值必须与 NERV_CHASSIS_TYPE_MECANUM, NERV_CHASSIS_TYPE_OMNI 与
  *         NERV_CHASSIS_TYPE_STEERING 一致, 用于运行期类型检查与调试器可视化.
  * @note   只在 NERV_USE_CHASSIS 为 NERV_ENABLE 时进行数值校验.
  */
typedef enum
{
    NERV_CHASSIS_TYPE_MECANUM_RT  = 1U, /**< 麦克纳姆轮底盘 */
    NERV_CHASSIS_TYPE_OMNI_RT     = 2U, /**< 全向轮底盘 */
    NERV_CHASSIS_TYPE_STEERING_RT = 3U, /**< 舵轮底盘 */
} NERV_Chassis_KindTypeDef;

/**
  * @brief  机体系速度.
  * @note   底盘接口只接受机体系速度. 遥控器给出的云台系指令由调用方转换, 转换用
  *         NERV_Math_RotateZ 绕 z 轴旋转, 角度为云台航向减底盘航向, 两个航向必须在
  *         同一参考系下测量. 绕 z 轴的角速度不随参考系改变, 转换时原样传递.
  */
typedef struct NERV_Chassis_Velocity
{
    float vx_mps;   /**< 车体系 x 轴线速度, 单位 m/s, 向前为正 */
    float vy_mps;   /**< 车体系 y 轴线速度, 单位 m/s, 向左为正 */
    float wz_radps; /**< 车体系 z 轴角速度, 单位 rad/s, 逆时针为正 */
} NERV_Chassis_VelocityTypeDef;

/**
  * @brief  里程计.
  * @note   位置在参考系中累积, 原点与轴向由调用方定义, 库不定义.
  * @note   底盘只用姿态的 yaw_rad 表达航向, pitch_rad 与 roll_rad 恒为 0.
  */
typedef struct NERV_Chassis_Odom
{
    float x_m;                     /**< 参考系 x 位置, 单位 m, 向前为正 */
    float y_m;                     /**< 参考系 y 位置, 单位 m, 向左为正 */
    NERV_AttitudeTypeDef attitude; /**< 姿态, yaw_rad 为相对参考系的航向 */
} NERV_Chassis_OdomTypeDef;

/**
  * @brief  机体系牵引力与转矩.
  * @note   由 NERV_Chassis_SetFeedforward 注入, 作为前馈叠加到速度环输出的牵引力
  *         与转矩上. 库不估计坡度与阻力, 前馈量由调用方给出.
  */
typedef struct NERV_Chassis_Force
{
    float fx_n;   /**< 车体系 x 轴牵引力, 单位 N, 向前为正 */
    float fy_n;   /**< 车体系 y 轴牵引力, 单位 N, 向左为正 */
    float mz_n_m; /**< 车体系 z 轴转矩, 单位 N·m, 逆时针为正 */
} NERV_Chassis_ForceTypeDef;

/**
  * @brief  底盘故障回调.
  * @param  user 注册时提供的上下文指针, 由调用者保证生命周期.
  * @param  error 当前的粘滞错误位掩码, 取值同 NERV_Chassis_GetError 的返回值.
  * @note   由 NERV_Chassis_Update 在任务上下文调用. 回调不得阻塞, 不得递归调用
  *         底盘接口, 不得依赖 error 指向的数据在回调返回后仍然有效.
  */
typedef void (*NERV_Chassis_FaultCallbackTypeDef)(void *user, uint32_t error);

/**
  * @brief  底盘实例配置.
  * @note   由调用方以静态存储提供, 生命周期必须覆盖实例生命周期.
  * @note   电机实例指针指向的对象由调用方提供, 绑定后只读. 全部轮向电机必须已通过
  *         NERV_Motor_Init 初始化, 减速比与安装方向在电机实例的配置中表达.
  */
typedef struct
{
    const NERV_Motor_InstanceTypeDef
        *drive_motors[NERV_CHASSIS_WHEEL_COUNT]; /**< 轮向电机实例, 按轮号 0 到 3 排列 */
    float wheel_radius_m;                        /**< 轮半径, 单位 m, 必须大于 0 */
    float center_offset_x_m; /**< 轮组中心到底盘中心的 x 投影距, 单位 m, 必须大于 0 */
    float center_offset_y_m; /**< 轮组中心到底盘中心的 y 投影距, 单位 m, 必须大于 0 */
    float
        drive_torque_constant_n_m_per_a; /**< 轮向电机扭矩常数, 单位 N·m/A, 必须大于 0, 力分解结果经本值换算为电流指令 */
    float max_force_n;    /**< 牵引力限幅, 单位 N, 0 表示不额外限幅 */
    float max_torque_n_m; /**< 转矩限幅, 单位 N·m, 0 表示不额外限幅 */
    float translate_kp;   /**< 平移速度环比例增益, 单位 N/(m/s), 必须大于等于 0 */
    float translate_ki;   /**< 平移速度环积分增益, 单位 N/(m·s), 必须大于等于 0 */
    float translate_kd;   /**< 平移速度环微分增益, 单位 N·s/m, 必须大于等于 0 */
    float rotate_kp;      /**< 旋转速度环比例增益, 单位 N·m/(rad/s), 必须大于等于 0 */
    float rotate_ki;      /**< 旋转速度环积分增益, 单位 N·m·s/rad, 必须大于等于 0 */
    float rotate_kd;      /**< 旋转速度环微分增益, 单位 N·m·s/rad, 必须大于等于 0 */
} NERV_Chassis_ConfigTypeDef;

/**
  * @brief  底盘实例.
  * @note   由调用方以静态存储提供并清零, 生命周期覆盖全部调用.
  * @note   速度环的积分量与上次偏差由模块维护, 调用方只读, 不得写入.
  * @note   config 之后的字段由模块维护, 调用方只读, 不得写入.
  */
typedef struct NERV_Chassis_Instance
{
    NERV_Chassis_ConfigTypeDef config; /**< 配置, 由 NERV_Chassis_Init 绑定, 之后只读 */

    NERV_Chassis_VelocityTypeDef velocity; /**< 目标机体系速度, 由 NERV_Chassis_SetVelocity 写入 */
    NERV_Chassis_ForceTypeDef
        feedforward;                   /**< 前馈牵引力与转矩, 由 NERV_Chassis_SetFeedforward 写入 */
    float translate_integral[2];       /**< 平移速度环积分量, 索引 0 为 x 轴, 1 为 y 轴 */
    float translate_prev_deviation[2]; /**< 平移速度环上次偏差, 供微分项使用 */
    float rotate_integral;             /**< 旋转速度环积分量 */
    float rotate_prev_deviation;       /**< 旋转速度环上次偏差, 供微分项使用 */
    NERV_Chassis_OdomTypeDef odom;     /**< 里程计, 由 NERV_Chassis_Update 刷新 */
    uint32_t error;                    /**< 粘滞错误位, 经 NERV_Chassis_GetError 读取 */
    uint32_t last_update_ms;           /**< 上次 Update 的时刻, 单位 ms, 供里程计积分求间隔 */
    bool initialized;                  /**< 初始化完成标志, 由 NERV_Chassis_Init 置位 */
    NERV_Chassis_FaultCallbackTypeDef fault_callback; /**< 故障回调, NULL 表示未注册 */
    void *fault_user;                                 /**< 故障回调上下文, 可以为 NULL */
} NERV_Chassis_InstanceTypeDef;

/* 调用顺序: NERV_Chassis_Init 用配置建立实例, 之后周期调用 NERV_Chassis_Update
 * 完成正解, 速度环, 力分解与里程计. 目标速度经 NERV_Chassis_SetVelocity 设置,
 * 前馈经 NERV_Chassis_SetFeedforward 注入. 完整契约见各函数定义处. */

NERV_StatusTypeDef NERV_Chassis_Init(NERV_Chassis_InstanceTypeDef *self,
                                     const NERV_Chassis_ConfigTypeDef *config);

NERV_StatusTypeDef NERV_Chassis_RegisterFaultCallback(NERV_Chassis_InstanceTypeDef *self,
                                                      NERV_Chassis_FaultCallbackTypeDef callback,
                                                      void *user);

NERV_StatusTypeDef NERV_Chassis_SetVelocity(NERV_Chassis_InstanceTypeDef *self,
                                            const NERV_Chassis_VelocityTypeDef *velocity);

NERV_StatusTypeDef NERV_Chassis_SetFeedforward(NERV_Chassis_InstanceTypeDef *self,
                                               const NERV_Chassis_ForceTypeDef *force);

NERV_StatusTypeDef NERV_Chassis_Update(NERV_Chassis_InstanceTypeDef *self, uint32_t now_ms);

NERV_StatusTypeDef NERV_Chassis_GetOdometry(const NERV_Chassis_InstanceTypeDef *self,
                                            NERV_Chassis_OdomTypeDef *odom);

uint32_t NERV_Chassis_GetError(const NERV_Chassis_InstanceTypeDef *self);

void NERV_Chassis_ClearError(NERV_Chassis_InstanceTypeDef *self, uint32_t mask);

NERV_StatusTypeDef NERV_Chassis_EnterSafeState(NERV_Chassis_InstanceTypeDef *self);

/* 全向轮底盘布局与编号约定 (NERV_CHASSIS_TYPE_OMNI, 4 轮):
 *
 * 车体系坐标系 x 指向车体前方, y 指向左方, z 指向上方. 轮号 0 到 3 依次为左前,
 * 左后, 右后, 右前, 沿底盘周向排列, 位置由配置的 center_offset_x_m 与
 * center_offset_y_m 给出轮到中心的投影距.
 *
 * 各轮驱动方向 (轮组线速度正方向) 与 x 轴夹角依次为 135°, 225°, 315°, 45°, 沿各自
 * 对角线向外. 驱动方向整体相差 180° 由轮向电机的安装转向吸收, 对应电机实例配置的
 * invert_direction, 不影响本表.
 *
 * 控制框架为速度环加力分解的三层串级: 由各轮测量角速度经运动学正解算得到当前机体系
 * 速度; 平移与旋转速度环把速度误差解为牵引力与转矩; 力分解把牵引力, 转矩与各轮固定
 * 驱动方向解为轮扭矩, 叠加调用方提供的前馈后经扭矩常数换算为电流下发轮向电机. 四个
 * 固定方向的轮子合成任意方向的牵引力与转矩, 以轮向电机实现舵轮框架中舵向与轮向电机
 * 的合力效果, 不设舵向电机. */

#ifdef __cplusplus
}
#endif

#endif /* NERV_USE_CHASSIS */

#endif /* NERV_CHASSIS_H */

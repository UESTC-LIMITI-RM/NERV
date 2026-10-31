/**
  * @file    nerv_math.c
  * @brief   NERV 数学工具实现.
  * @version 1.0.0
  * @date    2026-10-21
  * @copyright Copyright (c) 2026 NERV Contributors
  *
  * SPDX-License-Identifier: MIT
  */

#include "nerv_math.h"

#include <math.h>
#include <stddef.h>

/**
  * @brief  把数值归一化到指定区间.
  * @param  value 待归一化的数值.
  * @param  min_value 区间下界, 包含.
  * @param  max_value 区间上界, 不包含.
  * @return 与 value 相差整数个区间长度且落在 [min_value, max_value) 的值.
  * @note   必须满足 min_value 与 max_value 为有限值, 且 max_value 大于 min_value.
  *         边界非法时返回 value, 不做归一化.
  * @note   非有限输入原样返回, 结果不在区间内.
  * @note   当 min_value 的绝对值远大于区间长度时, 减法会损失有效位.
  * @note   ISR-safe.
  */
float NERV_Math_Wrap(float value, float min_value, float max_value)
{
    float period;
    float offset;
    float result;

    period = max_value - min_value;

    if ((!isfinite(min_value)) || (!isfinite(max_value)) || (!(period > 0.0f)) ||
        (!isfinite(period)))
    {
        return value;
    }

    if (!isfinite(value))
    {
        return value;
    }

    offset = fmodf(value - min_value, period);

    if (!isfinite(offset))
    {
        return value;
    }

    if (offset < 0.0f)
    {
        offset += period;
    }

    result = offset + min_value;

    if (!(result < max_value))
    {
        result = min_value;
    }

    return result;
}

/**
  * @brief  判断两个浮点数是否在容差内相等.
  * @param  a 第一个值.
  * @param  b 第二个值.
  * @param  tolerance 绝对容差, 单位与被比较值相同, 必须为大于等于 0 的有限值.
  * @param  is_close 比较结果输出, 不得为 NULL. 成功时写入, 失败时不修改.
  * @retval NERV_OK 比较完成, 结果已写入 is_close.
  * @retval NERV_ERROR is_close 为 NULL, 或 tolerance 小于 0 或非有限.
  * @note   只做绝对比较, 不做相对比较. 浮点数相等判断必须走本函数, 禁止直接用 `==`.
  * @note   任一输入为 NaN 或无穷时结果为 false.
  * @note   ISR-safe.
  */
NERV_StatusTypeDef NERV_Math_IsClose(float a, float b, float tolerance, bool *is_close)
{
    if (NULL == is_close)
    {
        return NERV_ERROR;
    }

    if ((!isfinite(tolerance)) || (tolerance < 0.0f))
    {
        return NERV_ERROR;
    }

    *is_close = (fabsf(a - b) <= tolerance);

    return NERV_OK;
}

/**
  * @brief  把数值限制在给定区间内.
  * @param  value 待限制的数值.
  * @param  min_value 区间下界.
  * @param  max_value 区间上界.
  * @return 落在区间内的值, 否则为越界方向的边界值.
  * @note   调用方必须保证 min_value 小于等于 max_value. 两者颠倒时返回 min_value.
  * @note   value 为 NaN 时原样返回 NaN (NaN 与边界比较均为假), 调用方必须先行
  *         检查非有限值. value 为无穷时按越界处理返回对应边界值.
  * @note   ISR-safe.
  */
float NERV_Math_Clamp(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }

    if (value > max_value)
    {
        return max_value;
    }

    return value;
}

/**
  * @brief  对数值做死区处理.
  * @param  value 待处理数值.
  * @param  threshold 死区阈值, 必须大于等于 0.
  * @return 死区内为 0. 死区外为扣除阈值后的同号数值, 在阈值边界处连续.
  * @note   连续型死区会压缩量程. 需要保留原量程时由调用方自行处理.
  * @note   threshold 为负时不做死区处理, 原值返回.
  * @note   ISR-safe.
  */
float NERV_Math_Deadband(float value, float threshold)
{
    if (threshold < 0.0f)
    {
        return value;
    }

    const float magnitude = fabsf(value);

    if (magnitude <= threshold)
    {
        return 0.0f;
    }

    if (value < 0.0f)
    {
        return value + threshold;
    }

    return value - threshold;
}

/**
  * @brief  绕 z 轴逆时针旋转平面向量.
  * @param  vx 向量的 x 分量, 单位与调用方约定一致. 就地更新.
  * @param  vy 向量的 y 分量, 单位与调用方约定一致. 就地更新.
  * @param  angle_rad 旋转角度, 单位 rad, 逆时针为正.
  * @retval NERV_OK 旋转成功.
  * @retval NERV_ERROR vx 或 vy 为 NULL.
  * @note   把云台系的底盘速度指令转到机体系时, 角度取云台航向减底盘航向.
  * @note   ISR-safe.
  */
NERV_StatusTypeDef NERV_Math_RotateZ(float *vx, float *vy, float angle_rad)
{
    if ((NULL == vx) || (NULL == vy))
    {
        return NERV_ERROR;
    }

    const float cos_value = cosf(angle_rad);
    const float sin_value = sinf(angle_rad);
    const float new_vx    = ((*vx) * cos_value) - ((*vy) * sin_value);
    const float new_vy    = ((*vx) * sin_value) + ((*vy) * cos_value);

    *vx = new_vx;
    *vy = new_vy;

    return NERV_OK;
}

/** @cond INTERNAL */

/** @brief 三阶线性方程组求解的相对奇异判据阈值. */
#define __NERV_MATH_LINEAR3_SINGULAR_EPS 1e-6f

/**
  * @brief  计算三阶矩阵的行和绝对值的最大值, 作为矩阵量级.
  * @param  matrix 九个元素, 不得为 NULL.
  * @return 矩阵无穷范数, 用于相对奇异判据的量纲基准.
  */
static float NERV_Math_MatrixNorm3(const float *matrix)
{
    float norm;

    norm = 0.0f;

    for (uint8_t row = 0U; row < 3U; row++)
    {
        const float row_sum =
            fabsf(matrix[row * 3U]) + fabsf(matrix[row * 3U + 1U]) + fabsf(matrix[row * 3U + 2U]);

        if (row_sum > norm)
        {
            norm = row_sum;
        }
    }

    return norm;
}

/**
  * @brief  计算三阶矩阵的行列式, 矩阵按行优先连续存储.
  * @param  matrix 九个元素, 不得为 NULL.
  * @return 行列式值.
  */
static float NERV_Math_Det3(const float *matrix)
{
    return matrix[0] * (matrix[4] * matrix[8] - matrix[5] * matrix[7]) -
           matrix[1] * (matrix[3] * matrix[8] - matrix[5] * matrix[6]) +
           matrix[2] * (matrix[3] * matrix[7] - matrix[4] * matrix[6]);
}

/** @endcond */

/**
  * @brief  解三阶线性方程组.
  * @param  matrix 三阶系数矩阵, 行优先连续存储, 九个元素, 不得为 NULL.
  * @param  rhs 右侧向量, 三个元素, 不得为 NULL.
  * @param  solution 输出解向量, 三个元素, 不得为 NULL. 失败时不修改.
  * @retval NERV_OK 解已写入 solution.
  * @retval NERV_ERROR 参数为 NULL, 矩阵奇异或近奇异, 或行列式为非有限值.
  * @note   用 Cramer 法则求解, 每个解为替换对应列后的行列式除以系数行列式.
  * @note   奇异判据是相对的: 行列式绝对值不超过矩阵无穷范数立方的 1e-6 时按奇异
  *         处理. 这样矩阵退化 (如各列共线) 与接近退化 (条件数极大, 解会放大到
  *         不可用的量级) 都会返回错误, 而不是输出巨大但有限的解.
  * @note   ISR-safe.
  */
NERV_StatusTypeDef NERV_Math_SolveLinear3(const float *matrix, const float *rhs, float *solution)
{
    float work[9];

    if ((NULL == matrix) || (NULL == rhs) || (NULL == solution))
    {
        return NERV_ERROR;
    }

    const float determinant = NERV_Math_Det3(matrix);

    if (!isfinite(determinant))
    {
        return NERV_ERROR;
    }

    const float matrix_norm = NERV_Math_MatrixNorm3(matrix);

    if (fabsf(determinant) <=
        (__NERV_MATH_LINEAR3_SINGULAR_EPS * matrix_norm * matrix_norm * matrix_norm))
    {
        return NERV_ERROR;
    }

    for (uint8_t column = 0U; column < 3U; column++)
    {
        for (uint8_t row = 0U; row < 9U; row++)
        {
            work[row] = matrix[row];
        }

        work[column]      = rhs[0];
        work[3U + column] = rhs[1];
        work[6U + column] = rhs[2];

        solution[column] = NERV_Math_Det3(work) / determinant;
    }

    return NERV_OK;
}

/**
 * @file algo_math.hpp
 * @author cuteelaina (1105549920@qq.com)
 * @brief 数学工具：循环限幅、符号函数、滤波等
 * @version 1.0
 * @date 2026-05-28
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef ALGO_MATH_HPP
#define ALGO_MATH_HPP

#include <cstdint>
#include <cmath>

namespace robotpilots::algorithm {

/**
 * @brief 循环限幅（角度归一化等），将 val 映射到 [lo, hi) 区间
 * @param val 输入值
 * @param lo 下限（包含）
 * @param hi 上限（不包含）
 * @return 循环映射后的值
 */
inline float wrap(float val, float lo, float hi) {
    if (lo >= hi) return val;  // 无效区间，直接返回
    float range = hi - lo;
    float t = std::fmod(val - lo, range);  // O(1)，避免大数值时循环卡死
    if (t < 0.0f) t += range;              // fmod 结果与被除数同号，负数需回正
    return lo + t;                          // 结果落在 [lo, hi)
}

/**
 * @brief 符号函数
 * @param val 输入值
 * @return 1 (正), -1 (负), 0 (零)
 */
inline float sign(float val) {
    return (val > 0) ? 1.0f : (val < 0) ? -1.0f : 0.0f;
}

/**
 * @brief 快速平方根倒数 (1/sqrt(x))
 * @param x 输入正数
 * @return 近似 1/sqrt(x)
 * @note 使用经典 Quake 算法，精度约 0.1%
 */
inline float invSqrt(float x) {
    union {
        int32_t i;
        float f;
    } u;
    u.f = x;
    u.i = 0x5f3759df - (u.i >> 1);
    return u.f * (1.5f - 0.5f * x * u.f * u.f);
}

/**
 * @brief 一阶低通滤波器
 * @param last 上一次滤波输出
 * @param current 当前输入
 * @param alpha 滤波系数 (0 < alpha < 1)，越接近1越平滑
 * @return 滤波结果 = last * alpha + current * (1 - alpha)
 */
inline float lowPassFilter(float last, float current, float alpha) {
    return last * alpha + current * (1.0f - alpha);
}

} // namespace balgorithm

#endif // ALGO_MATH_HPP

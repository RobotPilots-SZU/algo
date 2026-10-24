/**
 * @file math.hpp
 * @author cuteelaina (1105549920@qq.com)
 * @brief 数学工具：循环限幅、符号函数、滤波等
 * @version 1.0
 * @date 2026-05-28
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <cassert>
#include <cmath>
#include <cstdint>
#include <type_traits>

namespace robotpilots::algorithm
{

/**
 * @brief 循环限幅, 将 value 映射到 [lower, upper) 区间
 */
template <typename T>
constexpr T Wrap(T value, T lower, T upper)
{
	static_assert(std::is_floating_point_v<T>, "Wrap requires a floating point type");
	if (lower >= upper)
	{
		return value;
	}

	const T range = upper - lower;
	while (value >= upper)
	{
		value -= range;
	}
	while (value < lower)
	{
		value += range;
	}
	return value;
}

/**
 * @brief 将误差归一化到周期循环区间 [-period / 2, period / 2)
 */
template <typename T>
constexpr T WrapError(T error, T period)
{
	static_assert(std::is_floating_point_v<T>, "WrapError requires a floating point type");
	assert(period > T{});

	const T half = period / T{2};
	while (error >= half)
	{
		error -= period;
	}
	while (error < -half)
	{
		error += period;
	}
	return error;
}

/**
 * @brief 输入死区, 误差绝对值小于阈值时归零
 */
template <typename T>
constexpr T Deadband(T error, T threshold)
{
	static_assert(std::is_floating_point_v<T>, "Deadband requires a floating point type");
	return std::abs(error) < threshold ? T{} : error;
}

/**
 * @brief 符号函数
 */
template <typename T>
constexpr T Sign(T value)
{
	static_assert(std::is_floating_point_v<T>, "Sign requires a floating point type");
	return (value > T{}) ? T{1} : (value < T{}) ? T{-1} : T{};
}

/**
 * @brief 快速平方根倒数 (1/sqrt(x))
 * @note 使用经典 Quake 算法, 精度约 0.1%
 */
template <typename T>
constexpr T InvSqrt(T value)
{
	static_assert(std::is_same_v<T, float>, "InvSqrt currently supports float only");
	union Bits
	{
		std::int32_t Integer;
		float Real;
	} bits{};

	bits.Real = value;
	bits.Integer = 0x5f3759df - (bits.Integer >> 1);
	return bits.Real * (1.5f - 0.5f * value * bits.Real * bits.Real);
}

/**
 * @brief 一阶低通滤波器
 */
template <typename T>
constexpr T LowPassFilter(T last, T current, T alpha)
{
	static_assert(std::is_floating_point_v<T>, "LowPassFilter requires a floating point type");
	return last * alpha + current * (T{1} - alpha);
}

} // namespace robotpilots::algorithm

/**
 * @file pid.hpp
 * @author cuteelaina (1105549920@qq.com)
 * @brief PID 控制器
 * @version 2.0
 * @date 2026-05-28
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef ALGO_PID_HPP
#define ALGO_PID_HPP

#include <algorithm>
#include <cmath>
#include <type_traits>

namespace robotpilots::algorithm
{

/**
 * @brief 将误差归一化到周期循环区间 [-period / 2, period / 2)
 * @param error 原始误差
 * @param period 循环周期, 必须大于 0
 * @return 归一化后的误差
 */
template<typename T>
constexpr T wrap_error(T error, T period)
{
	static_assert(std::is_floating_point_v<T>, "wrap_error requires a floating point type");
	assert(period > T{});

	const T half = period / T{2};

	while (error > half) {
		error -= period;
	}

	while (error < -half) {
		error += period;
	}

	return error;
}

/**
 * @brief 输入死区, 误差绝对值小于阈值时归零
 * @param error 原始误差
 * @param threshold 死区阈值
 * @return 死区处理后的误差
 */
template<typename T>
constexpr T deadband(T error, T threshold)
{
	static_assert(std::is_floating_point_v<T>, "deadband requires a floating point type");

	return std::abs(error) < threshold ? T{} : error;
}

/**
 * @brief PID 控制器
 * @note 只负责根据误差和采样周期计算输出, 环回、限幅、死区等由外部处理
 */
template<typename T = float>
class Pid
{
public:
	struct Config
	{
		T kp = {};                ///< 比例系数
		T ki = {};                ///< 积分系数
		T kd = {};                ///< 微分系数

		T integral_limit = {};    ///< 积分限幅, <= 0 表示不限幅
		T integral_separation = {}; ///< 积分分离阈值, <= 0 表示不分离
	};

	explicit Pid(const Config config) : config_(config) {}

	/**
	 * @brief 根据误差更新 PID 并返回输出
	 * @param error 误差, 需由外部计算
	 * @param dt 采样周期, 单位为秒, 输入需 >0
	 * @return PID 输出
	 */
	T update(const T error, const T dt)
	{
		assert(dt > T{});

		if (config_.integral_separation > T{} &&
			std::abs(error) > config_.integral_separation)
		{
			integral_ = T{};
		}
		else
		{
			integral_ += error * dt;
			integral_ = limit(integral_, config_.integral_limit);
		}

		const T derivative = initialized_ ? (error - last_error_) / dt : T{};

		last_error_ = error;
		initialized_ = true;

		return config_.kp * error
			 + config_.ki * integral_
			 + config_.kd * derivative;
	}

	/**
	 * @brief 重置 PID 内部状态
	 */
	void reset()
	{
		integral_ = T{};
		last_error_ = T{};
		initialized_ = false;
	}

private:
	static constexpr T limit(const T value, const T bound)
	{
		return std::clamp(value, -bound, bound);
	}

	Config config_;

	T integral_ = {};
	T last_error_ = {};
	bool initialized_ = false;
};

} // namespace robotpilots::algorithm

#endif // ALGO_PID_HPP

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

#pragma once

#include <cmath>
#include <etl/algorithm.h>

namespace robotpilots::algorithm
{

/**
 * @brief PID 控制器
 * @note 只负责根据误差和采样周期计算输出, 环回、限幅、死区等由外部处理
 */
template <typename T = float>
class Pid
{
public:
	struct Config
	{
		T Kp = {}; ///< 比例系数
		T Ki = {}; ///< 积分系数
		T Kd = {}; ///< 微分系数

		T IntegralLimit = {};      ///< 积分限幅, <= 0 表示不限幅
		T IntegralSeparation = {}; ///< 积分分离阈值, <= 0 表示不分离
	};

	explicit Pid(const Config config)
	    : _config(config)
	{}

	/**
	 * @brief 根据误差更新 PID 并返回输出
	 * @param error 误差, 需由外部计算
	 * @param dt 采样周期, 单位为秒, 输入需 >0
	 * @return PID 输出
	 */
	T Update(const T error, const T dt)
	{
		assert(dt > T{});

		if (_config.IntegralSeparation > T{} && std::abs(error) > _config.IntegralSeparation)
		{
			_integral = T{};
		}
		else
		{
			_integral += error * dt;
			_integral = Limit(_integral, _config.IntegralLimit);
		}

		const T derivative = _initialized ? (error - _lastError) / dt : T{};

		_lastError = error;
		_initialized = true;

		return _config.Kp * error + _config.Ki * _integral + _config.Kd * derivative;
	}

	/**
	 * @brief 重置 PID 内部状态
	 */
	void Reset()
	{
		_integral = T{};
		_lastError = T{};
		_initialized = false;
	}

private:
	static constexpr T Limit(const T value, const T bound)
	{
		return etl::clamp(value, -bound, bound);
	}

	Config _config;

	T _integral = {};
	T _lastError = {};
	bool _initialized = false;
};

} // namespace robotpilots::algorithm

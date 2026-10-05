/**
 * @file conf_algo.hpp
 * @author sllllr (2997708711@qq.com)
 * @brief 算法配置
 * @version 1.0
 * @date 2026-01-12
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include <stdint.h>
#include <cmath>

namespace robotpilots::algorithm {

/**
 * @brief 算法ID枚举类型
 *
 */
enum class EAlgoID 
{
    ALGO_NULL = -1,   ///< 空算法
    ALGO_KF = 1,      ///< 基本卡尔曼滤波
    ALGO_IMU_EKF,     ///< IMU扩展卡尔曼滤波
};

/**
 * @brief Application Status枚举类型
 * @note 用作Application中函数的返回值
 */
enum EAppStatus 
{
    APP_RESET = 0, ///< 重置
    APP_OK = 1,    ///< 正常
    APP_ERROR,     ///< 错误
    APP_BUSY,      ///< 忙
    APP_TIMEOUT,   ///< 超时
    APP_FULL,      ///< 满
    APP_EMPTY,     ///< 空
    APP_INVALID,   ///< 无效
    APP_UNKNOWN    ///< 未知
};

} // namespace robotpilots::algorithm


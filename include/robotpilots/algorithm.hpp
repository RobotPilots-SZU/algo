/**
 * @file algorithm.hpp
 * @author cuteelaina (1105549920@qq.com)
 * @brief 算法库公共入口
 * @version 1.0
 * @date 2026-05-28
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef ALGORITHM_HPP
#define ALGORITHM_HPP

// 包含所有公共子模块
#include "algorithm/conf_algo.hpp"
#include "algorithm/math.hpp"
#include "algorithm/crc.hpp"
#include "algorithm/pid.hpp"
#ifdef ALGO_ENABLE_ARM_MATH
#include "algorithm/rp_matrix.hpp" 
#include "algorithm/filter_common.hpp"
#include "algorithm/kf_filter.hpp"
#include "algorithm/ekf_filter.hpp"
#endif

#endif  // ALGORITHM_HPP

/**
 * @file algo_types.hpp
 * @author cuteelaina (1105549920@qq.com)
 * @brief 类型定义与数据接收视图
 * @version 1.0
 * @date 2026-05-28
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <etl/span.h>

namespace robotpilots::algorithm {
/**
 * @brief 数据接收视图模板。不持有数据，只负责接收数据用于计算。
 * @tparam T为要存储的数据类型
 */
template<typename T> using DataReceiver = etl::span<const T>;

}   // namespace robotpilots::algorithm


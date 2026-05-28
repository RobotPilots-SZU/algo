#ifndef ALGO_TYPES_HPP
#define ALGO_TYPES_HPP

#include <etl/span.h>

namespace robotpilots::algorithm {

/**
 * @brief Application Status枚举类型
 * @note 用作Application中函数的返回值
 */
enum EAppStatus{
    APP_RESET = 0,  ///< 重置
    APP_OK = 1,     ///< 正常
    APP_ERROR,      ///< 错误
    APP_BUSY,       ///< 忙
    APP_TIMEOUT,    ///< 超时
    APP_FULL,       ///< 满
    APP_EMPTY,      ///< 空
    APP_INVALID,    ///< 无效
    APP_UNKNOWN     ///< 未知
};


/**
 * @brief 数据接收视图模板。不持有数据，只负责接收数据用于计算。
 * @tparam T为要存储的数据类型
 */
template<typename T> using DataReceiver = etl::span<const T>;

}   // namespace robotpilots::algorithm

#endif // ROBOTPILOTS_ALGORITHM_TYPES_HPP

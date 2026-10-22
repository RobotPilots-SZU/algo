/**
 * @file crc.hpp
 * @author cuteelaina (1105549920@qq.com)
 * @brief CRC8/CRC16 校验计算
 * @version 1.0
 * @date 2026-05-28
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef ALGO_CRC_HPP
#define ALGO_CRC_HPP

#include <cstdint>
#include <etl/span.h>

namespace robotpilots::algorithm
{

/**
 * @brief 从数据缓冲区计算 crc8.
 * @param data[in] 数据缓冲区输入
 * @param init 初始CRC值
 *
 * @return CRC8 Checksum Value
 */
uint8_t Crc8Calculate(etl::span<const uint8_t> data, uint8_t init = 0xff);

/**
 * @brief 从数据缓冲区计算 crc8.
 * @param data[in] 数据缓冲区输入
 * @param init 初始CRC值
 *
 * @return CRC16 Checksum Value
 */
uint16_t Crc16Calculate(etl::span<const uint8_t> data, uint16_t init = 0xffff);

} // namespace robotpilots::algorithm

#endif // ALGO_CRC_HPP

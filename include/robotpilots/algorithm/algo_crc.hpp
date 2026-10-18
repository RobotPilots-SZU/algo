/**
 * @file algo_crc.hpp
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

namespace robotpilots::algorithm {

/**
 * @brief Calculate CRC8 Checksum Value from Data Buffer
 * @param data[in] Data Buffer
 * @param init Initial CRC8 Value
 *
 * @return CRC8 Checksum Value
 */
uint8_t Crc8Calculate(etl::span<const uint8_t> data, uint8_t init = 0xff);

/**
 * @brief Calculate CRC16 Checksum Value from Data Buffer
 * @param data[in] Data Buffer
 * @param init Initial CRC16 Value
 *
 * @return CRC16 Checksum Value
 */
uint16_t Crc16Calculate(etl::span<const uint8_t> data, uint16_t init = 0xffff);

} // namespace robotpilots::algorithm

#endif // ALGO_CRC_HPP

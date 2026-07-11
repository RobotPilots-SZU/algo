/**
 * @file algo_crc.hpp
 * @author cuteelaina (1105549920@qq.com)
 * @brief CRC8/CRC16 校验计算与验证
 * @version 1.0
 * @date 2026-05-28
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef ALGO_CRC_HPP
#define ALGO_CRC_HPP

#include <cstdint>
#include <cstddef>
#include "algo_types.hpp"

namespace robotpilots::algorithm {

/**
 * @brief Verify CRC8 Checksum Value for Data Buffer
 * @param buffer[in] Data Buffer Vector
 * @param crc[in] CRC8 Checksum Value
 *
 * @returns RP_OK - Verify Succeeded \n
 *          RP_ERROR - Verify Failed
 */
bool Crc8Verify(const DataReceiver<uint8_t> buffer, uint8_t crc);

/**
 * @brief Verify CRC8 Value Checksum for Data Buffer
 * @param buffer[in] Data Buffer Pointer
 * @param crc[in] CRC8 Checksum Value
 * @param len[in] Bit Length of Data Buffer
 *
 * @returns RP_OK - Verify Succeeded \n
 *          RP_ERROR - Verify Failed
 */
bool Crc8Verify(const uint8_t *buffer, uint8_t crc, size_t len);

/**
 * @brief Verify CRC16 Checksum Value for Data Buffer
 * @param buffer[in] Data Buffer
 * @param crc[in] CRC16 Checksum Value
 *
 * @returns RP_OK - Verify Succeeded \n
 *          RP_ERROR - Verify Failed
 */
bool Crc16Verify(const DataReceiver<uint8_t> buffer, uint16_t crc);

/**
 * @brief Verify CRC16 Checksum Value for Data Buffer
 * @param buffer[in] Data Buffer Pointer
 * @param crc[in] CRC16 Checksum Value
 * @param len[in] Bit Length of Data Buffer
 *
 * @returns RP_OK - Verify Succeeded \n
 *          RP_ERROR - Verify Failed
 */
bool Crc16Verify(const uint8_t *buffer, uint16_t crc, size_t len);

/**
 * @brief Calculate CRC8 Checksum Value from Data Buffer
 * @param buffer[in] Data Buffer
 *
 * @return CRC8 Checksum Value
 */
uint8_t Crc8Calculate(const DataReceiver<uint8_t> buffer);

/**
 * @brief Calculate CRC8 Checksum Value from Data Buffer
 * @param buffer[in] Data Buffer Pointer
 * @param len[in] Bit Length of Data Buffer
 *
 * @return CRC8 Checksum Value
 */
uint8_t Crc8Calculate(const uint8_t *buffer, size_t len);

/**
 * @brief Calculate CRC16 Checksum Value from Data Buffer
 * @param buffer[in] Data Buffer
 *
 * @return CRC16 Checksum Value
 */
uint16_t Crc16Calculate(const DataReceiver<uint8_t> buffer);

/**
 * @brief Calculate CRC16 Checksum Value from Data Buffer
 * @param buffer[in] Data Buffer Pointer
 * @param len[in] Bit Length of Data Buffer
 *
 * @return CRC16 Checksum Value
 */
uint16_t Crc16Calculate(const uint8_t *buffer, size_t len);

} // namespace robotpilots::algorithm

#endif // ALGO_CRC_HPP

/*
 * Copyright (c) 2023 Particle Industries, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Adapted from:
 *   MODBUS over serial line, https://modbus.org/docs/Modbus_over_serial_line_V1_02.pdf
 *   modbus tools, https://www.modbustools.com/modbus_crc16.html
  */


#pragma once

#include <cstdint>
#include <cstddef>

/**
 * @brief Calculate the Modbus CRC16 with a given buffer.
 *
 * @param data Byte pointer to data
 * @param count Number of bytes to include from given data
 * @return uint16_t CRC16 value of data
 */
uint16_t ModbusCrc16 (const uint8_t* data, size_t count);

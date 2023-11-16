/**
 * Copyright (c) 2022 Particle Industries, Inc.
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
 */
#pragma once

#include <cstddef>
#include <cstdint>

#include "Particle.h"

class SensirionBase {
protected:
    SensirionBase(TwoWire &i2c, std::uint8_t address) : _i2c(i2c), _address(address) {};

    /**
     * @brief Initialize the interface
     *
     * @details Attempts to begin i2c transmission of the sensor to
     * validate the sensor can communicate
     *
     * @return true on success, false on failure
     */
    bool init();

    /**
     * @brief Used to read a sensirion sensor
     *
     * @details Will read multiple words from a read command
     *
     * @param[in] address of sensirion device to read
     * @param[in] command to read
     * @param[out] data_words buffer containing words read
     * @param[in] num_words number of words to read
     *
     * @return true on success, false on failure
     */
    bool readCmd(std::uint16_t command, std::uint16_t *data_words, std::size_t num_words);

    /**
     * @brief Used to write a command to a sensirion sensor
     *
     * @details Writes a command to a sensirion sensor
     *
     * @param[in] address of sensirion device to write
     * @param[in] command to write
     *
     * @return true on success, false on failure
     */
    bool writeCmd(std::uint16_t command);

    /**
     * @brief Used to write a command with multiple arguments to a sensirion
     * sensor
     *
     * @details Writes a a command with multiple arguments to a sensirion
     * sensor
     *
     * @param[in] address of sensirion device to write
     * @param[in] command to write
     * @param[in] data_words arguments to write, 1 argument = 1 word
     * @param[in] num_words number of arguments to write, 1 argument = 1 word
     *
     * @return <what does the function return (optional if void)>
     */
    bool writeCmdWithArgs(std::uint16_t command, const std::uint16_t *data_words, std::size_t num_words);

    /**
     * @brief Read a register from a sensirion device
     *
     * @details Given the address of a sensirion device, read out the register
     * contents of the device. User must first write to the register they want
     * to read, then you can read from that register
     *
     * @param[in] address of sensirion device to read
     * @param[out] buf buffer to store read bytes into
     * @param[in] length number of bytes to read
     *
     * @return number of bytes read
     */
    std::size_t readRegister(std::uint8_t *buf, std::size_t length);

    /**
     * @brief Write a register of a sensirion device
     *
     * @details Given the address of a sensirion device, write to that register
     * of the device.
     *
     * @param[in] address of sensirion device to write
     * @param[in] buf buffer containing payload to write
     * @param[in] length number of bytes to write
     * @param[in] stop true to send a stop message on the I2C bus after transmission
     *
     * @return number of bytes written
     */
    std::size_t writeRegister(const std::uint8_t *buf, std::size_t length, bool stop = true);

    /**
     * @brief Read words from a sensirion device
     *
     * @details Used to read a single word, or multiple words from a sensirion
     * device. Will read out single bytes (calls readWordsAsBytes() function)
     * from the sensirion device, and then concatenate those bytes to get a
     * full word from a device.
     *
     * @param[in] address of sensirion device to read
     * @param[out] buffer to store read words
     * @param[in] num_words number of words to read
     *
     * @return true on success, false on failure
     */
    bool readWords(std::uint16_t *data_words, std::size_t num_words);

    static Logger driver_log;

private:
    TwoWire &_i2c;
    std::uint8_t _address;

    /**
     * @brief Generates a CRC
     *
     * @details Used to generate a CRC from a buffer of a given length
     *
     * @param[in] data buffer of data to calculate CRC
     * @param[in] len length of the buffer
     *
     * @return the generated CRC
     */
    static std::uint8_t generateCrc(const std::uint8_t *data, std::size_t len);
};

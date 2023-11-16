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
 *
 * Copyright (c) 2018, Sensirion AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Sensirion AG nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <cstdint>

#include "SensirionBase.h"

constexpr std::size_t ReceiveBufferSize {32u};

Logger SensirionBase::driver_log("sensirion-driver");

bool SensirionBase::init()
{
    const std::lock_guard<TwoWire> lg(_i2c);

    _i2c.begin();
    _i2c.beginTransmission(_address);
    if (_i2c.endTransmission() != 0) {
        driver_log.error("address 0x%X invalid or device failed", _address);
        return false;
    }
    return true;
}

bool SensirionBase::readCmd(std::uint16_t command, std::uint16_t *data_words, std::size_t num_words)
{
    const std::lock_guard<TwoWire> lg(_i2c); // lock the I2C bus between write and read since no stop condition is sent
    std::uint16_t word {inet_htons(command)};

    if (writeRegister(reinterpret_cast<std::uint8_t *>(&word), sizeof(word), false) != sizeof(word)) {
        driver_log.error("failed read command: 0x%X", command);
        return false;
    }

    return readWords(data_words, num_words);
}

bool SensirionBase::writeCmd(std::uint16_t command)
{
    std::uint16_t word {inet_htons(command)};

    if (writeRegister(reinterpret_cast<std::uint8_t *>(&word), sizeof(word)) != sizeof(word)) {
        driver_log.error("failed write command: 0x%X", command);
        return false;
    }
    return true;
}

bool SensirionBase::writeCmdWithArgs(std::uint16_t command, const std::uint16_t *data_words, std::size_t num_words)
{
    std::uint8_t buf[ReceiveBufferSize];
    std::size_t buf_size {sizeof(std::uint16_t) * (1 + num_words)};
    auto it {reinterpret_cast<std::uint16_t *>(buf)};

    if (buf_size >= ReceiveBufferSize) {
        return false;
    }
    *it = inet_htons(command);
    while (num_words--) {
        *it++ = inet_htons(*data_words);
        data_words++;
    }

    if (writeRegister(buf, buf_size) != buf_size) {
        driver_log.error("failed write command 0x%X with args", command);
        return false;
    }
    return true;
}

std::uint8_t SensirionBase::generateCrc(const std::uint8_t *data, std::size_t len)
{
    std::uint8_t crc = 0xffu;

    for (auto i = 0u; i < len; ++i) {
        crc ^= data[i];
        auto j {8u};
        while (j--) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x31;
            } else {
                crc = (crc << 1);
            }
        }
    }
    return crc;
}

bool SensirionBase::readWords(std::uint16_t *data_words, std::size_t num_words)
{
    std::uint8_t buf[ReceiveBufferSize];
    std::size_t buf_size {3 * num_words};
    auto it {static_cast<std::uint8_t *>(buf)};

    if (buf_size >= ReceiveBufferSize) {
        return false;
    }

    if (readRegister(buf, buf_size) != buf_size) {
        driver_log.error("receive data failed");
        return false;
    }
    while (num_words--) {
        if (generateCrc(it, 2) != it[2]) {
            driver_log.error("checksum match failed");
            return false;
        }
        *data_words++ = inet_ntohs(*reinterpret_cast<std::uint16_t *>(it));
        it += 3;
    }
    return true;
}

std::size_t SensirionBase::writeRegister(const std::uint8_t *buf, std::size_t length, bool stop)
{
    const std::lock_guard<TwoWire> lg(_i2c);

    _i2c.beginTransmission(_address);
    std::size_t ret = _i2c.write(buf, length);
    _i2c.endTransmission(stop);
    return ret;
}

std::size_t SensirionBase::readRegister(std::uint8_t *buf, std::size_t length)
{
    const std::lock_guard<TwoWire> lg(_i2c);
    std::size_t count {};

    _i2c.requestFrom(_address, length);
    while (_i2c.available() && length--) {
        *buf++ = _i2c.read();
        count++;
    }
    return count;
}

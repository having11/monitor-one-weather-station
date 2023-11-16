/**
 * Copyright (c) 2022 Particle Industries, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at¡
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Copyright (c) 2018, Sensirion AG1
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

#include "Sht3x.h"

// Command words
constexpr std::uint16_t SHT3xReadAlertHighSet {0xE11F};
constexpr std::uint16_t SHT3xReadAlertHighClear {0xE114};
constexpr std::uint16_t SHT3xReadAlertLowSet {0xE109};
constexpr std::uint16_t SHT3xReadAlertLowClear {0xE102};

constexpr std::uint16_t SHT3xWriteAlertHighSet {0x611D};
constexpr std::uint16_t SHT3xWriteAlertHighClear {0x6116};
constexpr std::uint16_t SHT3xWriteAlertLowSet {0x610B};
constexpr std::uint16_t SHT3xWriteAlertLowClear {0x6100};

constexpr std::uint16_t SHT3xPeriodicRead {0xE000};
constexpr std::uint16_t SHT3xBreak {0x3093};
constexpr std::uint16_t SHT3xReadStatus {0xF32D};
constexpr std::uint16_t SHT3xClearStatus {0x3041};
constexpr std::uint16_t SHT3xHeaterOn {0x306D};
constexpr std::uint16_t SHT3xHeaterOff {0x3066};

RecursiveMutex Sht3x::mutexA;
RecursiveMutex Sht3x::mutexB;

static constexpr float from_raw_temperature(std::uint16_t temperature_raw)
{
    // integer version of -45 + 175 * S / (2^16 - 1)
    return ((21875 * temperature_raw >> 13) - 45000) / 1000.f;
}

static constexpr float from_raw_humidity(std::uint16_t humidity_raw)
{
    // integer version of 100 * S / (2^16 - 1)
    return (12500 * humidity_raw >> 13) / 1000.f;
}

static constexpr std::uint16_t to_raw_temperature(float temperature)
{
    // integer version of (T + 45) * (2^16 - 1) / 175
    return (static_cast<int>(1000 * temperature) * 12271 + 552210080) >> 15;
}

static constexpr std::uint16_t to_raw_humidity(float humidity)
{
    // integer version of (2^16 - 1) / 100 * H
    return (static_cast<int>(1000 * humidity) * 21475) >> 15;
}

bool Sht3x::init()
{
    bool ret = SensirionBase::init();

    if (ret) {
        pinMode(_alertPin, INPUT);
        ret = writeCmd(SHT3xBreak);
    }
    return ret;
}

bool Sht3x::singleMeasurement(float &temperature, float &humidity, SingleMode mode)
{
    constexpr auto delay_high {16u};
    constexpr auto delay_medium {7u};
    constexpr auto delay_low {5u};

    std::uint16_t data[2];

    // Acquire device mutex because of long delay between sending meaurement command and receiving data
    const std::lock_guard<RecursiveMutex> lg(_mutex);

    bool ret = writeCmd(static_cast<std::uint16_t>(mode));
    if (!ret) {
        return ret;
    }

    switch (mode) {
        case SingleMode::HighNoClockStretch:
            delay(delay_high);
            break;
        case SingleMode::MediumNoClockStretch:
            delay(delay_medium);
            break;
        case SingleMode::LowNoClockStretch:
            delay(delay_low);
            break;
        default:
            break;
    }

    ret = readWords(data, 2);
    if (ret) {
        temperature = from_raw_temperature(data[0]);
        humidity = from_raw_humidity(data[1]);
    }

    return ret;
}

bool Sht3x::startPeriodicMeasurement(PeriodicMode mode)
{
    return writeCmd(static_cast<std::uint16_t>(mode));
}

bool Sht3x::stopPeriodicMeasurement()
{
    return writeCmd(SHT3xBreak);
}

bool Sht3x::periodicDataRead(float &temperature, float &humidity)
{
    uint16_t data[2];
    bool ret = readCmd(SHT3xPeriodicRead, data, 2);

    if (ret) {
        temperature = from_raw_temperature(data[0]);
        humidity = from_raw_humidity(data[1]);
    }

    return ret;
}

bool Sht3x::setAlertThreshold(AlertThreshold limit, float temperature, float humidity)
{
    std::uint16_t write_cmd {};

    // convert inputs to alert threshold word
    std::uint16_t rawT {to_raw_temperature(temperature)};
    std::uint16_t rawRH {to_raw_humidity(humidity)};
    std::uint16_t limit_val {static_cast<std::uint16_t>((rawRH & 0xfe00u) | ((rawT >> 7) & 0x1ffu))};

    switch (limit) {
        case AlertThreshold::HighSet:
            write_cmd = SHT3xWriteAlertHighSet;
            break;
        case AlertThreshold::HighClear:
            write_cmd = SHT3xWriteAlertHighClear;
            break;
        case AlertThreshold::LowSet:
            write_cmd = SHT3xWriteAlertLowSet;
            break;
        case AlertThreshold::LowClear:
            write_cmd = SHT3xWriteAlertLowClear;
            break;
    }

    if (!writeCmdWithArgs(write_cmd, &limit_val, 1)) {
        driver_log.info("failed to set alert limit");
        return false;
    }

    return true;
}

bool Sht3x::getAlertThreshold(AlertThreshold limit, float &temperature, float &humidity)
{
    std::uint16_t word {};
    std::uint16_t read_cmd {};

    switch (limit) {
        case AlertThreshold::HighSet:
            read_cmd = SHT3xReadAlertHighSet;
            break;
        case AlertThreshold::HighClear:
            read_cmd = SHT3xReadAlertHighClear;
            break;
        case AlertThreshold::LowSet:
            read_cmd = SHT3xReadAlertLowSet;
            break;
        case AlertThreshold::LowClear:
            read_cmd = SHT3xReadAlertLowClear;
            break;
    }

    if (readCmd(read_cmd, &word, 1)) {
        std::uint16_t rawRH {static_cast<std::uint16_t>(word & 0xfe00u)};
        std::uint16_t rawT {static_cast<std::uint16_t>((word & 0x1ffu) << 7)};

        humidity = from_raw_humidity(rawRH);
        temperature = from_raw_temperature(rawT);
    } else {
        driver_log.info("failed to get alert limit");
        return false;
    }

    return true;
}

bool Sht3x::getStatus(std::uint16_t &status)
{
    return readCmd(SHT3xReadStatus, &status, 1);
}

bool Sht3x::clearStatus()
{
    return writeCmd(SHT3xClearStatus);
}

bool Sht3x::heaterOn()
{
    return writeCmd(SHT3xHeaterOn);
}

bool Sht3x::heaterOff()
{
    return writeCmd(SHT3xHeaterOff);
}

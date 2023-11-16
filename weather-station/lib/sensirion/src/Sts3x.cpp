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

#include <cstdint>

#include "Sts3x.h"

// Command words
constexpr std::uint16_t SHT3xReadAlertHighSet {0xE11F};
constexpr std::uint16_t SHT3xReadAlertHighClear {0xE114};
constexpr std::uint16_t SHT3xReadAlertLowSet {0xE109};
constexpr std::uint16_t SHT3xReadAlertLowClear {0xE102};

constexpr std::uint16_t SHT3xWriteAlertHighSet {0x611D};
constexpr std::uint16_t SHT3xWriteAlertHighClear {0x6116};
constexpr std::uint16_t SHT3xWriteAlertLowSet {0x610B};
constexpr std::uint16_t SHT3xWriteAlertLowClear {0x6100};

constexpr std::uint16_t STS3xPeriodicRead {0xE000};
constexpr std::uint16_t STS3xBreak {0x3093};
constexpr std::uint16_t STS3xReadStatus {0xF32D};
constexpr std::uint16_t STS3xClearStatus {0x3041};
constexpr std::uint16_t STS3xHeaterOn {0x306D};
constexpr std::uint16_t STS3xHeaterOff {0x3066};

RecursiveMutex Sts3x::mutexA;
RecursiveMutex Sts3x::mutexB;

static constexpr float from_raw_temperature(std::uint16_t temperature_raw)
{
    // integer version of -45 + 175 * S / (2^16 - 1)
    return ((21875 * temperature_raw >> 13) - 45000) / 1000.f;
}

static constexpr std::uint16_t to_raw_temperature(float temperature)
{
    // integer version of (T + 45) * (2^16 - 1) / 175
    return (static_cast<int>(1000 * temperature) * 12271 + 552210080) >> 15;
}

bool Sts3x::init()
{
    bool ret = SensirionBase::init();

    if (ret) {
        pinMode(_alertPin, INPUT);
        ret = writeCmd(STS3xBreak);
    }
    return ret;
}

bool Sts3x::singleMeasurement(float &temperature, SingleMode mode)
{
    constexpr auto delay_high {16u};
    constexpr auto delay_medium {7u};
    constexpr auto delay_low {5u};

    std::uint16_t data;

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

    ret = readWords(&data, 1);
    if (ret) {
        temperature = from_raw_temperature(data);
    }

    return ret;
}

bool Sts3x::startPeriodicMeasurement(PeriodicMode mode)
{
    return writeCmd(static_cast<std::uint16_t>(mode));
}

bool Sts3x::stopPeriodicMeasurement()
{
    return writeCmd(STS3xBreak);
}

bool Sts3x::periodicDataRead(float &temperature)
{
    uint16_t raw_temp;
    bool ret = readCmd(STS3xPeriodicRead, &raw_temp, 1);

    if (ret) {
        temperature = from_raw_temperature(raw_temp);
    }

    return ret;
}

bool Sts3x::setAlertThreshold(AlertThreshold limit, float temperature)
{
    std::uint16_t write_cmd {};

    // convert inputs to alert threshold word
    std::uint16_t rawT = to_raw_temperature(temperature);
    std::uint16_t limit_val {static_cast<std::uint16_t>((rawT >> 7) & 0x1ffu)};

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
        Log.info("failed to set alert limit");
        return false;
    }

    return true;
}

bool Sts3x::getAlertThreshold(AlertThreshold limit, float &temperature)
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
        std::uint16_t rawT {static_cast<std::uint16_t>((word & 0x1ffu) << 7)};
        temperature = from_raw_temperature(rawT);
    } else {
        Log.info("failed to get alert limit");
        return false;
    }

    return true;
}

bool Sts3x::getStatus(std::uint16_t &status)
{
    return readCmd(STS3xReadStatus, &status, 1);
}

bool Sts3x::clearStatus()
{
    return writeCmd(STS3xClearStatus);
}

bool Sts3x::heaterOn()
{
    return writeCmd(STS3xHeaterOn);
}

bool Sts3x::heaterOff()
{
    return writeCmd(STS3xHeaterOff);
}

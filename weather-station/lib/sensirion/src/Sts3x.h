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
#pragma once

#include <cstdint>

#include "SensirionBase.h"

class Sts3x : public SensirionBase {
public:
    static constexpr std::uint8_t AddrA {0x4au};
    static constexpr std::uint8_t AddrB {0x4bu};

    enum class SingleMode : std::uint16_t {
        HighClockStretch = 0x2C06u,
        MediumClockStretch = 0x2C0Du,
        LowClockStretch = 0x2C10u,
        HighNoClockStretch = 0x2400u,
        MediumNoClockStretch = 0x240Bu,
        LowNoClockStretch = 0x2416u,
    };

    enum class PeriodicMode : std::uint16_t {
        High500mHz = 0x2032u,
        Medium500mHz = 0x2024u,
        Low500mHz = 0x202Fu,
        High1Hz = 0x2130u,
        Medium1Hz = 0x2126u,
        Low1Hz = 0x212Du,
        High2Hz = 0x2236u,
        Medium2Hz = 0x2220u,
        Low2Hz = 0x222Bu,
        High4Hz = 0x2334u,
        Medium4Hz = 0x2322u,
        Low4Hz = 0x2329u,
        High10Hz = 0x2737u,
        Medium10Hz = 0x2721u,
        Low10Hz = 0x272Au,
    };

    enum class AlertThreshold {
        HighSet,
        HighClear,
        LowSet,
        LowClear,
    };

    Sts3x(TwoWire &interface, std::uint8_t address, pin_t alert_pin)
      : SensirionBase(interface, address),
        _alertPin(alert_pin),
        _mutex(address == AddrA ? mutexA : mutexB)
    {}

    /**
     * @brief Initialize the interface
     *
     * @details Attempts to begin I2C transmission of the sensor to
     * validate the sensor can communicate
     *
     * @return true on success, false on failure
     */
    bool init();

    /**
     * @brief Measure and read from an STS sensor the temperature and humidity
     *
     * @details Write to an STS sensor the command to start a measurement, and
     * read from the STS sensor the temperature and humidity. Calls the
     * measure() and read() functions internally
     *
     * @param[out] temperature measured and read in Celsius
     *
     * @return true on success, false on failure
     */
    bool singleMeasurement(float &temperature, SingleMode s_setting = SingleMode::HighNoClockStretch);

    /**
     * @brief Start periodic measurement
     *
     * @details Start periodic temperature and humidity measurements at the
     * commanded repeatability and rate
     *
     * @param[in] mode periodic mode to use
     *
     * @return true on success, false on failure
     */
    bool startPeriodicMeasurement(PeriodicMode);

    /**
     * @brief Stop periodic measurement
     *
     * @details Stop any periodic temperature and humidity measurement in
     * progress
     *
     * @return true on success, false on failure
     */
    bool stopPeriodicMeasurement();

    /**
     * @brief Read a started periodic mode measurement from an STS sensor.
     *
     * @details Read from an STS sensor periodic mode measurement(s) that has
     * already started. Each measurement contains a temperature value.
     * The number of measurements read back depends on the MPS for the peridoc
     * mode setting chosen when the measure() function was called
     *
     * @param[out] temperature contains the data read.
     *
     * @return true on success, false on failure
     */
    bool periodicDataRead(float &temperature);

    /**
     * @brief Set thresholds for alert mode
     *
     * @details Set limits for the alert mode. An alert can be disabled
     * by setting the low set point above the high set point.
     *
     * @param[in] limit the limit to set
     * @param[in] temperature temperature threshold value
     *
     * @return true on success, false on failure
     */
    bool setAlertThreshold(AlertThreshold limit, float temperature);

    /**
     * @brief Get tresholds for alert mode
     *
     * @details Read limits for the alert mode
     *
     * @param[in] limit the limit to read
     * @param[out] temperature temperature threshold value
     *
     * @return true on success, false on failure
     */
    bool getAlertThreshold(AlertThreshold limit, float &temperature);

    /**
     * @brief Read the status register
     *
     * @details Sends a command to read the SHT status register
     *
     * @param[out] status read from the register
     *
     * @return true on success, false on failure
     */
    bool getStatus(std::uint16_t &status);

    /**
     * @brief Clear the status register
     *
     * @details Sends a command to clear the SHT status register
     *
     * @return true on success, false on failure
     */
    bool clearStatus();

    /**
     * @brief Turns the heater on to see plausability of values
     *
     * @details Sends the heater on command
     *
     * @return true on success, false on failure
     */
    bool heaterOn();

    /**
     * @brief Turns the heater off
     *
     * @details Sends the heater off command
     *
     * @return true on success, false on failure
     */
    bool heaterOff();

private:
    pin_t _alertPin;
    RecursiveMutex &_mutex;

    // Use separate mutexes per address
    static RecursiveMutex mutexA;
    static RecursiveMutex mutexB;
};

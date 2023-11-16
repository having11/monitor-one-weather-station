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

#include <cstdint>

#include "SensirionBase.h"

class Sht3x : public SensirionBase {
public:
    static constexpr std::uint8_t AddrA {0x44u};
    static constexpr std::uint8_t AddrB {0x45u};

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

    Sht3x(TwoWire &interface, std::uint8_t address, pin_t alert_pin)
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
     * @brief Measure and read from an SHT sensor the temperature and humidity
     *
     * @details Write to an SHT sensor the command to start a measurement, and
     * read from the SHT sensor the temperature and humidity. Calls the
     * measure() and read() functions internally
     *
     * @param[out] temperature measured and read in Celsius
     * @param[out] humidity measured and read in %
     *
     * @return true on success, false on failure
     */
    bool singleMeasurement(float &temperature, float &humidity, SingleMode s_setting = SingleMode::HighNoClockStretch);

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
     * @brief Read a started periodic mode measurement from an SHT sensor.
     *
     * @details Read from an SHT sensor periodic mode measurement(s) that has
     * already started. Must call the startPeriodicMeasurement() function before
     * calling this function.
     *
     * @param[out] temperature measured and read in Celsius
     * @param[out] humidity measured and read in %
     *
     * @return true on success, false on failure
     */
    bool periodicDataRead(float &temperature, float &humidity);

    /**
     * @brief Set thresholds for alert mode
     *
     * @details Set limits for the alert mode. An alert can be disabled
     * by setting the low set point above the high set point.
     *
     * @param[in] limit the limit to set
     * @param[in] temperature temperature threshold value
     * @param[in] humidity humidity threshold value
     *
     * @return true on success, false on failure
     */
    bool setAlertThreshold(AlertThreshold limit, float temperature, float humidity);

    /**
     * @brief Get tresholds for alert mode
     *
     * @details Read limits for the alert mode
     *
     * @param[in] limit the limit to read
     * @param[out] temperature temperature threshold value
     * @param[out] humidity humidity threshold value
     *
     * @return true on success, false on failure
     */
    bool getAlertThreshold(AlertThreshold limit, float &temperature, float &humidity);

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

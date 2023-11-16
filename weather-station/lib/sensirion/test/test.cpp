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

#define CATCH_CONFIG_MAIN

#include <cmath>

#include "Particle.h"
#include "catch.h"

#include "Sht3x.h"
#include "Sts3x.h"

constexpr int clear_stat_pass_data[] = {0x83, 0xF0, 0x0D};
constexpr int clear_stat_fail_data[] = {0x83, 0xF0, 0x0E};

TEST_CASE("SHT tests")
{
    constexpr int singleshot_pass_data[] = {0x61, 0x21, 0x97, 0x74, 0xF8, 0x02};
    constexpr int singleshot_fail_data[] = {0x61, 0x21, 0x97, 0x74, 0xF8, 0x03};

    float temp;
    float humidity;

    Sht3x device(Wire, Sht3x::AddrA, 255);

    // setup init failure
    Wire.end_transmission_return = endTransmissionReturns::TIMEOUT;
    REQUIRE(device.init() == false);

    // setup init success
    Wire.end_transmission_return = endTransmissionReturns::SUCCESS;
    Wire.num_bytes_to_write = 2;
    REQUIRE(device.init() == true);

    // setup heaterOn failure
    Wire.num_bytes_to_write = 0;
    REQUIRE(device.heaterOn() == false);

    // setup heaterOn success
    Wire.num_bytes_to_write = 2;
    REQUIRE(device.heaterOn() == true);

    // setup heaterOff failure
    Wire.num_bytes_to_write = 0;
    REQUIRE(device.heaterOff() == false);

    // setup heaterOff success
    Wire.num_bytes_to_write = 2;
    REQUIRE(device.heaterOff() == true);

    // setup clearStatus failure
    Wire.num_bytes_to_write = 0;
    REQUIRE(device.clearStatus() == false);

    // setup getStatus success
    Wire.num_bytes_to_write = 2;
    REQUIRE(device.clearStatus() == true);

    // setup startPeriodicMeasurement failure
    Wire.num_bytes_to_write = 0;
    REQUIRE(device.startPeriodicMeasurement(Sht3x::PeriodicMode::High4Hz) == false);

    // setup startPeriodicMeasurement success
    Wire.num_bytes_to_write = 2;
    REQUIRE(device.startPeriodicMeasurement(Sht3x::PeriodicMode::High4Hz) == true);

    // setup stopPeriodicMeasurement failure
    Wire.num_bytes_to_write = 0;
    REQUIRE(device.stopPeriodicMeasurement() == false);

    // setup stopPeriodicMeasurement success
    Wire.num_bytes_to_write = 2;
    REQUIRE(device.stopPeriodicMeasurement() == true);

    // setup getStatus failure 1
    uint16_t dummy_status {};
    Wire.num_bytes_to_write = 0;
    Wire.num_bytes_to_read = 3;
    Wire.data_read = clear_stat_pass_data;
    REQUIRE(device.getStatus(dummy_status) == false);

    // setup getStatus failure 2
    Wire.num_bytes_to_write = 2;
    Wire.num_bytes_to_read = 0;
    Wire.data_read = clear_stat_pass_data;
    REQUIRE(device.getStatus(dummy_status) == false);

    // setup getStatus failure  3
    Wire.num_bytes_to_write = 2;
    Wire.num_bytes_to_read = 3;
    Wire.data_read = clear_stat_fail_data;
    REQUIRE(device.getStatus(dummy_status) == false);

    // setup getStatus success
    Wire.num_bytes_to_write = 2;
    Wire.num_bytes_to_read = 3;
    Wire.data_read = clear_stat_pass_data;
    REQUIRE(device.getStatus(dummy_status) == true);

    // setup singleMeasurement failure 1
    Wire.num_bytes_to_write = 0;
    Wire.num_bytes_to_read = 6;
    Wire.data_read = singleshot_pass_data;
    REQUIRE(device.singleMeasurement(temp, humidity) == false);

    // setup singleMeasurement failure 2
    Wire.num_bytes_to_write = 2;
    Wire.num_bytes_to_read = 5;
    Wire.data_read = singleshot_pass_data;
    REQUIRE(device.singleMeasurement(temp, humidity) == false);

    // setup singleMeasurement failure 3
    Wire.num_bytes_to_write = 2;
    Wire.num_bytes_to_read = 6;
    Wire.data_read = singleshot_fail_data;
    REQUIRE(device.singleMeasurement(temp, humidity) == false);

    // setup singleMeasurement success
    Wire.num_bytes_to_write = 2;
    Wire.num_bytes_to_read = 6;
    Wire.data_read = singleshot_pass_data;
    REQUIRE(device.singleMeasurement(temp, humidity) == true);
    REQUIRE(std::fabs(temp - 21.398f) < 3e-3f);
    REQUIRE(std::fabs(humidity - 45.692f) < 3e-3f);

    // setup periodicDataRead failure 1
    Wire.num_bytes_to_write = 0;
    Wire.num_bytes_to_read = 6;
    Wire.data_read = singleshot_pass_data;
    REQUIRE(device.periodicDataRead(temp, humidity) == false);

    // setup periodicDataRead failure 2
    Wire.num_bytes_to_write = 2;
    Wire.num_bytes_to_read = 5;
    Wire.data_read = singleshot_pass_data;
    REQUIRE(device.periodicDataRead(temp, humidity) == false);

    // setup periodicDataRead failure 3
    Wire.num_bytes_to_write = 2;
    Wire.num_bytes_to_read = 6;
    Wire.data_read = singleshot_fail_data;
    REQUIRE(device.periodicDataRead(temp, humidity) == false);

    // setup periodicDataRead success
    Wire.num_bytes_to_write = 2;
    Wire.num_bytes_to_read = 6;
    Wire.data_read = singleshot_pass_data;
    REQUIRE(device.periodicDataRead(temp, humidity) == true);
    REQUIRE(std::fabs(temp - 21.398f) < 3e-3f);
    REQUIRE(std::fabs(humidity - 45.692f) < 3e-3f);
}

TEST_CASE("STS tests")
{
    constexpr int singleshot_pass_data[] = {0x60, 0xC4, 0x57};
    constexpr int singleshot_fail_data[] = {0x60, 0xC4, 0x56};

    float temp;

    Sts3x device(Wire, Sts3x::AddrA, 255);

    // setup init failure
    Wire.end_transmission_return = endTransmissionReturns::TIMEOUT;
    REQUIRE(device.init() == false);

    // setup init success
    Wire.end_transmission_return = endTransmissionReturns::SUCCESS;
    Wire.num_bytes_to_write = 2;
    REQUIRE(device.init() == true);

    // setup heaterOn failure
    Wire.num_bytes_to_write = 0;
    REQUIRE(device.heaterOn() == false);

    // setup heaterOn success
    Wire.num_bytes_to_write = 2;
    REQUIRE(device.heaterOn() == true);

    // setup heaterOff failure
    Wire.num_bytes_to_write = 0;
    REQUIRE(device.heaterOff() == false);

    // setup heaterOff success
    Wire.num_bytes_to_write = 2;
    REQUIRE(device.heaterOff() == true);

    // setup clearStatus failure
    Wire.num_bytes_to_write = 0;
    REQUIRE(device.clearStatus() == false);

    // setup getStatus success
    Wire.num_bytes_to_write = 2;
    REQUIRE(device.clearStatus() == true);

    // setup startPeriodicMeasurement failure
    Wire.num_bytes_to_write = 0;
    REQUIRE(device.startPeriodicMeasurement(Sts3x::PeriodicMode::High4Hz) == false);

    // setup startPeriodicMeasurement success
    Wire.num_bytes_to_write = 2;
    REQUIRE(device.startPeriodicMeasurement(Sts3x::PeriodicMode::High4Hz) == true);

    // setup stopPeriodicMeasurement failure
    Wire.num_bytes_to_write = 0;
    REQUIRE(device.stopPeriodicMeasurement() == false);

    // setup stopPeriodicMeasurement success
    Wire.num_bytes_to_write = 2;
    REQUIRE(device.stopPeriodicMeasurement() == true);

    // setup getStatus failure 1
    uint16_t dummy_status {};
    Wire.num_bytes_to_write = 0;
    Wire.num_bytes_to_read = 3;
    Wire.data_read = clear_stat_pass_data;
    REQUIRE(device.getStatus(dummy_status) == false);

    // setup getStatus failure 2
    Wire.num_bytes_to_write = 2;
    Wire.num_bytes_to_read = 0;
    Wire.data_read = clear_stat_pass_data;
    REQUIRE(device.getStatus(dummy_status) == false);

    // setup getStatus failure  3
    Wire.num_bytes_to_write = 2;
    Wire.num_bytes_to_read = 3;
    Wire.data_read = clear_stat_fail_data;
    REQUIRE(device.getStatus(dummy_status) == false);

    // setup getStatus success
    Wire.num_bytes_to_write = 2;
    Wire.num_bytes_to_read = 3;
    Wire.data_read = clear_stat_pass_data;
    REQUIRE(device.getStatus(dummy_status) == true);

    // setup singleMeasurement failure 1
    Wire.num_bytes_to_write = 0;
    Wire.num_bytes_to_read = 3;
    Wire.data_read = singleshot_pass_data;
    REQUIRE(device.singleMeasurement(temp) == false);

    // setup singleMeasurement failure 2
    Wire.num_bytes_to_write = 2;
    Wire.num_bytes_to_read = 2;
    Wire.data_read = singleshot_pass_data;
    REQUIRE(device.singleMeasurement(temp) == false);

    // setup singleMeasurement failure 3
    Wire.num_bytes_to_write = 2;
    Wire.num_bytes_to_read = 3;
    Wire.data_read = singleshot_fail_data;
    REQUIRE(device.singleMeasurement(temp) == false);

    // setup singleMeasurement success
    Wire.num_bytes_to_write = 2;
    Wire.num_bytes_to_read = 3;
    Wire.data_read = singleshot_pass_data;
    REQUIRE(device.singleMeasurement(temp) == true);
    REQUIRE(std::fabs(temp - 21.149f) < 3e-3f);

    // setup periodicDataRead failure 1
    Wire.num_bytes_to_write = 0;
    Wire.num_bytes_to_read = 3;
    Wire.data_read = singleshot_pass_data;
    REQUIRE(device.periodicDataRead(temp) == false);

    // setup periodicDataRead failure 2
    Wire.num_bytes_to_write = 2;
    Wire.num_bytes_to_read = 2;
    Wire.data_read = singleshot_pass_data;
    REQUIRE(device.periodicDataRead(temp) == false);

    // setup periodicDataRead failure 3
    Wire.num_bytes_to_write = 2;
    Wire.num_bytes_to_read = 3;
    Wire.data_read = singleshot_fail_data;
    REQUIRE(device.periodicDataRead(temp) == false);

    // setup periodicDataRead success
    Wire.num_bytes_to_write = 2;
    Wire.num_bytes_to_read = 3;
    Wire.data_read = singleshot_pass_data;
    REQUIRE(device.periodicDataRead(temp) == true);
    REQUIRE(std::fabs(temp - 21.149f) < 3e-3f);
}

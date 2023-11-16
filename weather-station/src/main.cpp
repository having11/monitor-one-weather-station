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
 */

#include "Particle.h"
#include "edge.h"
#include "Adafruit_BME680.h"
#include "Adafruit_ADT7410.h"
#include "constants.h"

SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(SEMI_AUTOMATIC);

#if EDGE_PRODUCT_NEEDED
PRODUCT_ID(EDGE_PRODUCT_ID);
#endif // EDGE_PRODUCT_NEEDED
PRODUCT_VERSION(EDGE_PRODUCT_VERSION);

STARTUP(
    Edge::startup();
);

SerialLogHandler logHandler(115200, LOG_LEVEL_TRACE, {
    { "app.gps.nmea", LOG_LEVEL_INFO },
    { "app.gps.ubx",  LOG_LEVEL_INFO },
    { "ncp.at", LOG_LEVEL_INFO },
    { "net.ppp.client", LOG_LEVEL_INFO },
});

constexpr float SeaLevelPressureHPa = 1013.25F;

typedef struct
{
    double temperatureC;
    double relativeHumidity;
    double pressureHPa;
    double gasResistanceKOhms;
    double altitudeM;
    time_t lastWriteTime;
} SensorData_t;

bool readBmeData(SensorData_t *);
void onPrepareForSleep(TrackerSleepContext);
void onDeviceWake(EdgeSleepContext);

Adafruit_BME680 bme;
SensorData_t data;
Adafruit_ADT7410 tempsensor;

void setup()
{
    Edge::instance().init();

    if (!bme.begin())
    {
        Log.error("Failed to start BME680");
        Particle.publish("Log", "Failed to start BME680");
    }

    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150); // 320*C for 150 ms

    if (!tempsensor.begin())
    {
        Log.error("Failed to start ADT7410");
        Particle.publish("Log", "Failed to start ADT7410");
    }

    Log.info("SENSORS INITIALIZED");

    TrackerSleep::instance().registerSleepPrepare(onPrepareForSleep);
    TrackerSleep::instance().registerWake(onDeviceWake);

    Particle.publish("Ready", "System is ready");
}

void loop()
{
    Edge::instance().loop();
}

bool readSensorData(SensorData_t *senData)
{
    if (!bme.performReading())
    {
        return false;
    }

    senData->temperatureC = bme.temperature;
    senData->relativeHumidity = bme.humidity;
    senData->pressureHPa = bme.pressure / 100.0;
    senData->gasResistanceKOhms = bme.gas_resistance / 1000.0;
    senData->altitudeM = bme.readAltitude(SeaLevelPressureHPa);

    LocationPoint point;
    Edge::instance().locationService.getLocation(point);
    senData->lastWriteTime = point.epochTime;

    float accurateTemperatureC = tempsensor.readTempC();
    senData->temperatureC = accurateTemperatureC;

    return true;
}

void onPrepareForSleep(TrackerSleepContext context)
{
    TrackerSleep::instance().wakeAtMilliseconds(System.millis() + WeatherStationConstants::sleepTime.count());
}

void onDeviceWake(EdgeSleepContext context)
{
    if (context.reason == EdgeSleepReason::WAKE &&
        TrackerSleep::instance().isFullWakeCycle())
    {
        readSensorData(&data);
        String formattedData = String::format(
        "{"
            "\"temperatureInC\":%.2f,"
            "\"humidityPercentage\":%.2f,"
            "\"pressureHpa\":%.2f,"
            "\"gasResistanceKOhms\":%.2f,"
            "\"approxAltitudeInM\":%.2f,"
            "\"lastGoodReading\": %ld"
        "}",
        data.temperatureC,
        data.relativeHumidity,
        data.pressureHPa,
        data.gasResistanceKOhms,
        data.altitudeM,
        data.lastWriteTime);

        Particle.publish("SensorReadings", formattedData);
    }
}
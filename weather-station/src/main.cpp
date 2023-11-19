// Uncomment for detailed BME680 debug info
// #define BME680_DEBUG

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

bool readSensorData(SensorData_t *);
void myLocationGenerationCallback(JSONWriter &writer, LocationPoint &point, const void *context);

Adafruit_BME680 bme;
SensorData_t data = { .temperatureC = -1.0f };
Adafruit_ADT7410 tempsensor;

bool failure;

void setup()
{
    Serial.begin(115200);
    Wire.begin();
    Edge::instance().init();

    TrackerLocation::instance().regLocGenCallback(myLocationGenerationCallback);

    if (!bme.begin())
    {
        Log.error("Failed to start BME680");
        Particle.publish("Log", "Failed to start BME680");
        failure = true;
    }

    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150); // 320*C for 150 ms

    Particle.variable("fail", &failure, BOOLEAN);

    if (!tempsensor.begin())
    {
        Log.error("Failed to start ADT7410");
        Particle.publish("Log", "Failed to start ADT7410");
    }

    Log.info("SENSORS INITIALIZED");
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

    return true;
}

void myLocationGenerationCallback(JSONWriter &writer, LocationPoint &point, const void *context)
{
    readSensorData(&data);
    writer.name("weather").beginObject();
    writer.name("temperatureC").value(data.temperatureC);
    writer.name("humidity").value(data.relativeHumidity);
    writer.name("pressureHPa").value(data.pressureHPa);
    writer.name("gasResKOhm").value(data.gasResistanceKOhms);
    writer.name("altitudeM").value(data.altitudeM);
    writer.endObject();
}
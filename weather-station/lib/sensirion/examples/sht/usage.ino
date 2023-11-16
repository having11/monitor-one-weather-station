#include "Sht3x.h"

#define USE_SINGLE_SHOT

SerialLogHandler logHandler(
  115200,
  LOG_LEVEL_ALL,
  {
    // { "app", LOG_LEVEL_ALL },
    // { "hal", LOG_LEVEL_NONE},
    // { "lwip", LOG_LEVEL_NONE},
  }
);

STARTUP(System.enableFeature(FEATURE_RESET_INFO);
        System.enableFeature(FEATURE_RETAINED_MEMORY););

SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(SEMI_AUTOMATIC);

Sht3x sht(Wire, Sht3x::AddrA, 255);

void setup()
{
    Wire.setSpeed(CLOCK_SPEED_100KHZ);
    Wire.begin();
    if (!sht.init()) {
        Log.error("failed to initialize device, halting");
        while (true) {}
    }

#if !defined(USE_SINGLE_SHOT)
    sht.startPeriodicMeasurement(Sht3x::PeriodicMode::High10Hz);
#endif // !defined(USE_SINGLE_SHOT)
}

void loop()
{
    static uint32_t read_time_ms = millis();

    if (millis() - read_time_ms >= 1000) {
        float temp;
        float humidity;
#if defined(USE_SINGLE_SHOT)
        if (sht.singleMeasurement(temp, humidity)) {
            Log.info("temp:%f, humidity:%f", temp, humidity);
        } else {
            Log.error("failed to read temp and humidity");
        }
#else
        if (sht.periodicDataRead(temp, humidity)) {
            Log.info("temp:%f, humidity:%f", temp, humidity);
        } else {
            Log.error("failed to read periodic temp and humidity");
        }
#endif // defined(USE_SINGLE_SHOT)
        read_time_ms = millis();
    }
}

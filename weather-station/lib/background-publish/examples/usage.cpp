/*
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

#include "BackgroundPublish.h"

constexpr uint32_t TIMEOUT_SEC = 1000;
constexpr unsigned int APP_VERSION = 1;

SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(SEMI_AUTOMATIC);

PRODUCT_ID(PLATFORM_ID);
PRODUCT_VERSION(APP_VERSION);

void priority_high_cb(publishStatus status,
    const char *event_name,
    const char *event_data,
    const void *event_context);

void priority_low_cb(publishStatus status,
    const char *event_name,
    const char *event_data,
    const void *event_context);

SerialLogHandler logHandler(115200, LOG_LEVEL_ALL, {
});

void setup() {
    BackgroundPublish::instance().init();
    Particle.connect();
}

void loop() {
    static int counter = 0;
    static system_tick_t timer_start_ms = millis();
    String* str;

    if(millis() - timer_start_ms > TIMEOUT_SEC) {
        auto format = String::format("Counter:%d", counter);
        str = new (std::nothrow) String(format);
        if(Particle.connected()) {
            if(!(counter % 2)) {
                if(!BackgroundPublish::instance().publish("TEST_PUB_HIGH", 
                                    str->c_str(), 
                                    PRIVATE,
                                    0, 
                                    priority_high_cb )) {
                    Log.info("Failed publish request");
                }
            }
            else {
                if(!BackgroundPublish::instance().publish("TEST_PUB_LOW", 
                                    str->c_str(), 
                                    PRIVATE,
                                    1, 
                                    priority_low_cb )) {
                    Log.info("Failed publish request");
                }
            }
            counter++;
        }
        else {
            Log.info("Not connected to cloud");
        }
        timer_start_ms = millis();
        //cleanup any unsent data after 100
        if(counter > 100) {
            BackgroundPublish::instance().cleanup();
            counter = 0;
        }
    }
}

void priority_high_cb(publishStatus status,
    const char *event_name,
    const char *event_data,
    const void *event_context) {
    
    delete event_data;
    Log.info("High callback fired, cleaned up memory");
}

void priority_low_cb(publishStatus status,
    const char *event_name,
    const char *event_data,
    const void *event_context) {

    delete event_data;
    Log.info("Low callback fired, cleaned up memory");
}
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

#include "edge_motion_configuration.h"
#include "edge_location.h"
#include "edge_sleep.h"

#include "config_service.h"
#include "edge_motion.h"

EdgeMotionConfiguration *EdgeMotionConfiguration::_instance = nullptr;

static int get_motion_enabled_cb(int32_t &value, const void *context)
{
    value = (int32_t) static_cast<EdgeMotion *>((void *)context)->getMotionDetection();
    return 0;
}

static int set_motion_enabled_cb(int32_t value, const void *context)
{
    EdgeMotion *motion_service = static_cast<EdgeMotion *>((void *)context);

    motion_service->enableMotionDetection((MotionDetectionMode) value);
    if ((MotionDetectionMode)value == MotionDetectionMode::NONE) {
        if (!motion_service->isAnyAwake()) {
            EdgeSleep::instance().ignore((pin_t)BMI_INT_PIN);
        }
    }
    else {
        EdgeSleep::instance().wakeFor((pin_t)BMI_INT_PIN, BMI_INT_MODE);
    }
    return 0;
}

static int get_high_g_enabled_cb(int32_t &value, const void *context)
{
    value = (int32_t) static_cast<EdgeMotion *>((void *)context)->getHighGDetection();
    return 0;
}

static int set_high_g_enabled_cb(int32_t value, const void *context)
{
    EdgeMotion *motion_service = static_cast<EdgeMotion *>((void *)context);

    if(value == (int32_t) HighGDetectionMode::DISABLE)
    {
        motion_service->disableHighGDetection();
        if (!motion_service->isAnyAwake()) {
            EdgeSleep::instance().ignore((pin_t)BMI_INT_PIN);
        }
    }
    else if(value == (int32_t) HighGDetectionMode::ENABLE)
    {
        motion_service->enableHighGDetection();
        EdgeSleep::instance().wakeFor((pin_t)BMI_INT_PIN, BMI_INT_MODE);
    }
    else
    {
        return -EINVAL;
    }

    return 0;
}

void EdgeMotionConfiguration::init()
{
    static ConfigObject imu_desc
    (
        "imu_trig",
        {
            ConfigStringEnum(
                "motion",
                {
                    {"disable", (int32_t) MotionDetectionMode::NONE},
                    {"low", (int32_t) MotionDetectionMode::LOW_SENSITIVITY},
                    {"medium", (int32_t) MotionDetectionMode::MEDIUM_SENSITIVITY},
                    {"high", (int32_t) MotionDetectionMode::HIGH_SENSITIVITY},
                },
                get_motion_enabled_cb,
                set_motion_enabled_cb,
                &EdgeMotion::instance()
            ),
            ConfigStringEnum(
                "high_g",
                {
                    {"disable", (int32_t) HighGDetectionMode::DISABLE},
                    {"enable", (int32_t) HighGDetectionMode::ENABLE},
                },
                get_high_g_enabled_cb,
                set_high_g_enabled_cb,
                &EdgeMotion::instance()
            ),
        }
    );

    ConfigService::instance().registerModule(imu_desc);
}

void EdgeMotionConfiguration::loop()
{
    MotionEvent motion_event;
    size_t depth = EdgeMotion::instance().getQueueDepth();

    do {
        EdgeMotion::instance().waitOnEvent(motion_event, 0);
        switch (motion_event.source)
        {
            case MotionSource::MOTION_HIGH_G:
                EdgeLocation::instance().triggerLocPub(Trigger::NORMAL, "imu_g");
                break;
            case MotionSource::MOTION_MOVEMENT:
                EdgeLocation::instance().triggerLocPub(Trigger::NORMAL,"imu_m");
                break;
        }
    } while (--depth && (motion_event.source != MotionSource::MOTION_NONE));
}

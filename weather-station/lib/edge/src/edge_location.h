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

#pragma once

#include "config_service.h"
#include "cloud_service.h"
#include "edge_gnss_abstraction.h"
#include "edge_motion.h"
#include "edge_sleep.h"
#include "Geofence.h"

#define EDGE_LOCATION_INTERVAL_MIN_DEFAULT_SEC (900)
#define EDGE_LOCATION_INTERVAL_MAX_DEFAULT_SEC (3600)
#define EDGE_LOCATION_MIN_PUBLISH_DEFAULT (false)
#define EDGE_LOCATION_LOCK_TRIGGER (true)
#define EDGE_LOCATION_PROCESS_ACK (true)

// wait at most this many seconds for a locked GPS location to become stable
// before publishing regardless
#define EDGE_LOCATION_STABLE_WAIT_MAX (30)

// wait at most this many seconds for initial lock on boot before publishing
// regardless
#define EDGE_LOCATION_INITIAL_LOCK_MAX (90)

constexpr int EdgeLocationMaxWpsCollect = 20;
constexpr int EdgeLocationMaxWpsSend = 5;
constexpr int EdgeLocationMaxTowerSend = 3;
constexpr int NUM_OF_GEOFENCE_ZONES = 4;

struct edge_location_config_t {
    int32_t interval_min_seconds; // 0 = no min
    int32_t interval_max_seconds; // 0 = no max
    bool min_publish;
    bool lock_trigger;
    bool process_ack;
    bool tower;
    bool gnss;
    bool wps;
    bool enhance_loc;
    bool loc_cb;
    bool diag;
};

enum class Trigger {
    NORMAL = 0,
    IMMEDIATE = 1,
};

enum class GnssState {
    OFF,
    ERROR,
    ON_UNLOCKED,
    ON_LOCKED_UNSTABLE,
    ON_LOCKED_STABLE,
    DISABLED,
};

enum class PublishReason {
    NONE,
    TIME,
    TRIGGERS,
    IMMEDIATE,
};

struct EvaluationResults {
    PublishReason reason;
    bool networkNeeded;
    bool lockWait;
};

struct EdgeGeofenceConfig {
    int32_t interval; // seconds
};

class EdgeLocation
{
    public:
        /**
         * @brief Return instance of the tracker location object
         *
         * @retval CloudService&
         */
        static EdgeLocation &instance()
        {
            if(!_instance)
            {
                _instance = new EdgeLocation();
            }
            return *_instance;
        }

        /**
         * @brief Initialize the EdgeLocation object
         *
         * @param gnssRetries GNSS initialization retry count
         */
        void init(unsigned int gnssRetries);

        void loop();

        // register for callback during generation of location publish allowing
        // for insertion of custom fields into the output
        // these callbacks are persistent and not removed on generation
        int regLocGenCallback(
            std::function<void(JSONWriter&, LocationPoint &, const void *)>,
            const void *context=nullptr);

        template <typename T>
        int regLocGenCallback(
            void (T::*cb)(JSONWriter&, LocationPoint &, const void *),
            T *instance,
            const void *context=nullptr);

        // register for callback on location publish success/fail
        // these callbacks are NOT persistent and are used for the next publish
        int regLocPubCallback(
            std::function<int(CloudServiceStatus, const String&)> cb);

        template <typename T>
        int regLocPubCallback(
            int (T::*cb)(CloudServiceStatus status, const String&),
            T *instance);

        int regPendLocPubCallback(std::function<int(CloudServiceStatus, const String&)> cb);

        template <typename T>
        int regPendLocPubCallback(
            int (T::*cb)(CloudServiceStatus status, const String&),
            T *instance);

        // register for callback after location publish for the cloud supplied ehanced callback
        // these callbacks are persistent and not removed on generation
        int regEnhancedLocCallback(
            std::function<void(const LocationPoint&, const void *)>,
            const void *context=nullptr);

        template <typename T>
        int regEnhancedLocCallback(
            void (T::*cb)(const LocationPoint&, const void *),
            T *instance,
            const void *context=nullptr);

        int triggerLocPub(Trigger type = Trigger::NORMAL, const char *s = "user");

        void lock() {mutex.lock();}
        void unlock() {mutex.unlock();}

        inline bool getMinPublish() { return _config_state.min_publish; }

        int addWap(WiFiAccessPoint* wap);

        Geofence& getGeoFence() {
            return _geofence;
        }
        bool isProcessAckEnabled() {return _config_state.process_ack;}
        int location_publish_cb(CloudServiceStatus status, String&& req_event, std::uint32_t last_publish_time);
        void issue_location_publish_callbacks(CloudServiceStatus status, const String &req_event);

    private:
        EdgeLocation() :
            _sleep(EdgeSleep::instance()),
            _geofence(NUM_OF_GEOFENCE_ZONES),
            _loopSampleTick(0),
            _pending_immediate(false),
            _first_publish(true),
            _pending_first_publish(false),
            _pendingShutdown(false),
            _earlyWake(0),
            _nextEarlyWake(0),
            _pendingGeofence(false),
            _lastInterval(0),
            _publishAttempted(0),
            _monotonic_publish_sec(0),
            _newMonotonic(true),
            _firstLockSec(0),
            _gnssStartedSec(0),
            _lastGnssState(GnssState::OFF),
            _gnssRetryDefault(0),
            _gnssCycleCurrent(0) {

            _config_state = {
                .interval_min_seconds = EDGE_LOCATION_INTERVAL_MIN_DEFAULT_SEC,
                .interval_max_seconds = EDGE_LOCATION_INTERVAL_MAX_DEFAULT_SEC,
                .min_publish = EDGE_LOCATION_MIN_PUBLISH_DEFAULT,
                .lock_trigger = EDGE_LOCATION_LOCK_TRIGGER,
                .process_ack = EDGE_LOCATION_PROCESS_ACK,
                .tower = true,
                .gnss = true,
                .wps = true,
                .enhance_loc = true,
                .loc_cb = false,
                .diag = false,
            };

            _config_state_loop_safe = _config_state;
        }
        static EdgeLocation *_instance;
        EdgeSleep& _sleep;
        Geofence _geofence;

        RecursiveMutex mutex;

        Vector<const char *> _pending_triggers;
        system_tick_t _loopSampleTick;
        bool _pending_immediate;
        bool _first_publish;
        bool _pending_first_publish;
        bool _pendingShutdown;
        unsigned int _earlyWake;
        unsigned int _nextEarlyWake;
        EdgeGeofenceConfig _geofenceConfig {};
        bool _pendingGeofence;

        int enter_location_config_cb(bool write, const void *context);
        int exit_location_config_cb(bool write, int status, const void *context);

        int get_loc_cb(JSONValue *root);

        void location_publish();

        bool isSleepEnabled();
        void enableNetwork();
        int enableGnss();
        int disableGnss();
        void onSleepPrepare(EdgeSleepContext context);
        void onSleep(EdgeSleepContext context);
        void onSleepCancel(EdgeSleepContext context);
        void onWake(EdgeSleepContext context);
        void onSleepState(EdgeSleepContext context);
        void onGeofenceCallback(CallbackContext& context);
        EvaluationResults evaluatePublish(bool error);
        void buildPublish(LocationPoint& cur_loc, bool error = false);
        GnssState loopLocation(LocationPoint& cur_loc);
        size_t buildTowerInfo(JSONBufferWriter& writer, size_t size);
        static void wifi_cb(WiFiAccessPoint* wap, EdgeLocation* context);
        size_t buildWpsInfo(JSONBufferWriter& writer, size_t size);

        int buildEnhLocation(JSONValue& node, LocationPoint& point);
        int enhanced_cb(JSONValue* root);

        unsigned int setGnssCycle() {
            return _gnssCycleCurrent = _gnssRetryDefault + 1; // Initial attempt plus retries
        }

        unsigned int getGnssCycle() const {
            return _gnssCycleCurrent;
        }

        unsigned int decGnssCycle() {
            if (0 != _gnssCycleCurrent) {
                _gnssCycleCurrent--;
            }
            return _gnssCycleCurrent;
        }

        uint32_t _last_location_publish_sec;
        int32_t _lastInterval;
        std::atomic<size_t> _publishAttempted;
        uint32_t _monotonic_publish_sec;
        bool _newMonotonic;
        uint32_t _firstLockSec;
        uint32_t _gnssStartedSec;
        GnssState _lastGnssState;
        unsigned int _gnssRetryDefault;
        unsigned int _gnssCycleCurrent;

        edge_location_config_t _config_state, _config_state_shadow, _config_state_loop_safe;

        Vector<std::function<void(JSONWriter&, LocationPoint&)>> locGenCallbacks;
        // publish callback for the next publish (not in flight)
        Vector<std::function<void(CloudServiceStatus status, const String&)>> locPubCallbacks;
        // publish callbacks for the current/pending publish (in flight)
        Vector<std::function<void(CloudServiceStatus status, const String&)>> pendingLocPubCallbacks;
        // publish callbacks for the enhanced location callback
        Vector<std::function<void(const LocationPoint&)>> enhancedLocCallbacks;
        os_queue_t _enhancedLocQueue;

        Vector<WiFiAccessPoint> wpsList;
};

template <typename T>
int EdgeLocation::regLocGenCallback(
    void (T::*cb)(JSONWriter&, LocationPoint &, const void *),
    T *instance,
    const void *context)
{
    return regLocGenCallback(std::bind(cb, instance, _1, _2), context);
}

template <typename T>
int EdgeLocation::regLocPubCallback(
    int (T::*cb)(CloudServiceStatus status, const String &),
    T *instance)
{
    return regLocPubCallback(std::bind(cb, instance, _1, _2));
}

template <typename T>
int EdgeLocation::regPendLocPubCallback(
    int (T::*cb)(CloudServiceStatus status, const String &),
    T *instance)
{
    return regPendLocPubCallback(std::bind(cb, instance, _1, _2));
}

template <typename T>
int EdgeLocation::regEnhancedLocCallback(
    void (T::*cb)(const LocationPoint&, const void*),
    T* instance,
    const void* context)
{
    return regEnhancedLocCallback(std::bind(cb, instance, _1), context);
}

// These are here for compatibility
using tracker_location_config_t = edge_location_config_t;
using TrackerGeofenceConfig = EdgeGeofenceConfig;
using TrackerLocation = EdgeLocation;

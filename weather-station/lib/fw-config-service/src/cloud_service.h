/*
 * Copyright (c) 2020 Particle Industries, Inc.
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

#include "Particle.h"
#include "BackgroundPublish.h"

// default name for incoming Particle.function
#define CLOUD_DEFAULT_FUNCTION_NAME "cmd"

// JSON keys for common fields, not overridable
#define CLOUD_KEY_CMD "cmd"
#define CLOUD_KEY_TIME "time"
#define CLOUD_KEY_REQ_ID "req_id"
#define CLOUD_KEY_SRC_CMD "src_cmd"

// built-in supported JSON commands, user may add more but cannot overlap
#define CLOUD_CMD_SYNC "sync"
#define CLOUD_CMD_ACK "ack"
#define CLOUD_CMD_CFG "cfg"

#define CLOUD_MAX_CMD_LEN (32)
#define CLOUD_PUB_PREFIX ""

#define CLOUD_DEFAULT_TIMEOUT_MS (10000)

#include <cstddef>
#include <functional>
#include <limits>
#include <list>
#include <utility>

enum class CloudServiceStatus {
    SUCCESS = 0,
    FAILURE, // publish to Particle cloud failed, etc
    TIMEOUT, // waiting for application response, etc
};

enum CloudServicePublishFlags {
    NONE = 0x00, // no special flags
    FULL_ACK = 0x01 // full end-to-end acknowledgement
};

using cloud_service_ack_callback = std::function<int(CloudServiceStatus, String&&)>;

template<typename T>
using cloud_service_ack_callback_ptmf = int (T::*)(CloudServiceStatus, String&&);

struct cloud_service_ack_context {
    std::uint32_t req_id;
    system_tick_t timeout; // absolute time of timeout, compared against millis()
    cloud_service_ack_callback callback;
    String data; // copy of original payload
};

class CloudService
{
    public:
        /**
         * @brief Return instance of the cloud service
         *
         * @retval CloudService&
         */
        static CloudService &instance()
        {
            if(!_instance)
            {
                _instance = new CloudService();
            }
            return *_instance;
        }

        void init(const char *cmd=nullptr);

        // process quick actions
        void tick();

        // starts a new command/ack
        int beginCommand(const char *cmd);
        int beginResponse(const char *cmd, JSONValue &root);

        int send(const char *data,
            PublishFlags publish_flags = PRIVATE,
            CloudServicePublishFlags cloud_flags = CloudServicePublishFlags::NONE,
            cloud_service_ack_callback cb=nullptr,
            unsigned int timeout_ms=std::numeric_limits<system_tick_t>::max(),
            const char *event_name=nullptr,
            uint32_t req_id=0,
            std::size_t priority=0u);

        int send(PublishFlags publish_flags = PRIVATE,
            CloudServicePublishFlags cloud_flags = CloudServicePublishFlags::NONE,
            cloud_service_ack_callback cb=nullptr,
            unsigned int timeout_ms=std::numeric_limits<system_tick_t>::max(),
            std::size_t priority=0u);

        template <typename T>
        int send(PublishFlags publish_flags = PRIVATE,
            CloudServicePublishFlags cloud_flags = CloudServicePublishFlags::NONE,
            cloud_service_ack_callback_ptmf<T> cb=nullptr,
            T *instance=nullptr,
            uint32_t timeout_ms=std::numeric_limits<system_tick_t>::max(),
            std::size_t priority=0u)
        {
            return send(publish_flags, cloud_flags, std::bind(cb, instance, std::placeholders::_1, std::placeholders::_2), timeout_ms, priority);
        }

        template <typename T>
        int send(const char *data,
            PublishFlags publish_flags = PRIVATE,
            CloudServicePublishFlags cloud_flags = CloudServicePublishFlags::NONE,
            cloud_service_ack_callback_ptmf<T> cb=nullptr,
            T *instance=nullptr,
            uint32_t timeout_ms=std::numeric_limits<system_tick_t>::max(),
            const char *event_name=nullptr,
            uint32_t req_id=0,
            std::size_t priority=0u)
        {
            return send(data, publish_flags, cloud_flags, std::bind(cb, instance, std::placeholders::_1, std::placeholders::_2), timeout_ms, event_name, req_id, priority);
        }

        int sendAck(JSONValue &root, int status);

        JSONBufferWriter &writer() { return _writer; };

        void lock() {mutex.lock();}
        void unlock() {mutex.unlock();}

        // process and dispatch incoming commands to registered callbacks
        int dispatchCommand(String cmd);

        int registerCommand(const char *name, std::function<int(JSONValue *)> handler);

    private:
        CloudService();
        static CloudService *_instance;

        BackgroundPublish<> background_publish;

        int registerAckCallback(cloud_service_ack_context&&);

        // process infrequent actions
        void tick_sec();

        uint32_t get_next_req_id();

        char json_buf[particle::protocol::MAX_EVENT_DATA_LENGTH + 1];
        JSONBufferWriter _writer;
        char _writer_event_name[sizeof(CLOUD_PUB_PREFIX) + CLOUD_MAX_CMD_LEN];

        // iterate req_id on each send
        uint32_t _req_id;

        uint32_t last_tick_sec;

        std::list<cloud_service_ack_context> ack_handlers;
        std::list<std::pair<String, std::function<int(JSONValue *)>>> command_handlers;
        std::list<std::function<int()>> deferred_acks;

        RecursiveMutex mutex;
};

void log_json(const char *json, size_t size);

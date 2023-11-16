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

#include "Particle.h"

#include "cloud_service.h"

#include "BackgroundPublish.h"

#include <limits>
#include <memory>
#include <string.h>

// shared_function<F> allows a move-only function to be (unsafely) passed as a std::function<F>
// until C++23 makes std::move_only_function available.
// Calling code is responsible for ensuring it is always passed by rvalue reference.
template<typename F>
struct shared_function {
    std::shared_ptr<F> f;
    shared_function() = default;
    shared_function(F&& f_): f(std::make_shared<F>(std::forward<F>(f_))) {}
    shared_function(const shared_function&) = default;
    shared_function(shared_function&&) = default;
    shared_function& operator=(const shared_function&) = default;
    shared_function& operator=(shared_function&&) = default;

    template<typename... Args>
    auto operator()(Args&&... args) const {
        return (*f)(std::forward<Args>(args)...);
    }
};
template<typename F>
shared_function<std::decay_t<F>> make_shared_function(F&& f) {
    return std::forward<F>(f);
}

CloudService *CloudService::_instance = nullptr;

CloudService::CloudService() :
     background_publish(), _writer(json_buf, sizeof(json_buf)), _req_id(1)
{
}

void CloudService::init(const char *cmd)
{
    // use default function name if not provided on init
    Particle.function(cmd ? cmd : CLOUD_DEFAULT_FUNCTION_NAME,
        &CloudService::dispatchCommand, this);
    background_publish.start();
}

void CloudService::tick()
{
    auto sec = System.uptime();
    std::lock_guard<RecursiveMutex> lg(mutex);

    if(sec != last_tick_sec)
    {
        last_tick_sec = sec;
        tick_sec();
    }

    for(auto &ack: deferred_acks)
    {
        ack();
    }
    deferred_acks.clear();
}

void CloudService::tick_sec()
{
    uint32_t ms_now = millis();

    // timeout ack handlers
    for (auto it = ack_handlers.begin(); it != ack_handlers.end();) {
        if (ms_now > it->timeout) {
            it->callback(CloudServiceStatus::TIMEOUT, std::move(it->data));
            it = ack_handlers.erase(it);
        } else {
            ++it;
        }
    }
}

 uint32_t CloudService::get_next_req_id()
 {
    auto req_id = _req_id;
    if(!++_req_id)
    {
        _req_id = 1;
    }
    return req_id;
}

int CloudService::registerCommand(const char *cmd, std::function<int(JSONValue *)> handler)
{
    std::lock_guard<RecursiveMutex> lg(mutex);

    if (!cmd || (strnlen(cmd, 1 + CLOUD_MAX_CMD_LEN) > CLOUD_MAX_CMD_LEN) || !handler) {
        return -EINVAL;
    }
    command_handlers.push_back(std::make_pair(String(cmd), handler));
    return 0;
}

int CloudService::registerAckCallback(cloud_service_ack_context&& context)
{
    std::lock_guard<RecursiveMutex> lg(mutex);
    ack_handlers.push_back(std::move(context));
    return 0;
}

static int _get_common_fields(JSONValue &root, const char **cmd, const char **src_cmd, uint32_t *req_id, uint32_t *timestamp)
{
    int rval = 0;

    // iterate and peel out necessary fields for command dispatching
    JSONObjectIterator it(root);
    while(it.next())
    {
        if (!it.value().isValid())
        {
            rval = -EINVAL;
            break;
        }

        const char *it_name = (const char *) it.name();
        if(!strcmp(CLOUD_KEY_CMD, (const char *) it_name))
        {
            if(!it.value().isString())
            {
                rval = -EINVAL;
                break;
            }
            if(cmd)
            {
                *cmd = (const char *) it.value().toString();
            }
        }
        else if(!strcmp(CLOUD_KEY_SRC_CMD, (const char *) it_name))
        {
            if(!it.value().isString())
            {
                rval = -EINVAL;
                break;
            }
            if(src_cmd)
            {
                *src_cmd = (const char *) it.value().toString();
            }
        }
        else if(!strcmp(CLOUD_KEY_REQ_ID, (const char *) it_name))
        {
            if(!it.value().isNumber())
            {
                rval = -EINVAL;
                break;
            }
            if(req_id)
            {
                *req_id = it.value().toInt();
            }
        }
        else if(!strcmp(CLOUD_KEY_TIME, (const char *) it_name))
        {
            if(!it.value().isNumber())
            {
                rval = -EINVAL;
                break;
            }
            if(timestamp)
            {
                *timestamp = it.value().toInt();
            }
        }
    }

    if(!*cmd)
    {
        return -EINVAL;
    }

    return rval;
}

int CloudService::dispatchCommand(String data)
{
    Log.info("cloud received: %s", data.c_str());
    JSONValue root = JSONValue::parseCopy(data, data.length());
    int rval = -ENOENT;

    const char *cmd = nullptr;
    uint32_t req_id=0, timestamp=0;

    // for now we are expecting a full json object
    // in future we may accept non-json objects and process separately
    if(!root.isObject() || !root.isValid())
    {
        return -EINVAL;
    }

    _get_common_fields(root, &cmd, nullptr, &req_id, &timestamp);

    if(!cmd)
    {
        return -EINVAL;
    }
 
    std::lock_guard<RecursiveMutex> lg(mutex);

    for (auto& [cmd_name, handler]: command_handlers) {
        if (cmd_name == cmd) {
            return handler(&root);
        }
    }

    // Process ack messages
    if (strncmp(cmd, "ack", 1 + sizeof("ack"))) {
        return -ENOENT;
    }
    for (auto it = ack_handlers.begin(); it != ack_handlers.end();) {
        if (req_id == it->req_id) {
            rval = it->callback(CloudServiceStatus::SUCCESS, std::move(it->data));
            it = ack_handlers.erase(it);
        } else {
            ++it;
        }
    }

    return rval;
}

int CloudService::beginCommand(const char *cmd)
{
    // hold lock for duration between begin_command/send as the json buffer is
    // a singular shared resource
    // calling processes should not unnecessarily delay when formatting the
    // output command to not unnecessarily block other processes and should not
    // access other external resources that may result in a deadlock
    // (for example, don't begin_command and THEN read off a register from an
    // I2C device in order to format into the output command)
    mutex.lock();

    _writer = JSONBufferWriter(json_buf, sizeof(json_buf)); // reset the output

    writer().beginObject();
    writer().name(CLOUD_KEY_CMD).value(cmd);
    snprintf(_writer_event_name, sizeof(_writer_event_name), CLOUD_PUB_PREFIX "%s", cmd);
    writer().name(CLOUD_KEY_TIME).value((unsigned int) Time.now());

    return 0;
}

int CloudService::beginResponse(const char *cmd, JSONValue &root)
{
    const char *src_cmd = nullptr;
    uint32_t req_id = 0;

    if(!root.isObject())
    {
        return -EINVAL;
    }

    _get_common_fields(root, &src_cmd, nullptr, &req_id, nullptr);

    if(!src_cmd || !req_id)
    {
        return -EINVAL;
    }

    beginCommand(cmd);

    writer().name(CLOUD_KEY_REQ_ID).value((unsigned int) req_id);
    writer().name(CLOUD_KEY_SRC_CMD).value(src_cmd);

    return 0;
}

int CloudService::send(const char *data,
    PublishFlags publish_flags,
    CloudServicePublishFlags cloud_flags,
    cloud_service_ack_callback cb,
    unsigned int timeout_ms,
    const char *event_name,
    uint32_t req_id,
    std::size_t priority)
{
    int rval = 0;
    size_t data_len = strlen(data);
    std::lock_guard<RecursiveMutex> lg(mutex);

    if(!event_name ||
        (!req_id && cb && (cloud_flags & CloudServicePublishFlags::FULL_ACK)))
    {
        // should have request id or event name but it wasn't passed in
        // extract from event
        JSONValue root = JSONValue::parseCopy(data, data_len);
        _get_common_fields(root, &event_name, nullptr, &req_id, nullptr);

        if(!event_name)
        {
            Log.info("Event Name failed: %s", data);
            return -EINVAL;
        }
    }    

    strlcpy(_writer_event_name, event_name, sizeof(_writer_event_name));

    // much simpler if there is no callback and can just publish into the void
    if(!cb)
    {
        if (!background_publish.publish(_writer_event_name, data, PRIVATE, priority))
        {
            rval = -EBUSY;
        }
        return rval;
    }

    auto timeout = millis() + timeout_ms;
    if (timeout < timeout_ms) { // unsigned overflow
        timeout = std::numeric_limits<system_tick_t>::max();
    }

    // Bind the data needed for deferred ack processing together with our publish callback. The original payload in data is copied
    // to a String and is moved around until it reaches the user callback.
    cloud_service_ack_context context {req_id, timeout, cb, data};
    auto publish_cb = make_shared_function([this, cloud_flags, context = std::move(context)]
        (particle::Error error, const char *event_name, const char *event_data) mutable -> void {
            std::lock_guard<RecursiveMutex> lg(mutex);

            if(error == Error::NONE) {
                if(cloud_flags & CloudServicePublishFlags::FULL_ACK) {
                    registerAckCallback(std::move(context));
                } else {
                    deferred_acks.push_back(std::move(make_shared_function([context = std::move(context)] () mutable -> int {
                        return context.callback(CloudServiceStatus::SUCCESS, std::move(context.data));
                    })));
                }
            } else if (error != Error::CANCELLED) {
                deferred_acks.push_back(std::move(make_shared_function([context = std::move(context)] () mutable -> int {
                    return context.callback(CloudServiceStatus::FAILURE, std::move(context.data));
                })));
            }
            // particle::Error::CANCELLED is used by BackgroundPublish::cleanup()/stop() to shut down the publisher; do not retry.
        }
    );

    if(!background_publish.publish(_writer_event_name, data,
                                   publish_flags | PRIVATE, priority, std::move(publish_cb)))
    {
        rval = -EBUSY;
    }

    if(!rval)
    {
        Log.info("cloud sent: %s", data);
    }

    return rval;
}

int CloudService::send(PublishFlags publish_flags, 
                    CloudServicePublishFlags cloud_flags, 
                    cloud_service_ack_callback cb,
                    unsigned int timeout_ms, 
                    std::size_t priority)
{
    int rval = 0;
    // NOTE: if this JSON object close code changes then estimatedEndCommandSize() must be updated.
    // The general pattern is:
    //       ,\"req_id\":0000000000}
    uint32_t req_id = (cb && (cloud_flags & CloudServicePublishFlags::FULL_ACK)) ? get_next_req_id() : 0;

    if(req_id)
    {
        writer().name(CLOUD_KEY_REQ_ID).value((unsigned int) req_id);
    }
    writer().endObject();

    // output json overflowed the buffer
    // dataSize does not include the null terminator
    if(writer().dataSize() >= writer().bufferSize())
    {
        unlock();
        return -ENOSPC;
    }

    // ensure null termination of the output json
    writer().buffer()[writer().dataSize()] = '\0';

    rval = send(writer().buffer(), publish_flags, cloud_flags, cb, timeout_ms, _writer_event_name, req_id, priority);

    unlock();
    return rval;
}

int CloudService::sendAck(JSONValue &root, int status)
{
    int rval = beginResponse(CLOUD_CMD_ACK, root);
    if(!rval)
    {
        writer().name("status").value(status);
        rval = send();
    }

    return rval;
}

void print_tab(int count)
{
    for(int i=0; i < count; i++)
    {
        Log.printf("\t");
    }
}

// logs a parsed json object to output
void _log_json(JSONValue &root, int level)
{
    switch(root.type())
    {
        case JSON_TYPE_INVALID:
            break;
        case JSON_TYPE_NULL:
            Log.printf("null\n");
            break;
        case JSON_TYPE_BOOL:
            Log.printf("%s\n", (const char *) root.toString());
            break;
        case JSON_TYPE_NUMBER:
            Log.printf("%lf\n", root.toDouble());
            break;
        case JSON_TYPE_STRING:
            Log.printf("\"%s\"\n", (const char *) root.toString());
            break;
        case JSON_TYPE_ARRAY:
        {
            JSONArrayIterator it(root);
            Log.printf("array (length %d)\n", it.count());
            while(it.next())
            {
                JSONValue val = it.value();
                print_tab(level+1);
                _log_json(val, level + 1); // dirty dirty recursion!
            }
            break;
        }
        case JSON_TYPE_OBJECT:
        {
            JSONObjectIterator it(root);
            Log.printf("object (length %d)\n", it.count());
            while(it.next())
            {
                JSONValue val = it.value();
                print_tab(level+1);
                Log.printf("%s: ", (const char *) it.name());
                _log_json(val, level + 1); // dirty dirty recursion!
            }
            break;
        }
    }
}

// parse and log json object to output
void log_json(const char *json, size_t size)
{
    JSONValue root = JSONValue::parseCopy(json, size);

    _log_json(root, 0);
}

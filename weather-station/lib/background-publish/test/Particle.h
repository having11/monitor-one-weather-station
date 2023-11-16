#pragma once

#include <chrono>
#include <functional>
#include <mutex>
#include "concurrent_hal.h"

// List of all defined system errors
#define SYSTEM_ERROR_NONE                   (0)
#define SYSTEM_ERROR_UNKNOWN                (-100)
#define SYSTEM_ERROR_BUSY                   (-110)
#define SYSTEM_ERROR_NOT_SUPPORTED          (-120)
#define SYSTEM_ERROR_NOT_ALLOWED            (-130)
#define SYSTEM_ERROR_CANCELLED              (-140)
#define SYSTEM_ERROR_ABORTED                (-150)
#define SYSTEM_ERROR_TIMEOUT                (-160)
#define SYSTEM_ERROR_NOT_FOUND              (-170)
#define SYSTEM_ERROR_ALREADY_EXISTS         (-180)
#define SYSTEM_ERROR_TOO_LARGE              (-190)
#define SYSTEM_ERROR_NOT_ENOUGH_DATA        (-191)
#define SYSTEM_ERROR_LIMIT_EXCEEDED         (-200)
#define SYSTEM_ERROR_END_OF_STREAM          (-201)
#define SYSTEM_ERROR_INVALID_STATE          (-210)
#define SYSTEM_ERROR_IO                     (-220)
#define SYSTEM_ERROR_WOULD_BLOCK            (-221)
#define SYSTEM_ERROR_FILE                   (-225)
#define SYSTEM_ERROR_NETWORK                (-230)
#define SYSTEM_ERROR_PROTOCOL               (-240)
#define SYSTEM_ERROR_INTERNAL               (-250)
#define SYSTEM_ERROR_NO_MEMORY              (-260)
#define SYSTEM_ERROR_INVALID_ARGUMENT       (-270)
#define SYSTEM_ERROR_BAD_DATA               (-280)
#define SYSTEM_ERROR_OUT_OF_RANGE           (-290)
#define SYSTEM_ERROR_DEPRECATED             (-300)
#define SYSTEM_ERROR_COAP                   (-1000)
#define SYSTEM_ERROR_COAP_4XX               (-1100)
#define SYSTEM_ERROR_COAP_5XX               (-1132)
#define SYSTEM_ERROR_AT_NOT_OK              (-1200)
#define SYSTEM_ERROR_AT_RESPONSE_UNEXPECTED (-1210)

typedef uint32_t system_tick_t;
typedef uint16_t pin_t;

namespace particle {

template<bool S, size_t bits, typename T>
struct bits_fit_in_type {
    using type = typename std::conditional<S, typename std::make_signed<T>::type, typename std::make_unsigned<T>::type>::type;
    static const bool value = (bits <= std::numeric_limits<type>::digits);
};
} //particle

typedef void* os_mutex_recursive_t;
class RecursiveMutex
{
    os_mutex_recursive_t handle_;
public:
    /**
     * Creates a shared mutex.
     */
    RecursiveMutex(os_mutex_recursive_t handle) : handle_(handle) {}

    RecursiveMutex() : handle_(nullptr)
    {
    }

    ~RecursiveMutex() {
        dispose();
    }

    void dispose()
    {
    }

    void lock() { }
    bool trylock() { return true; }
    bool try_lock() { return true; }
    void unlock() {}

};

inline void delay(uint32_t ms) {}
inline void delayMicroseconds(uint32_t us) {}

class SystemClass {
public:
    SystemClass() : _tick(0) {}

    system_tick_t Uptime() const {
        return (system_tick_t)_tick;
    }

    unsigned uptime() const {
        return _tick / 1000;
    }

    uint64_t millis() const {
        return _tick;
    }

    void inc(int i = 1) {
        _tick += i;
    }

private:
    uint64_t _tick;
};

class Logger {
public:
    Logger() = default;
    Logger(const char *) {};
    void trace(const char* str, ...) {};
    void info(const char* str, ...) {};
    void error(const char* str, ...) {};
    void warn(const char* str, ...) {};
};

extern SystemClass System;
extern Logger Log;

inline system_tick_t millis(void) { return System.millis(); }

static inline bool HAL_IsISR() 
{
	return false;
}

static void vPortYield( void )
{}
#define portYIELD_FROM_ISR( x ) if( x ) vPortYield()

typedef short BaseType_t;

namespace particle {

template<typename TagT, typename ValueT>
class Flags;

// Class storing a typed flag value
template<typename TagT, typename ValueT = unsigned>
class Flag {
public:
    explicit Flag(ValueT val) {}

    explicit operator ValueT() const {}

    ValueT value() const {}

private:
    ValueT val_;
};

// Class storing or-combinations of typed flag values
template<typename TagT, typename ValueT = unsigned>
class Flags {
public:
    typedef TagT TagType;
    typedef ValueT ValueType;
    typedef Flag<TagT, ValueT> FlagType;

    Flags() {}
    Flags(Flag<TagT, ValueT> flag) {}

    explicit operator ValueT() const;
    explicit operator bool() const;

    ValueT value() const;

private:
    ValueT val_;

    explicit Flags(ValueT val);
};

class Error {
public:
    // Error type
    enum Type {
        NONE = 0,
        UNKNOWN,
        INVALID_STATE,
        INVALID_ARGUMENT,
        BUSY,
        LIMIT_EXCEEDED,
        CANCELLED,
    };

    Error(Type type = UNKNOWN);
    Error(Type type, const char* msg);
    explicit Error(const char* msg);
    Error(const Error& error);

    Type type() const;
    const char* message() const;

    bool operator==(const Error& error) const;
    bool operator!=(const Error& error) const;
    bool operator==(Type type) const;
    bool operator!=(Type type) const;

    explicit operator bool() const;

private:
    const char* msg_;
    Type type_;

    friend void swap(Error& error1, Error& error2);
};

inline Error::Error(Type type) :
        msg_(nullptr),
        type_(type) {
}

inline Error::Error(Type type, const char* msg) :
        msg_(msg),
        type_(type) {
}

inline Error::Error(const char* msg) :
        Error(UNKNOWN, msg) {
}

inline Error::Error(const Error& error) :
        Error(error.type_, error.msg_) {
}

inline Error::Type Error::type() const {
    return type_;
}

inline const char* Error::message() const {
    return msg_ ? msg_ : "";
}

inline bool Error::operator==(const Error& error) const {
    return (type_ == error.type_);
}

inline bool Error::operator!=(const Error& error) const {
    return !operator==(error);
}

inline bool Error::operator==(Type type) const {
    return (type_ == type);
}

inline bool Error::operator!=(Type type) const {
    return !operator==(type);
}

inline Error::operator bool() const {
    return type_ != NONE;
}

template<typename ContextT>
class Future {
public:
    Future() {}
    bool isSucceeded() const {
        return isSucceededReturn;
    }

    bool isDone() const {
        return isDoneReturn;
    }

    Error error() const {
        return err;
    }

    bool isDoneReturn;
    bool isSucceededReturn;
    Error err;
};

namespace protocol {
    const size_t MAX_EVENT_NAME_LENGTH = 64;
    const size_t MAX_EVENT_DATA_LENGTH = 1024;
}

} // namespace particle

struct PublishFlagType; // Tag type for Particle.publish() flags
typedef particle::Flags<PublishFlagType, uint8_t> PublishFlags;
typedef PublishFlags::FlagType PublishFlag;

class CloudClass {
public:

    CloudClass() {}
    inline particle::Future<bool> publish(const char *eventName, 
                                        const char *eventData, 
                                        PublishFlags flags1, 
                                        PublishFlags flags2 = PublishFlags()) {
        return state_output;
    }
    particle::Future<bool> state_output;
};
extern CloudClass Particle;

const uint32_t PUBLISH_EVENT_FLAG_PUBLIC = 0x0;
const uint32_t PUBLISH_EVENT_FLAG_PRIVATE = 0x1;
const uint32_t PUBLISH_EVENT_FLAG_NO_ACK = 0x2;
const uint32_t PUBLISH_EVENT_FLAG_WITH_ACK = 0x8;

const PublishFlag PUBLIC(PUBLISH_EVENT_FLAG_PUBLIC);
const PublishFlag PRIVATE(PUBLISH_EVENT_FLAG_PRIVATE);
const PublishFlag NO_ACK(PUBLISH_EVENT_FLAG_NO_ACK);
const PublishFlag WITH_ACK(PUBLISH_EVENT_FLAG_WITH_ACK);

#define pdFALSE			( ( BaseType_t ) 0 )
#define pdTRUE			( ( BaseType_t ) 1 )
#define pdPASS			( pdTRUE )
#define pdFAIL			( pdFALSE )

/* Default priority is the same as the application thread */
#define OS_THREAD_PRIORITY_DEFAULT       (2)
#define OS_THREAD_PRIORITY_CRITICAL      (9)
#define OS_THREAD_PRIORITY_NETWORK       (7)
#define OS_THREAD_PRIORITY_NETWORK_HIGH  (8)
#define OS_THREAD_STACK_SIZE_DEFAULT (3*1024)
#define OS_THREAD_STACK_SIZE_DEFAULT_HIGH (4*1024)
#define OS_THREAD_STACK_SIZE_DEFAULT_NETWORK (6*1024)

class Thread
{
public:
    Thread() = default;

    Thread(const char *name, 
            wiring_thread_fn_t function,
            os_thread_prio_t priority=OS_THREAD_PRIORITY_DEFAULT, 
            size_t stack_size=OS_THREAD_STACK_SIZE_DEFAULT) {
    }

    bool join() { return true; }
};


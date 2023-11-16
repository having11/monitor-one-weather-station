#ifndef __DEBOUNCESWITCHRK_H
#define __DEBOUNCESWITCHRK_H

// Switch Debouncing Library for Particle devices
// Repository: https://github.com/rickkas7/DebounceSwitchRK
// License: MIT

#include "Particle.h"

#include <vector>

class DebounceSwitch; // Forward declaration

/**
 * @brief These constants are the types of switch inputs that are supported
 */
enum class DebounceSwitchStyle {
    /**
     * @brief Momentary switch to GND with an external pull-up
     */
    PRESS_LOW,

    /**
     * @brief Momentary switch to 3V3 with an external pull-down
     */
    PRESS_HIGH,

    /**
     * @brief Momentary switch to GND, along with using the MCU internal pull-up resistor
     * 
     * This is the recommended mode for most circuits that don't already
     * have an external pull resistor.
     */
    PRESS_LOW_PULLUP,

    /**
     * @brief Momentary switch to 3V3, along with using the MCU internal pull-up resistor
     */
    PRESS_HIGH_PULLDOWN,

    /**
     * @brief Toggle (on/off) switch that drives the input to GND or 3V3
     */
    TOGGLE,

    /**
     * @brief Toggle switch that connects the input to 3V3 in one position
     * and disconnected in the other
     */
    TOGGLE_PULLDOWN,

    /**
     * @brief Toggle switch that connects the input to GND in one position
     * and disconnected in the other.
     * 
     * This is the recommended mode for toggle switches because TOGGLE
     * mode may leave the input in an unconnected/indeterminate state 
     * when between on and off positions. This option makes sure there
     * is always pull on the input.
     */
    TOGGLE_PULLUP,
};

/**
 * @brief These constants indicate the state of button debouncing. 
 * 
 * You can use getPressState() to find the state in your callback. Polling
 * it is not recommended because you will likely miss states.
 */
enum class DebouncePressState {
    /**
     * @brief Button is not pressed. You won't receive this on your callback.
     */
    NOT_PRESSED,

    /**
     * @brief Button was just pressed. This starts every press sequence.
     */
    PRESS_START,

    /**
     * @brief Button has been held down longer than the long button press time but less 
     * than the very long button press time.
     * 
     * The is an intermediate state that will be followed by LONG or VERY_LONG. But at 
     * this point it is known that it will not be SHORT.
     */
    PROGRESS,

    /**
     * @brief Short button press, less than longPressMs (3000) milliseconds
     * 
     * The normal sequence will be: PRESS_START, SHORT, RELEASED. Then, later,
     * TAP with a count of how many short taps there were.
     * 
     * If you are not handing multi-tap, use SHORT. You'll get the notification immediately
     * after release, instead of having to wait the inter-tap delay (500 milliseconds).
     * 
     * Otherwise, wait for TAP will be sent and you can use getTapCount() to find out how 
     * many taps there were.
     */
    SHORT,

    /**
     * @brief Long button press, greater than longPressMs (3000) milliseconds but less than veryLongPress (10000).
     * 
     * The normal sequence will be: PRESS_START, PROGRESS, LONG, RELEASED.
     * 
     * LONG is normally generated when the button is released, because that's when we know that
     * the press isn't VERY_LONG.
     * 
     * If you don't want to use long or very long button presses, call withNoLongPress(). If
     * very long presses are disabled, the LONG is generated while the button is still down
     * after the longPressMs (3 seconds) occurs.
     */
    LONG,

    /**
     * @brief Very long button press, greater than veryLongPress (10000).
     * 
     * The normal sequence will be: PRESS_START, PROGRESS, VERY_LONG, RELEASED.
     * 
     * If you don't want to use very long button presses, call withNoVeryLongPress().
     * 
     * Note that VERY_LONG will be generated while the button is still down.
     */
    VERY_LONG,

    /**
     * @brief Used internally, you won't get this on the callback
     */
    WAIT_RELEASE,

    /**
     * @brief Button has been released. This generally follows SHORT, LONG, or VERY_LONG.
     */
    RELEASED,

    /**
     * @brief There was a single or multi-tap.
     * 
     * After a button release, if there's another short tap started within interTapMs (500 
     * milliseconds), then a multi-tap sequence is started. Use getTapCount() to determine
     * how many there were. 1 is a single-tap, 2 is a double-tap, etc.
     */
    TAP,

    /**
     * @brief Used internally, you won't get this on your callback.
     */
    TOGGLE_START,

    /**
     * @brief The state of the toggle switch is LOW
     * 
     * You will get one of TOGGLE_LOW or TOGGLE_HIGH at initialization, then on
     * each state change.
     */
    TOGGLE_LOW,

    /**
     * @brief The state of the toggle switch is HIGH
     * 
     * You will get one of TOGGLE_LOW or TOGGLE_HIGH at initialization, then on
     * each state change.
     */
    TOGGLE_HIGH
};

/**
 * @brief Container for timing-related settings for switches. 
 * 
 * These are set per-switch, but you can create one of these objects and set all
 * of the settings for multiple switches at once if you have a number of identical
 * switches, for example all momentary switches. If you have a mix of momentary
 * and toggle switches that require different debounce timing, you can set the
 * settings independently.
 */
class DebounceConfiguration {
public:
    /**
     * @brief Set the debounce press and release time in milliseconds (default: 20)
     * 
     * @param ms Value to change to in milliseconds. Reasonable values are 20 to 100.
     * 
     * There are also methods to set the press and release time individually.
     */
    DebounceConfiguration &withDebounceMs(unsigned long ms) { debouncePressMs = debounceReleaseMs = ms; return *this; };

    /**
     * @brief Set the debounce press time in milliseconds (default: 20)
     * 
     * @param ms Value to change to in milliseconds. Reasonable values are 20 to 100.
     * 
     * If the switch is particularly bouncy you can make this larger, but increasing it
     * also adds to the latency for detecting button presses, so 20 is a good balance.
     * 
     * For toggle switches, set debouncePressMs and debounceReleaseMs to be the same value, which
     * could be a bit larger, possibly even 100 ms for both.
     */
    DebounceConfiguration &withDebouncePressMs(unsigned long ms) { debouncePressMs = ms; return *this; };

    /**
     * @brief Gets the debounce time in milliseconds (default: 20)
     */
    unsigned long getDebouncePressMs() const { return debouncePressMs; };

    /**
     * @brief Set the debounce release time in milliseconds (default: 20)
     * 
     * @param ms Value to change to in milliseconds. Reasonable values are 20 to 100.
     * 
     * The minimum is around 10 ms. It should not be larger than 100 milliseconds as
     * it affects the latency of detecting button presses. Momentary switches usually
     * are bouncier on release (because of the spring), so setting it a little higher
     * may help if you are still seeing bounces.
     * 
     * For toggle switches, set debouncePressMs and debounceReleaseMs to be the same value, which
     * could be a bit larger, possibly even 100 ms for both.
     */
    DebounceConfiguration &withDebounceReleaseMs(unsigned long ms) { debouncePressMs = ms; return *this; };

    /**
     * @brief Gets the debounce time in milliseconds (default: 20)
     */
    unsigned long getDebounceReleaseMs() const { return debouncePressMs; };

    /**
     * @brief Set the inter-tap time in milliseconds (default: 500)
     * 
     * @param ms Value to change to in milliseconds. Reasonable values are 250 to 2000.
     * 
     * When detecting multiple taps, there needs to be a timeout
     * from the last release before we know whether it was a single, double, or triple
     * tap. After the inter-tap timeout occurs, we definitively know that the last tap
     * has been made and any new tap will start over again at 1.
     * 
     * Making this longer makes it easier to double-tap, but it also delays the amount
     * of time until a TAP is generated.
     * 
     * If you are not using double or triple tap, you can ignore this setting and only
     * respond to the SHORT state instead of TAP. SHORT is generated after each
     * release without consulting the inter-tap timeout.
     */
    DebounceConfiguration &withInterTapMs(unsigned long ms) { interTapMs = ms; return *this; };

    /**
     * @brief Gets the inter-tap time in milliseconds (default: 500)
     */
    unsigned long getInterTapMs() const { return interTapMs; };

    /**
     * @brief Set the long press duration in milliseconds (default: 3000, 3 seconds)
     * 
     * @param ms Value to change to in milliseconds. Reasonable values are 2000 to 6000 or 0 to disable.
     * 
     * If the button is held down longer than long press, but shorter than very long
     * press, then a LONG button state is generated. It also means a SHORT will not
     * be generated.
     */
    DebounceConfiguration &withLongPressMs(unsigned long ms) { longPressMs = ms; return *this; };

    /**
     * @brief Disables support for long and very long press. Only short press is returned.
     */ 
    DebounceConfiguration &withNoLongPress() { longPressMs = 0; return *this; };

    /**
     * @brief Get the long press duration in milliseconds (default: 3000, 3 seconds)
     */
    unsigned long getLongPressMs() const { return longPressMs; };

    /**
     * @brief Set the very long press duration in milliseconds (default: 10000, 10 seconds)
     * 
     * @param ms Value to change to in milliseconds. Reasonable values are 2000 to 15000. 
     * Also 0 is valid as very long press disabled; see withNoVeryLongPress() or 0 to disable.
     */
    DebounceConfiguration &withVeryLongPressMs(unsigned long ms) { veryLongPressMs = ms; return *this; };

    /**
     * @brief Disables support for very long press
     * 
     * By disabling VERY_LONG the states: PRESS_START, LONG, and RELEASED will be sent to the
     * callback. PROGRESS and VERY_LONG will never be sent. This can simplify your code if
     * you only need two press states (SHORT and LONG).
     */
    DebounceConfiguration &withNoVeryLongPress() { veryLongPressMs = 0; return *this; };

    /**
     * @brief Gets the very long press duration in milliseconds (default: 10000, 10 seconds)
     */
    unsigned long getVeryLongPressMs() const { return veryLongPressMs; };

    /**
     * @brief Copy settings from another DebounceConfiguration
     * 
     * @param src The settings to copy from
     */
    DebounceConfiguration &operator=(const DebounceConfiguration &src);

protected:
    /**
     * @brief Debounce period for press in milliseconds (default: 20)
     * 
     * The minimum is around 10 ms. It should not be larger than 100 milliseconds as
     * it affects the latency of detecting button presses. 
     * 
     * For toggle switches, set debouncePressMs and debounceReleaseMs to be the same value, which
     * could be a bit larger, possibly even 100 ms for both.
     */
    unsigned long debouncePressMs = 20;

    /**
     * @brief Debounce period for release in milliseconds (default: 20)
     * 
     * The minimum is around 10 ms. It should not be larger than 100 milliseconds as
     * it affects the latency of detecting button presses. Momentary switches usually
     * are bouncier on release (because of the spring), so setting it a little higher
     * may help if you are still seeing bounces.
     * 
     * For toggle switches, set debouncePressMs and debounceReleaseMs to be the same value, which
     * could be a bit larger, possibly even 100 ms for both.
     */
    unsigned long debounceReleaseMs = 20;

    /**
     * @brief How long to wait for double-tap, triple-tap, etc. in milliseconds (default: 500)
     * 
     * Making this short reduces the latency until the TAP event is generated. However, it
     * makes it harder to multi-tap as you have less time between release and the next press.
     * Making it too long causes the TAP event to be very delayed, and also causes extraneous
     * multi-taps. 500 (1/2 second) feels about right to me. 
     */
    unsigned long interTapMs = 500;

    /**
     * @brief How long to wait for a long press in milliseconds (default: 3000, or 3 seconds)
     * 
     * Set to 0 to disable longPress and veryLongPress. 
     */
    unsigned long longPressMs = 3000;

    /**
     * @brief How long to wait for a very long press in milliseconds (default: 10000, or 10 seconds)
     * 
     * Set to 0 to disable veryLongPress. 
     */
    unsigned long veryLongPressMs = 10000;
};

/**
 * @brief Configuration and state for a single switch
 * 
 * The DebounceSwitch class has one global singleton instance, but there's an instance of 
 * for each switch that stores information about that one switch. This allows efficient
 * handling of multiple buttons while keeping their debouncing state separate.
 * 
 * You do not instantate these directly, the methods in DebounceSwitch such as 
 * addSwitch(), addVirtualSwitch(), and addNotifySwitch() will instantiate it for you.
 */
class DebounceSwitchState : public DebounceConfiguration {

public:    
    /**
     * @brief In notify mode, addNotifySwitch() used, tells the library when the signal changes state
     * 
     * @param signal true = HIGH and false = LOW. Whether this is pressed or not depends on the 
     * DebounceSwitchStyle for this input.
     */
    void notify(bool signal);

    /**
     * @brief Returns true if the switch is currently pressed. 
     * 
     * This works like getDebouncedLastSignal() except it inverts the debouncedLastState if
     * the button is a press = LOW type so pressed is always true.
     * 
     * This is the debounced signal and is fast as it just returns a variable. It can be called from
     * an ISR. 
     */
    bool isPressed() const;

    /**
     * @brief Gets the debounced version of lastSignal, essentially the current debounced state of this pin.
     * 
     * See also isPressed()
     */
    bool getDebouncedLastSignal() const { return debouncedLastSignal; };

    /**
     * @brief Returns the current press state
     * 
     * You should avoid polling the press state because you can easily miss transitions. However,
     * you will likely need to call this method from your callback so you can find out what
     * happened.
     */
    DebouncePressState getPressState() const { return pressState; };

    /**
     * @brief Returns a readable name for the current pressed state
     * 
     * Returned value is a short English string that matches the constant name.
     */
    const char *getPressStateName() const { return getPressStateName(pressState); };


    /**
     * @brief When a TAP state is sent to the callback, this determines how many.
     * 
     * @return 1 for single tap, 2 for double tap, 3 for triple tap, ...
     * 
     * You can count as many taps are you want, but it's a little unwieldy for more than 3.
     */
    int getTapCount() const { return sequenceCount; };

    /**
     * @brief Get the pin this object is configured for
     * 
     * In addition to real pins (D2, D3, A4, ...) it can also be a constant: 
     * 
     * - DebounceSwitch::VIRTUAL_PIN The pin is not a directly connected GPIO, a polling function is used
     * - DebounceSwitch::NOTIFY_PIN The pin is not polled; a function is called when the state changes
     */
    pin_t getPin() const { return pin; };

    /**
     * @brief Sets the configuration for this switch
     * 
     * @param config a A DebounceConfiguration object with the settings you want to use
     * 
     * This method is handy if you want to share the same configuration across several switches
     * instead of calling methods like withDebouncePressMs() individually for each switch.
     * The settings are copied from config.
     */
    DebounceSwitchState &withConfig(const DebounceConfiguration &config) {
        *(DebounceConfiguration *)this = config;
        return *this;
    }

    /**
     * @brief Converts a signal value (false = LOW, true = HIGH) to a DebouncePressState
     * 
     * @param signal the value to convert
     * 
     * @return Either TOGGLE_LOW (false or LOW), TOGGLE_HIGH (true or HIGH)
     */
    static DebouncePressState signalToPressState(bool signal);

    /**
     * @brief Gets a readable name for a pressState value
     * 
     * @param pressState the value to convert
     * 
     * @return A constant c-string literal that corresponds to the C++ enum name, for
     * example "PRESS_START".
     */
    static const char *getPressStateName(DebouncePressState pressState);

protected:
    /**
     * @brief Constructor (used internally)
     * 
     * @param pin The pin to add a switch to (D2, D3, ...) or a special constant: DebounceSwitch::VIRTUAL_PIN
     * or DebounceSwitch::NOTIFY_PIN.
     * 
     * @param style The style of button (push button or toggle), with or without pull-up or pull-down.
     * 
     * @param config The configuration to use
     * 
     * @param callback The function to call 
     * 
     * @param context Optional pointer to pass to the callback.
     * 
     * @param pollCallback The callback to poll for the value. Not used for notify callbacks. This
     * is called every checkMs (5 milliseconds).
     * 
     * @param pollContext Optional pointer to pass to the pollCallback.
     * 
     * Do not instantate this directly, the methods in DebounceSwitch such as 
     * addSwitch(), addVirtualSwitch(), and addNotifySwitch() will instantiate it for you.
     */
    DebounceSwitchState(pin_t pin, DebounceSwitchStyle style, DebounceConfiguration *config, std::function<void(DebounceSwitchState *switchState, void *context)> callback, void *context, std::function<bool(DebounceSwitchState *switchState, void *pollContext)> pollCallback, void *pollContext);

    /**
     * @brief Destructor - used internally
     * 
     * This object is only deleted by the DebounceSwitch class.
     */
    virtual ~DebounceSwitchState();

    /**
     * @brief This class is not copyable
     */
    DebounceSwitchState(const DebounceSwitchState&) = delete;

    /**
     * @brief This class is not copyable
     */
    DebounceSwitchState& operator=(const DebounceSwitchState&) = delete;

    /**
     * @brief Used internally to poll the value of this pin
     * 
     * @return The value of the pin (false = LOW, true = HIGH)
     */
    bool poll();

    /**
     * @brief Run the state machine for this pin
     */
    void run();

    /**
     * @brief Used internally to set pressState and optionally call the callback
     * 
     * @param pressState The new state to set
     * 
     * @param callCallback True to call the callback
     * 
     * Normally you pass true for callCallback but for hidden, internal states,
     * you pass false so the state will change but the callback is not called.
     */
    void setPressState(DebouncePressState pressState, bool callCallback);

    /**
     * @brief Handle debouncing this pin
     * 
     * This is called every checkMs milliseconds (default: 5) to handle debouncing.
     * Input is the lastSignal (which is updated either by polling or notification)
     * and the output is debouncedLastSignal which is the debounced version.
     */
    void checkDebounce();

    /**
     * @brief pin The pin being monitored (D2, D3, ...) or a special constant
     * 
     * - DebounceSwitch::VIRTUAL_PIN
     * = DebounceSwitch::NOTIFY_PIN.
     */
    pin_t pin;

    /**
     * @brief The style of switch (button or toggle) allow with whether it's active high or low and any pull
     */
    DebounceSwitchStyle style;

    /**
     * @brief Function to call when the state of the button changes
     */
    std::function<void(DebounceSwitchState *switchState, void *context)> callback;

    /**
     * @brief Optional data to pass to callback. May be NULL. Not interpreted by this class.
     */
    void *context;

    /**
     * @brief Function to call to determine the state of the switch using polling
     */
    std::function<bool(DebounceSwitchState *switchState, void *context)> pollCallback;

    /**
     * @brief Optional data to pass to pollCallback. May be NULL. Not interpreted by this class.
     */
    void *pollContext;

    /**
     * @brief The current state of the button state machine
     * 
     * Use setSwitchState() to change the state.
     */
    DebouncePressState pressState = DebouncePressState::NOT_PRESSED;

    /**
     * @brief Last state of the switch signal
     * 
     * This is updated by the poll() method when using polling periodically (default: 5 milliseconds based on checkMs).
     * 
     * If using notify, calling notify updates this variable.
     */
    bool lastSignal = false;

    /**
     * @brief millis() value when the button was pressed
     * 
     * Even though millis() rolls over to 0 every 49 days, this variable works correctly on rollover.
     */
    unsigned long pressMs = 0;

    /**
     * @brief millis() value when the button was released
     * 
     * Even though millis() rolls over to 0 every 49 days, this variable works correctly on rollover.
     */
    unsigned long releaseMs = 0;

    /**
     * @brief Number of taps for multi-tap
     */
    int sequenceCount = 0;

    /**
     * @brief The last millis() lastSignal and debouncedLastSignal were the same. Used in the debounce algorithm.
     * 
     * Even though millis() rolls over to 0 every 49 days, this variable works correctly on rollover.
     */
    unsigned long debounceLastSameMs = 0;

    /**
     * @brief Debounced version of lastSignal
     */
    bool debouncedLastSignal = false;

    friend class DebounceSwitch;
};

/**
 * @brief Singleton class for all debounced switches on a device
 * 
 * Use DebounceSwitch::getInstance() to get the object instance pointer.
 * 
 * Call DebounceSwitch::getInstance()->setup() during global setup() to initialize
 * the library. This is required!
 * 
 * Call DebounceSwitch::getInstance()->addSwitch() to add switches to debounce.
 * You should add switches during setup().
 * 
 * It uses threads so you do not need to call anything from loop().
 */
class DebounceSwitch : public DebounceConfiguration {
public:
    /**
     * @brief This class is a singleton - use getInstance() to get the pointer to the object
     * 
     * You never construct one of these object directly (using a global, stack, or new).
     * You also cannot destruct it once created.
     */
    static DebounceSwitch *getInstance();

    /**
     * @brief You must call DebounceSwitch::getInstance()->setup() from the global setup!
     * 
     * This initializes the library. You can call it more than once safely, this is handy
     * if you want to initialize it from a class instance setup.
     */
    void setup();   

    /**
     * @brief Adds a new switch to debounce. Normally done during setup.
     * 
     * @param pin The pin to add a switch to (D2, D3, ...) or a special constant: DebounceSwitch::VIRTUAL_PIN
     * or DebounceSwitch::NOTIFY_PIN.
     * 
     * @param style The type of switch, PRESS for momentary switches or TOGGLE for toggle switches, along 
     * with whether they're connected to 3V3 or GND, and whether MCU pull-up or down should be used.
     * 
     * @param callback The function to call when a switch event occurs.
     * 
     * @param context Optional data passed to the callback if desired. Pass 0 if not using.
     * 
     * @param pollCallback The function to call to poll the GPIO. Not needed for standard GPIO that can be
     * read using pinReadFast. Optional for NOTIFY_PIN callbacks. Required for VIRTUAL_PIN (not a standard
     * GPIO and using polling, not notify).
     * 
     * @param pollContext Optional data to pass to pollCallback. Pass 0 if not using.
     * 
     * Note that there are three overloads for the addSwitch method. It's also possible to use it
     * with class member functions instead of the functions and context here. Also, since the
     * functions are defined using a `std::function` you can also pass a C++11 lambda function
     * to this method. This is shown in the 03-notify example.
     * 
     * The prototype for callback is:
     * 
     * ```
     * void callback(DebounceSwitchState *switchState, void *context)
     * ```
     * 
     * The prototype for pollCallback is:
     * 
     * ```
     * bool pollCallback(DebounceSwitchState *switchState, void *context)
     * ```
     * 
     */
    DebounceSwitchState *addSwitch(pin_t pin, DebounceSwitchStyle style, 
        std::function<void(DebounceSwitchState *switchState, void *context)> callback, void *context = 0, 
        std::function<bool(DebounceSwitchState *switchState, void *pollContext)> pollCallback = 0, void *pollContext = 0);

    /**
     * @brief Adds a new switch to debounce with callback in a class member function
     * 
     * @param pin The pin to add a switch to (D2, D3, ...) or a special constant: DebounceSwitch::VIRTUAL_PIN
     * or DebounceSwitch::NOTIFY_PIN.
     * 
     * @param style The type of switch, PRESS for momentary switches or TOGGLE for toggle switches, along 
     * with whether they're connected to 3V3 or GND, and whether MCU pull-up or down should be used.
     * 
     * @param callback The class member to call when a switch event occurs.
     * 
     * @param instance The "this" pointer to use for the instance to call the class member function with.
     * 
     * You typically call it like this:
     * 
     * ```
     * DebounceSwitch::getInstance()->addSwitch(pin, DebounceSwitchStyle::PRESS_LOW_PULLUP, 
     *   &MyButtonClass::callback, this);
     * ```
     * 
     * Note the very specific syntax for callback: `&MyButtonClass::callback`. The callback member function
     * has this prototype:
     * 
     * ```
     * void YourClass::callback(DebounceSwitchState *switchState);
     * ```
     * 
     * Note that when using a class member function, context is not used; if you want to
     * pass data to the callback, use a class member variable instead of context.
     */
    template <typename T>
    DebounceSwitchState *addSwitch(pin_t pin, DebounceSwitchStyle style, 
        void (T::*callback)(DebounceSwitchState *), T *instance) {
        return addSwitch(pin, style, std::bind(callback, instance, std::placeholders::_1), 0, 0, 0);
    };

    /**
     * @brief Adds a new switch to debounce with callback in a class member function
     * 
     * @param pin The pin to add a switch to (D2, D3, ...) or a special constant: DebounceSwitch::VIRTUAL_PIN
     * or DebounceSwitch::NOTIFY_PIN.
     * 
     * @param style The type of switch, PRESS for momentary switches or TOGGLE for toggle switches, along 
     * with whether they're connected to 3V3 or GND, and whether MCU pull-up or down should be used.
     * 
     * @param callback The class member to call when a switch event occurs.
     * 
     * @param instance The "this" pointer to use for the instance to call the class member function with.
     * 
     * @param pollingCallback The class member to call to poll for the current value.
     * 
     * @param pollingInstance The "this" pointer to use for the instance to call the class member function with.
     * Note that instance and pollingInstance do not need to be for the same class.
     * 
     * You typically call it like this:
     * 
     * ```
     * DebounceSwitch::getInstance()->addSwitch(pin, DebounceSwitchStyle::PRESS_LOW_PULLUP,
     *  &MyButtonClass::callback, this, &MyButtonClass::pollingCallback, this);
     * ```
     * 
     * Note the very specific syntax for callback: `&MyButtonClass::callback`. The callback member function
     * has this prototype:
     * 
     * ```
     * void YourClass::callback(DebounceSwitchState *switchState);
     * ```
     * 
     * And, similarly for pollingCallback, which must return a bool:
     * 
     * ```
     * bool YourClass::pollingCallback(DebounceSwitchState *switchState);
     * ```
     * 
     * Note that when using a class member function, context is not used; if you want to
     * pass data to the callback, use a class member variable instead of context. Also, while both callbacks
     * are in the same class in example 09-class-member-polling, they don't have to be.
     */
    template <typename T1, typename T2>
    DebounceSwitchState *addSwitch(pin_t pin, DebounceSwitchStyle style, 
        void (T1::*callback)(DebounceSwitchState *), T1 *instance,
        bool (T2::*pollingCallback)(DebounceSwitchState *), T2 *pollingInstance) {
        return addSwitch(pin, style, std::bind(callback, instance, std::placeholders::_1), 0, 
            std::bind(pollingCallback, pollingInstance, std::placeholders::_1), 0);
    };


    
    /**
     * @brief Adjust how often to poll the switches in milliseconds (default: 5)
     * 
     * @param ms Value to change to in milliseconds
     * 
     * You probably should not change this, because making it smaller
     * doesn't really improve performance, and making it longer can cause presses
     * to be missed. It cannot be larger than debounceMs.
     */
    DebounceSwitch &withCheckMs(unsigned long ms) { checkMs = ms; return *this; };

    /**
     * @brief Get
     *  how often to poll the switches in milliseconds (default: 5)
     */
    unsigned long getCheckMs() const { return checkMs; };

    /**
     * @brief Set the stack size for the worker thread (default: 1024 bytes)
     * 
     * You might want to make this bigger if you get stack overflow in your callback,
     * or you may want to reduce the amount of code you execute in your callback.
     * 
     * You must call this before the first call to the setup() method! Changing it
     * later will have no effect.
     */
    DebounceSwitch &withStackSize(size_t _stackSize) { stackSize = _stackSize; return *this; };

    /**
     * @brief Constant to pass to addSwitch() if you are using something other than built-in GPIO
     * 
     * For example, if you are using an MCP23008 I2C GPIO expander and the switch is connected to
     * that and you are polling for changes. 
     * 
     * See also NOTIFY_PIN.
     */
    static const pin_t VIRTUAL_PIN = 8192;

    /**
     * @brief Constant to pass to addSwitch() if you are using something other than built-in GPIO
     * 
     * For example, if you are using an MCP23008 I2C GPIO expander and the switch is connected to
     * that and you are using an interrupt line from the MCP23008 to the MCU to notify of GPIO
     * state changes on the expander so you don't need to poll over I2C constantly.
     * 
     * See also VIRTUAL_PIN.
     */
    static const pin_t NOTIFY_PIN = 8193;

protected:
    /**
     * @brief Private constructor - use getInstance() instead to get the singleton
     */
    DebounceSwitch();

    /**
     * @brief The singleton object is never destructed one constructed.
     */
    virtual ~DebounceSwitch();

    /**
     * @brief This class is not copyable
     */
    DebounceSwitch(const DebounceSwitch&) = delete;

    /**
     * @brief This class is not copyable
     */
    DebounceSwitch& operator=(const DebounceSwitch&) = delete;

    /**
     * @brief Internal thread function. Never returns.
     */
    void threadFunction();

    /**
     * @brief Internal thread function. Never returns.
     */
    static os_thread_return_t threadFunctionStatic(void* param);

    /**
     * @brief Function used to poll a hardware GPIO using pinReadFast
     * 
     * @param switchState The pin to poll
     * 
     * @param context The option context that was passed into addSwitch. This
     * may be NULL. 
     * 
     * If you addSwitch with a physical GPIO pin (D2, A3, etc.) and do not 
     * set a pollingCallback then this function is used to read the GPIO.
     */
    static bool gpioPoll(DebounceSwitchState *switchState, void *context);

    /**
     * @brief How often to check switch state (default: 5)
     * 
     * Switches are checked on a constant cadence as part of the debouncing process.
     * There's a good article about pitfalls of some commonly used debouncing algoriths
     * (here)[https://www.embedded.com/my-favorite-software-debouncers/]. 
     * 
     * In any case, every checkMs milliseconds the debouncing algorithm runs. This
     * is done even for notify pins, which are still checked on the same cadence.
     * 
     * The default value is 5 milliseconds and this should be appropriate in most cases.
     * It can't be larger than 20 milliseconds, and shouldn't be less than 1, and 5 
     * is about right.
     * 
     * This must be the same for all switches, but the other parameters like the length
     * of debounce are configurable on a per-switch basis.
     */
    unsigned long checkMs = 5;

    /**
     * @brief Thread object for the worker thread
     * 
     * This is also used to determine if the setup method has already been called
     */
    Thread *thread = 0;

    /**
     * @brief All of the DebounceSwitchState classes, one for each switch
     * 
     * These are instantiated by addSwitch() and added to this vector.
     */
    std::vector<DebounceSwitchState *> switchStates;

    /**
     * @brief Stack size. Must be set before calling the setup method
     */
    size_t stackSize = 1024; // OS_THREAD_STACK_SIZE_DEFAULT == 3K

    /**
     * @brief millis() value at last check of buttons. Compared with checkMs.
     * 
     * This works properly across millis() rollover at 49 days.
     */
    unsigned long lastCheck = 0;

    /**
     * @brief Singleton object instance. getInstance() allocates it if it has not been allocated, otherwise returns instance.
     */
    static DebounceSwitch *instance;
};

#endif /* __DEBOUNCESWITCHRK_H */

# DebounceSwitchRK

Library for switch debouncing for Particle devices


## Library features

- Debounces both momentary (push button) and toggle switches.
- For push buttons, handles long press, very long press, and multi-tap (double-tap, triple-tap, etc.).
- Thread-based, so button changes are not lost even if loop() is blocked or delayed.
- Handles buttons connected to 3V3 or GND (active high or active low) with internal or external pull.
- Handles buttons connected to a GPIO pin.
- Also handles buttons connected to expanders, such as MCP23008 I2C GPIO expanders.
- Works with both polling (default) and notify-on-change (handy for I2C GPIO with interrupts).
- Many configurable parameters.

The full browsable API documentation is available [here](https://rickkas7.github.io/DebounceSwitchRK/index.html) as well as in the docs/html directory in the [Github Repository](https://github.com/rickkas7/DebounceSwitchRK).

## General Design


### Button Styles

Different types of buttons (momentary push button or toggle), active high or low, with or without pull resistors, are specified using the `DebounceSwitchStyle` parameter to addSwitch().

| Constant | Purpose |
| :--- | :--- |
| PRESS_LOW | Momentary switch to GND with an external pull-up |
| PRESS_HIGH | Momentary switch to 3V3 with an external pull-down |
| PRESS_LOW_PULLUP | Momentary switch to GND, along with using the MCU internal pull-up resistor |
| PRESS_HIGH_PULLDOWN | Momentary switch to 3V3, along with using the MCU internal pull-up resistor | 
| TOGGLE | Toggle (on/off) switch that drives the input to GND or 3V3 |
| TOGGLE_PULLDOWN | Toggle switch that connects the input to 3V3 in one position and disconnected in the other |
| TOGGLE_PULLUP | Toggle switch that connects the input to GND in one position and disconnected in the other |

If you are designing a new circuit, the best option is to have your switch connect the GPIO to GND (not 3V3) and use on the of the MCU pull-up modes. So `PRESS_LOW_PULLUP` for a momentary push-button switch or `TOGGLE_PULLUP` for a toggle switch.

### Callbacks

This library uses a thread and a callback approach. Even if the GPIO are polled (as is typically
the case for hardware GPIO connected to the MCU), you will be notified of debounced press and tap
events using a callback.

The callback has this prototype:

```
void callback(DebounceSwitchState *switchState, void *context);
```

- switchState is an object pointer for the switch that caused the event, and also what happened.
- context is an optional pointer you passed into addSwitch() you can use to communicate extra
data to the callback.

It's also possible to make the callback a class member function of a class you've created.

```
void YourClass::callback(DebounceSwitchState *switchState);
```

The callback is called from a worker thread with a small (1024 byte) stack, so you should avoid
doing operations which block or use a lot of stack space.

Returning as quickly as possible without blocking is the best. If you must block, make sure you don't block more than 5 milliseconds total or the debouncing of the buttons and performance will be affected.

### Polling Callback

If you are using GPIO connected to another chip, you can provide a pollCallback to poll the GPIO
values. For example, if you have an MCP23008 I2C GPIO expander, you can connect buttons to that.

The poolCallback has this prototype:

```
bool callback(DebounceSwitchState *switchState, void *context);
```

- switchState is an object pointer for the switch that caused the event, and also what happened.
- context is an optional pointer you passed into addSwitch() you can use to communicate extra
data to the callback.

Note that the pollCallback returns a bool for the state of the GPIO (false = LOW, true = HIGH).

It's also possible to make the pollCallback a class member function of a class you've created.

```
bool YourClass::callback(DebounceSwitchState *switchState);
```

If you are using an external GPIO you need to manage the pull resistors in your own code, so you'd use one of switch types `PRESS_LOW`, `PRESS_HIGH`, or `TOGGLE`.

### Notify Mode

In some cases, it's inefficient to poll the switch. Polling happens every 5 milliseconds by
default, which is fine for GPIO connected directly to the MCU. However, if the GPIO is on an I2C
expander, it's still well within the capabilities of I2C, but it can be made more efficient.
 
The MCP23008 supports interrupt mode. When the external GPIO changes state, it sets an output
that is read by a hardware GPIO on the MCU. This allows the MCU to determine if a change has
occurred on the expander without having to do an I2C transaction; instead if can just read a 
hardware GPIO. 

The MCP23008-RK library supports this mode, and allows for a callback to be called when the
external GPIO state changes. This callback can then use the notify mode of the 
DebounceSwitchRK library to tell the debouncer what the current state of the switch signal 
is.

See the example in more-tests/02-mcp23008-interrupts:

```
DebounceSwitchState *sw = DebounceSwitch::getInstance()->addNotifySwitch(DebounceSwitchStyle::PRESS_LOW, 
    [](DebounceSwitchState *switchState, void *) {
        // Called to notify of switch operations
        Log.info("state=%s", switchState->getPressStateName());
        if (switchState->getPressState() == DebouncePressState::TAP) {
            Log.info("%d taps", switchState->getTapCount());
        }
    }, NULL);

gpio.attachInterrupt(SWITCH_PIN, CHANGE, [sw](bool bValue) {
    // Log.info("bValue=%d", bValue);
    sw->notify(bValue);
});
```

## DebounceConfiguration

There are a number of configurable parameters. They can be set globally, for future calls to addSwitch(), per-switch, if desired.

For example:

```
void setup() {
    DebounceSwitch::getInstance()->setup();

    // Disable very long press mode
    DebounceSwitch::getInstance()->withNoVeryLongPress();

    DebounceSwitch::getInstance()->addSwitch(TEST_PIN, DebounceSwitchStyle::PRESS_LOW_PULLUP, 
        [](DebounceSwitchState *switchState, void *) {
            Log.info("pin=%d state=%s", switchState->getPin(), switchState->getPressStateName());
            if (switchState->getPressState() == DebouncePressState::TAP) {
                Log.info("%d taps", switchState->getTapCount());
            }
    });
}
```

This changes the default, then adds a switch. If you change the default again, the existing switch will not be changed, but future calls to addSwitch() will be affected.

### Per switch configuration

```
#include "DebounceSwitchRK.h"

SYSTEM_THREAD(ENABLED);

SerialLogHandler logHandler;

void switchCallback(DebounceSwitchState *switchState, void *) {
    const char *name = (switchState->getPin() == D2) ? "toggle D2" : "button D3";
    Log.info("%s state=%s", name, switchState->getPressStateName());

    if (switchState->getPressState() == DebouncePressState::TAP) {
        Log.info("%d taps", switchState->getTapCount());
    }
}

void setup() {
    DebounceSwitch::getInstance()->setup();

    DebounceSwitchState *sw;
    
    // Toggle switch
    sw = DebounceSwitch::getInstance()->addSwitch(D2, DebounceSwitchStyle::TOGGLE_PULLUP, switchCallback);
    // Give the toggle switch a longer debounce time
    sw->withDebounceMs(100);

    // Push button switch
    sw = DebounceSwitch::getInstance()->addSwitch(D3, DebounceSwitchStyle::PRESS_LOW_PULLUP, switchCallback);
    // Disable long press and very long press mode on the button
    sw->withNoLongPress();
}

void loop() {
}
```

The same methods also work for on the DebounceSwitchState object, so you can set the parameters per-switch.


## Usage Patterns

### Hardware GPIO, Simple Callback

```
#include "Particle.h"

#include "DebounceSwitchRK.h"

SYSTEM_THREAD(ENABLED);

SerialLogHandler logHandler;

void switchCallback(DebounceSwitchState *switchState, void *context) {
    // This function is called from a worker thread with a small (1K) stack.
    // You should avoid any large, lengthy operations here.
    Log.info("pin=%d state=%s", switchState->getPin(), switchState->getPressStateName());
    if (switchState->getPressState() == DebouncePressState::TAP) {
        Log.info("%d taps", switchState->getTapCount());
    }    
}

void setup() {
    DebounceSwitch::getInstance()->setup();

    DebounceSwitch::getInstance()->addSwitch(D3, DebounceSwitchStyle::PRESS_LOW_PULLUP, switchCallback);
}

void loop() {
}
```

This is the simplest model for GPIO connected switches. Of note:

```
DebounceSwitch::getInstance()->setup();
```

Be sure to call this from global setup(). It's required!


```
DebounceSwitch::getInstance()->addSwitch(D3, DebounceSwitchStyle::PRESS_LOW_PULLUP, switchCallback);
```

This is how you add a switch to be handled. You can add multiple switches in this manner.

The parameters of interest are:

- `D3` The pin the switch is connected to
- `DebounceSwitchStyle::PRESS_LOW_PULLUP` The switch is a momentary (push-button) switch and connects the GPIO to GND (active low). It uses the MCU's hardware pull-up to handle the case where the switch is open (not connected) to bring the signal HIGH.
- `switchCallback` The function below.

This is the callback function:

```
void switchCallback(DebounceSwitchState *switchState, void *context) {
    Log.info("pin=%d state=%s", switchState->getPin(), switchState->getPressStateName());
    if (switchState->getPressState() == DebouncePressState::TAP) {
        Log.info("%d taps", switchState->getTapCount());
    }    
}
```

The callback is called from a worker thread with a small (1024 byte) stack, so you should avoid doing operations which block or use a lot of stack space. This example just logs things to debug USB serial using `Log.info`.

Note that there is no code needed from loop(). This means that if your code is blocking loop, it won't adversely affect the handling of buttons.

### Hardware GPIO, Lambda Callback

The callback can be a C++11 lambda function:

```
DebounceSwitch::getInstance()->addSwitch(D2, DebounceSwitchStyle::TOGGLE, [](DebounceSwitchState *switchState, void *) {
    Log.info("toggle state=%s", switchState->getPressStateName());
});
```

This is the actual lambda part:

```
[](DebounceSwitchState *switchState, void *) {
    Log.info("toggle state=%s", switchState->getPressStateName());
}
```

This declared a function in-line to be called when the event occurs. The function has this prototype:

```
void callback(DebounceSwitchState *switchState, void *)
```

It's important to remember the callback part is execute later, not inline with the rest of the code. 

The `[]` part is the lambda capture, which allows local variables and `this` (for C++ class instances) to be captured for use within the lambda function. In this case, there is nothing to capture.




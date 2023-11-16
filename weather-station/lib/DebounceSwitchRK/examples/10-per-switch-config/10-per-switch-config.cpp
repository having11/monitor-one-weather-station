#include "Particle.h"

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
    // Comment this out to wait for USB serial connections to see more debug logs
    // waitFor(Serial.isConnected, 15000);

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
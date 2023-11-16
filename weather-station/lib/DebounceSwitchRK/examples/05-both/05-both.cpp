#include "Particle.h"

#include "DebounceSwitchRK.h"

SYSTEM_THREAD(ENABLED);

SerialLogHandler logHandler;


void setup() {
    // Comment this out to wait for USB serial connections to see more debug logs
    // waitFor(Serial.isConnected, 15000);

    DebounceSwitch::getInstance()->setup();

    // It's usually easier to use DebounceSwitchStyle::TOGGLE_PULLUP, but you can
    // do it separately if you prefer.
    pinMode(D2, INPUT_PULLUP);
    pinMode(D3, INPUT_PULLUP);

    DebounceSwitch::getInstance()->addSwitch(D2, DebounceSwitchStyle::TOGGLE, [](DebounceSwitchState *switchState, void *) {
        Log.info("toggle state=%s", switchState->getPressStateName());
    });

    DebounceSwitch::getInstance()->addSwitch(D3, DebounceSwitchStyle::PRESS_LOW, [](DebounceSwitchState *switchState, void *) {
        Log.info("button state=%s", switchState->getPressStateName());
        if (switchState->getPressState() == DebouncePressState::TAP) {
            Log.info("%d taps", switchState->getTapCount());
        }
    });

}

void loop() {
}
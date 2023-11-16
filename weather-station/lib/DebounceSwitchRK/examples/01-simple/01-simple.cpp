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
    // Comment this out to wait for USB serial connections to see more debug logs
    // waitFor(Serial.isConnected, 15000);

    DebounceSwitch::getInstance()->setup();

    DebounceSwitch::getInstance()->addSwitch(D3, DebounceSwitchStyle::PRESS_LOW_PULLUP, switchCallback);
}

void loop() {
}
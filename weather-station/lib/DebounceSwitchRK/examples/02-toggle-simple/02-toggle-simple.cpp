#include "Particle.h"

#include "DebounceSwitchRK.h"

SYSTEM_THREAD(ENABLED);

SerialLogHandler logHandler;

void toggleSwitchCallback(DebounceSwitchState *switchState, void *context) {
    Log.info("state=%s", switchState->getPressStateName());
}

void setup() {
    // Comment this out to wait for USB serial connections to see more debug logs
    // waitFor(Serial.isConnected, 15000);

    DebounceSwitch::getInstance()->setup();

    DebounceSwitch::getInstance()->addSwitch(D2, DebounceSwitchStyle::TOGGLE_PULLUP, toggleSwitchCallback);
}

void loop() {
}
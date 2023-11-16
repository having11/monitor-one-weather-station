#include "Particle.h"

#include "DebounceSwitchRK.h"

SYSTEM_THREAD(ENABLED);

SerialLogHandler logHandler;

void interruptHandler();
DebounceSwitchState *notifyHandler;

void setup() {
    // Comment this out to wait for USB serial connections to see more debug logs
    // waitFor(Serial.isConnected, 15000);

    DebounceSwitch::getInstance()->setup();

    pinMode(D2, INPUT_PULLUP);
    attachInterrupt(D2, interruptHandler, CHANGE);

    notifyHandler = DebounceSwitch::getInstance()->addNotifySwitch(DebounceSwitchStyle::TOGGLE, [](DebounceSwitchState *switchState, void *) {
        Log.info("state=%s", switchState->getPressStateName());
    });

    // Notify of initial state
    notifyHandler->notify(pinReadFast(D2));

}

void loop() {
}

void interruptHandler() {
    // It's safe to call this from an ISR
    if (notifyHandler) {
        notifyHandler->notify(pinReadFast(D2));
    }
}

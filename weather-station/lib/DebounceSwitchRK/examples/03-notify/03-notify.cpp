#include "Particle.h"

#include "DebounceSwitchRK.h"

SYSTEM_THREAD(ENABLED);

SerialLogHandler logHandler;

void interruptHandler();
DebounceSwitchState *notifyHandler;

pin_t TEST_PIN = D3;

void setup() {
    // Comment this out to wait for USB serial connections to see more debug logs
    // waitFor(Serial.isConnected, 15000);

    DebounceSwitch::getInstance()->setup();

    pinMode(TEST_PIN, INPUT_PULLUP);
    attachInterrupt(TEST_PIN, interruptHandler, CHANGE);

    notifyHandler = DebounceSwitch::getInstance()->addSwitch(DebounceSwitch::NOTIFY_PIN, DebounceSwitchStyle::PRESS_LOW, [](DebounceSwitchState *switchState, void *) {
        Log.info("state=%s", switchState->getPressStateName());
        if (switchState->getPressState() == DebouncePressState::TAP) {
            Log.info("%d taps", switchState->getTapCount());
        }
    });

}

void loop() {
}

void interruptHandler() {
    // It's safe to call this from an ISR
    if (notifyHandler) {
        notifyHandler->notify(pinReadFast(TEST_PIN));
    }
}

#include "Particle.h"

#include "DebounceSwitchRK.h"
#include "MCP23008-RK.h"

SYSTEM_THREAD(ENABLED);

SerialLogHandler logHandler;

MCP23008 gpio(Wire3, 0x20);

void setup() {
    waitFor(Serial.isConnected, 15000);

    DebounceSwitch::getInstance()->setup();

    // Turn on power on Tracker CAN_5V
    pinMode(CAN_PWR, OUTPUT);
    digitalWrite(CAN_PWR, HIGH);
    delay(200);

    // Initialize MCP23008
    gpio.begin();
    gpio.pinMode(2, INPUT_PULLUP);

    DebounceSwitch::getInstance()->addSwitch(
        DebounceSwitch::NOTIFY_PIN, DebounceSwitchStyle::PRESS_LOW, 
        [](DebounceSwitchState *switchState, void *) {
            // Called to notify of switch operations
            Log.info("pin=%d state=%s", switchState->getPin(), switchState->getPressStateName());
            if (switchState->getPressState() == DebouncePressState::TAP) {
                Log.info("%d taps", switchState->getTapCount());
            }
        }, NULL,
        [](DebounceSwitchState *switchState, void *) {
            // Called to poll for values
            Wire3.lock();
            bool bResult = (bool) gpio.digitalRead(2);
            Wire3.unlock();

            return bResult;
        }, NULL);
}


void loop() {
}
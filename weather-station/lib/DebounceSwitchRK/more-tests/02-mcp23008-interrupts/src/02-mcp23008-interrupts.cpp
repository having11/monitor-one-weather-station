#include "Particle.h"

#include "DebounceSwitchRK.h"
#include "MCP23008-RK.h"

SYSTEM_THREAD(ENABLED);

SerialLogHandler logHandler;

MCP23008 gpio(Wire3, 0x20);

pin_t GPIO_INT_PIN = A3;

uint16_t SWITCH_PIN = 2; // GP2

void setup() {
    waitFor(Serial.isConnected, 15000);

    DebounceSwitch::getInstance()->setup();

    // Turn on power on Tracker CAN_5V
    pinMode(CAN_PWR, OUTPUT);
    digitalWrite(CAN_PWR, HIGH);
    delay(200);

    // Initialize MCP23008
    gpio.begin();

    // When using interrupt mode, you need to assocate a physical MCU pin as an interrupt pin
    // from the MCP23008 INT output
    gpio.enableInterrupts(GPIO_INT_PIN, MCP23008InterruptOutputType::OPEN_DRAIN);

    gpio.pinMode(SWITCH_PIN, INPUT_PULLUP);

    DebounceSwitchState *sw = DebounceSwitch::getInstance()->addSwitch(
        DebounceSwitch::NOTIFY_PIN, DebounceSwitchStyle::PRESS_LOW,
        [](DebounceSwitchState *switchState, void *) {
            // Called to notify of switch operations
            Log.info("state=%s", switchState->getPressStateName());
            if (switchState->getPressState() == DebouncePressState::TAP) {
                Log.info("%d taps", switchState->getTapCount());
            }
        }, NULL);

    gpio.attachInterrupt(SWITCH_PIN, CHANGE, 
        [sw](bool bValue) {
        // This code runs in a worker thread with a 1024 byte stack, so avoid doing
        // anything that requires a long time or stack here.
        // Log.info("bValue=%d", bValue);
        sw->notify(bValue);
    });

}


void loop() {
}
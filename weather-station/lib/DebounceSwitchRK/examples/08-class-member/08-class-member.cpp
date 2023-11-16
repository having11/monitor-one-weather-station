#include "Particle.h"

#include "DebounceSwitchRK.h"

SYSTEM_THREAD(ENABLED);

SerialLogHandler logHandler;

class MyButtonClass {
public:
    MyButtonClass(pin_t pin);
    virtual ~MyButtonClass();

    void setup();

    void callback(DebounceSwitchState *switchState);

protected:
    pin_t pin;
};

MyButtonClass::MyButtonClass(pin_t pin) : pin(pin) {
    
}

MyButtonClass::~MyButtonClass() {
    
}

void MyButtonClass::setup() {
    // OK to call setup() more than once (if there are multiple instances of MyButtonClass)
    DebounceSwitch::getInstance()->setup();

    DebounceSwitch::getInstance()->addSwitch(pin, DebounceSwitchStyle::PRESS_LOW_PULLUP, 
        &MyButtonClass::callback, this);
}

void MyButtonClass::callback(DebounceSwitchState *switchState) {
    Log.info("pin=%d state=%s", switchState->getPin(), switchState->getPressStateName());
    if (switchState->getPressState() == DebouncePressState::TAP) {
        Log.info("%d taps", switchState->getTapCount());
    }    
}

MyButtonClass myButtonClass(D3);


void setup() {
    // Comment this out to wait for USB serial connections to see more debug logs
    // waitFor(Serial.isConnected, 15000);

    myButtonClass.setup();

}

void loop() {
}
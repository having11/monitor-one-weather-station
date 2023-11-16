#include "Particle.h"
#include "dct.h"
#include "adp8866_led.h"

SYSTEM_MODE(MANUAL);

using namespace particle;

SerialLogHandler logHandler(115200, LOG_LEVEL_NONE, {
    { "app", LOG_LEVEL_ALL },
});

enum class TestStep {
    TOGGLE = 0,
    BRIGHTNESS,
    COLOR,
    SPEED_PATTERN,
    INVALID
};

STARTUP(
    // set done flag
    uint8_t read_value = 0x01;
    dct_read_app_data_copy(DCT_SETUP_DONE_OFFSET, &read_value, 1);
    if(read_value != 1)
    {
        const uint8_t write_value = 1;
        dct_write_app_data(&write_value, DCT_SETUP_DONE_OFFSET, 1);
    }
);
ADP8866 *pAdp8866 = nullptr;
ADP8866_LED *pLEDs[9] = {nullptr};
int ledIndex = static_cast<int>(IscLed::LED1);
TestStep testStep = TestStep::INVALID;
static uint32_t tick = millis();

void setup()
{
    Serial.begin(115200);
    waitUntil(Serial.isConnected);
    delay(300);
    Log.info("### ADP8866-LED Test ###");
    pAdp8866 = new ADP8866(Wire,PIN_INVALID);
    for (int i = 0; i < 9; i++)
    {
        pLEDs[i] = new ADP8866_LED(*pAdp8866,static_cast<IscLed>(i));
        pLEDs[i]->brightness(0xFF);
    }
    pAdp8866->set_leds_off(LED_ALL_BIT_MASK);
    testStep = TestStep::TOGGLE;
    Log.info("### Enter TOGGLE Test ###");
}


void toggle_test()
{
    static uint32_t count = 2;
    if (millis() - tick >= 500)
    {   
        pLEDs[ledIndex]->toggle();
        Log.info("%s: LED_%d  state == %s",__FUNCTION__,ledIndex + 1,pLEDs[ledIndex]->isOn() ? "ON" : "OFF");
        if (--count == 0)
        {
            count = 2;
            if (++ledIndex > static_cast<int>(IscLed::LED9))
            {
                ledIndex = static_cast<int>(IscLed::LED1);
                testStep = TestStep::BRIGHTNESS;                
                Log.info("### Enter BRIGHTNESS Test ###");
                pAdp8866->set_leds_on(LED_ALL_BIT_MASK);
            }
        }
        tick = millis();
    }
}



void brightness_test()
{
    static uint8_t brightness = 0;
    if (millis() - tick >= 100)
    {         
        for (int i = 0; i < 9; i++)
        {
            pLEDs[i]->brightness(brightness);
        }
        brightness += 10;
        Log.info("%s: brightness == %d",__FUNCTION__,brightness);        
        if (brightness >= 250)
        {
            testStep = TestStep::SPEED_PATTERN;
            brightness = 64;
            Log.info("### Enter SPEED PATTERN Test ###");
            for (int i = 0; i < static_cast<int>(IscLed::LED6); i++)
            {
                pLEDs[i]->off();
                pLEDs[i]->set_pattern(LED_PATTERN_FADE);
                pLEDs[i]->brightness(brightness);
                pLEDs[i]->set_speed(LED_SPEED_SLOW);
                pLEDs[i]->on();
            }
            for (int i = static_cast<int>(IscLed::LED6); i <= static_cast<int>(IscLed::LED9); i++)
            {
                pLEDs[i]->off();
                pLEDs[i]->set_pattern(LED_PATTERN_BLINK);
                pLEDs[i]->brightness(brightness);
                pLEDs[i]->set_speed(LED_SPEED_SLOW);
                pLEDs[i]->on();
            }
             Log.info("SPEED == %d",static_cast<int>(LED_SPEED_SLOW));
        }
        tick = millis();
    }
}

void speed_pattern_test()
{
    static LEDSpeed speed = LED_SPEED_SLOW;
    if (millis() - tick >= 5000)
    {         
        switch (speed)
        {
        case LED_SPEED_SLOW:
            speed = LED_SPEED_NORMAL;
            break;
        case LED_SPEED_NORMAL:
            speed = LED_SPEED_FAST;
            break;        
        default:
            speed = LED_SPEED_SLOW;
            break;
        }
        Log.info("%s: SPEED == %d",__FUNCTION__,static_cast<int>(speed));

        for (int i = 0; i < static_cast<int>(IscLed::LED6); i++)
        {
            pLEDs[i]->set_speed(speed);
        }
        for (int i = static_cast<int>(IscLed::LED6); i <= static_cast<int>(IscLed::LED9); i++)
        {
            pLEDs[i]->off();
            pLEDs[i]->set_speed(speed);
        }
        pAdp8866->set_leds_on(LED6_BIT_MASK | LED7_BIT_MASK | LED8_BIT_MASK | LED9_BIT_MASK);

        tick = millis();
    }
}

void loop()
{
    switch (testStep)
    {
    case TestStep::TOGGLE:
        toggle_test();
        break;
    case TestStep::BRIGHTNESS:
        brightness_test();
        break;
    case TestStep::SPEED_PATTERN:
        speed_pattern_test();
        break;         
    default:
        break;
    }
}
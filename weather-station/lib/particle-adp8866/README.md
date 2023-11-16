# particle-adp8866
Particle driver library to support ADP8866 LED Driver

## ADP8866
The class of ADP8866 LED Driver.
### Todo List
- [ ] Heartbeat support

## ADP8866_LED
The class provides some abilities for the LED.
Such as:
- [x] Read/Write the brightness
- [x] Read/Write the pattern
- [x] Read/Write the speed
- [x] Update the on/off state
- [x] Toggle the `LED` state
### Example
```C++
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
```

## ADP8866_RGB
The class to group the LEDs as RGB. 
And provide some interfaces to control the `RGB`.
Such as:
- [x] Read/Write the color
- [x] Read/Write the brightness
- [x] Read/Write the pattern
- [x] Read/Write the speed
- [x] Update the on/off state
- [x] Toggle the RGB state
### Example
```C++
#include "Particle.h"
#include "dct.h"
#include "adp8866_rgb.h"

SYSTEM_MODE(MANUAL);

using namespace particle;

SerialLogHandler logHandler(115200, LOG_LEVEL_NONE, {
    { "app", LOG_LEVEL_ALL },
});

enum class TestStep {
    TOGGLE = 0,
    BRIGHTNESS,
    SPEED_PATTERN,
    COLOR,
    INVALID
};

enum class TestColor {
    RED = 0,
    GREEN,
    BLUE,
    MIX,
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
IRGB *RGBs[3] = {nullptr};
TestStep testStep = TestStep::INVALID;
int rgbIndex = 0;


static uint32_t tick = millis();

void setup()
{
    Serial.begin(115200);
    waitUntil(Serial.isConnected);
    delay(300);
    Log.info("### ADP8866-RGB Test ###");
    pAdp8866 = new ADP8866(Wire,PIN_INVALID);
    RGBs[0] = new ADP8866_RGB(IscLed::LED2,IscLed::LED1,IscLed::LED3,pAdp8866);
    RGBs[1] = new ADP8866_RGB(IscLed::LED5,IscLed::LED4,IscLed::LED6,pAdp8866);
    RGBs[2] = new ADP8866_RGB(IscLed::LED8,IscLed::LED7,IscLed::LED9,pAdp8866);
    testStep = TestStep::TOGGLE;
    Log.info("### Enter TOGGLE Test ###");
}


void toggle_test()
{
    static uint32_t count = 2;
    if (millis() - tick >= 500)
    {   
        RGBs[rgbIndex]->toggle();
        Log.info("%s: RGB_%d  state == %s",__FUNCTION__,rgbIndex,RGBs[rgbIndex]->isOn() ? "ON" : "OFF");
        if (--count == 0)
        {
            count = 2;
            if (++rgbIndex > 2)
            {
                rgbIndex = 0;
                testStep = TestStep::BRIGHTNESS;                
                Log.info("### Enter BRIGHTNESS Test ###");
                for (int i = 0; i < 3; i++)
                {
                    RGBs[i]->on();
                }                
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
        for (int i = 0; i < 3; i++)
        {
            RGBs[i]->brightness(brightness);
        }
        brightness += 10;
        Log.info("%s: brightness == %d",__FUNCTION__,brightness);        
        if (brightness >= 250)
        {
            testStep = TestStep::SPEED_PATTERN;
            brightness = 0xFF;
            Log.info("### Enter SPEED PATTERN Test ###");
            RGBs[0]->setPattern(LED_PATTERN_FADE);
            RGBs[1]->setPattern(LED_PATTERN_FADE);
            RGBs[2]->setPattern(LED_PATTERN_BLINK);
            for (int i = 0; i < 3; i++)
            {
                RGBs[i]->brightness(brightness);
                RGBs[i]->setSpeed(LED_SPEED_SLOW);
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
            Log.info("### Enter COLOR Test ###");
            testStep = TestStep::COLOR;
            for (int i = 0; i < 3; i++)
            {
                RGBs[i]->setPattern(LED_PATTERN_SOLID);
            }     
            return;       
            break;
        }
        Log.info("%s: SPEED == %d",__FUNCTION__,static_cast<int>(speed));
        for (int i = 0; i < 3; i++)
        {
            RGBs[i]->setSpeed(speed);
        }        
        tick = millis();
    }
}



void color_test()
{
    static uint8_t r = 0,g = 0,b = 0,temp = 0;
    static TestColor testColor = TestColor::RED;
    if (millis() - tick >= 100)
    {         
        temp += 10;
        switch (testColor)
        {
        case TestColor::RED:
            r = temp;
            break;
        case TestColor::GREEN:
            g = temp;
            break;
        case TestColor::BLUE:
            b = temp;
            break;                    
        default:
            r = random(256);
            g = random(256);
            b = random(256);
            break;
        }
        for (int i = 0; i < 3; i++)
        {
            RGBs[i]->color(r,g,b);
        }

        Log.info("%s: Red == %03d,Green == %03d,Blue == %03d",__FUNCTION__,r,g,b);
        if (temp >= 250)
        {
            r = 0;
            g = 0;
            b = 0;
            temp = 0;
            switch (testColor)
            {
            case TestColor::RED:
                testColor = TestColor::GREEN;
                break;
            case TestColor::GREEN:
                testColor = TestColor::BLUE;
                break;
            case TestColor::BLUE:
                testColor = TestColor::MIX;
                break;                    
            default:

                break;
            }
        }
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
    case TestStep::COLOR:
        color_test();
        break;        
    case TestStep::SPEED_PATTERN:
        speed_pattern_test();
        break;         
    default:
        break;
    }
}
```
## Dependency Library
- [particle-led](https://github.com/particle-iot/particle-led)
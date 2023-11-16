#include "adp8866_led.h"

using namespace spark;
using namespace particle;

ADP8866_LED::ADP8866_LED(ADP8866 &drv, IscLed led) : _drv(drv),
                                                     _led(led)
{
    if (!_drv.is_inited())
    {
        _drv.begin();
    }
    if (_led >= IscLed::LED6)
    {
        _offTime = static_cast<uint8_t>(IscOffTime::SC_OFF_ADV_0_0_0_S) * 18 + 1;
    }
    set_pattern(_pattern);
    set_speed(_speed);
}

ADP8866_LED::~ADP8866_LED()
{
    
}

ADP8866 &ADP8866_LED::get_driver()
{
    return _drv;
}

IscLed ADP8866_LED::get_led_index()
{
    return _led;
}

ADP8866_LED &ADP8866_LED::brightness(uint8_t brightness, bool update)
{
    uint8_t value = (brightness * SINK_CURRENT_MAX) / 0xFF;
    if (LED_PATTERN_FADE == _pattern)
    {
        _drv.set_backlight_current(value);
    }
    else
    {
        _drv.set_led_sink_current(_led,value);
    }
    _brightness = brightness;
    return *this;
}

uint8_t ADP8866_LED::brightness()
{
    return _brightness;
}

ADP8866_LED &ADP8866_LED::set_pattern(LEDPattern pattern)
{
    switch (pattern)
    {
    case LED_PATTERN_FADE:
        _drv.set_backlight_fade_in_time(_fadeInTime)
            .set_backlight_fade_out_time(_fadeOutTime);
        break;
    case LED_PATTERN_BLINK:
        _drv.set_leds_fade_in_time(IscFadeTime::FADE_DISABLE)
            .set_leds_fade_out_time(IscFadeTime::FADE_DISABLE);
        break;        
    default:
        _drv.set_leds_fade_in_time(IscFadeTime::FADE_DISABLE)
            .set_leds_fade_out_time(IscFadeTime::FADE_DISABLE);
        break;
    }
    _pattern = pattern;
    return *this;    
}

LEDPattern ADP8866_LED::pattern()
{
    return _pattern;
}

ADP8866_LED &ADP8866_LED::set_speed(LEDSpeed speed)
{
    _speed = speed;
    set_period(pattern_period(_pattern,speed));
    return *this;    
}

ADP8866_LED &ADP8866_LED::set_period(uint16_t period)
{
    IscOnTime onTime;
    uint8_t offTime;
    IscFadeTime fadeInTime,fadeOutTime;
    if (LED_PATTERN_FADE == _pattern)
    {
        if (period >= static_cast<uint16_t>(PatternPeriod::FADE_SLOW))
        {
            fadeOutTime = fadeInTime = IscFadeTime::FADE_1_7_5_S;
            onTime = IscOnTime::SC_ON_0_7_5_S;
            if (_led >= IscLed::LED6)
            {
                offTime = static_cast<uint8_t>(IscOffTime::SC_OFF_ADV_0_0_0_S) * 18 + 1;         // 1.8 second
            }
            else
            {
                offTime = static_cast<uint8_t>(IscOffTime::SC_OFF_1_8_S);
            }            
        }
        else if (period >= static_cast<uint16_t>(PatternPeriod::FADE_NORMAL))
        {
            fadeOutTime = fadeInTime = IscFadeTime::FADE_1_5_0_S;
            onTime = IscOnTime::SC_ON_0_4_0_S;
            if (_led >= IscLed::LED6)
            {
                offTime = static_cast<uint8_t>(IscOffTime::SC_OFF_ADV_0_0_0_S) * 6 + 1;         // 0.6 second
            }
            else
            {
                offTime = static_cast<uint8_t>(IscOffTime::SC_OFF_0_6_S);
            }
        }
        else
        {
            fadeOutTime = fadeInTime = IscFadeTime::FADE_0_1_5_S;
            onTime = IscOnTime::SC_ON_0_1_0_S;
            if (_led >= IscLed::LED6)
            {
                offTime = static_cast<uint8_t>(IscOffTime::SC_OFF_ADV_0_0_0_S) * 6 + 1;         // 0.6 second
            }
            else
            {
                offTime = static_cast<uint8_t>(IscOffTime::SC_OFF_0_6_S);
            }
        }        
    }
    else if(LED_PATTERN_BLINK == _pattern)
    {
        fadeOutTime = fadeInTime = IscFadeTime::FADE_DISABLE;      
        if (_led >= IscLed::LED6)
        {
            if (period >= static_cast<uint16_t>(PatternPeriod::BLINK_SLOW))
            {
                onTime = IscOnTime::SC_ON_0_2_5_S;
                offTime = static_cast<uint8_t>(IscOffTime::SC_OFF_ADV_0_0_0_S) * 2 + 1;         // 0.2 second

            }
            else if (period >= static_cast<uint16_t>(PatternPeriod::BLINK_NORMAL))
            {
                onTime = IscOnTime::SC_ON_0_1_0_S;
                offTime = static_cast<uint8_t>(IscOffTime::SC_OFF_ADV_0_0_1_S);
            }
            else
            {
                onTime = IscOnTime::SC_ON_0_0_5_S;
                offTime = static_cast<uint8_t>(IscOffTime::SC_OFF_ADV_0_0_1_S);
            }        
        }
        else
        {
             if (period >= static_cast<uint16_t>(PatternPeriod::BLINK_SLOW))
            {
                onTime = IscOnTime::SC_ON_0_2_5_S;
                fadeOutTime = IscFadeTime::FADE_0_2_5_S;
                offTime = static_cast<uint8_t>(IscOffTime::SC_OFF_0_6_S);
            }
            else if (period >= static_cast<uint16_t>(PatternPeriod::BLINK_NORMAL))
            {
                onTime = IscOnTime::SC_ON_0_1_0_S;
                fadeOutTime = IscFadeTime::FADE_0_1_0_S;
                offTime = static_cast<uint8_t>(IscOffTime::SC_OFF_0_6_S);
            }
            else
            {
                onTime = IscOnTime::SC_ON_0_0_5_S;
                fadeOutTime = IscFadeTime::FADE_0_0_5_S;
                offTime = static_cast<uint8_t>(IscOffTime::SC_OFF_0_6_S);
            } 
        }
    }
    else
    {
        onTime = IscOnTime::SC_ON_0_0_0_S;
        fadeOutTime = fadeInTime = IscFadeTime::FADE_DISABLE;     
        offTime = static_cast<uint8_t>(IscOffTime::SC_OFF_DISABLE);
    }
    update_on_time(onTime);
    update_off_time(offTime);
    update_fade_in_time(fadeInTime);
    update_fade_out_time(fadeOutTime);
    _period = period;
    return *this;
}

uint16_t ADP8866_LED::period()
{
    return _period;
}

ADP8866_LED &ADP8866_LED::on()
{
    if (LED_PATTERN_FADE == _pattern)
    {
        _drv.leds_backlight_select(_led)
            .set_backlight(true);
    }
    else
    {
        _drv.set_led_on_off(_led,true);
    }  
    _isOn = true;
    return *this;    
}

ADP8866_LED &ADP8866_LED::off()
{
    if (LED_PATTERN_FADE == _pattern)
    {
        _drv.leds_backlight_select(_led,false);
    }
    else
   {
        _drv.set_led_on_off(_led,false); 
   } 
    _isOn = false;
    return *this;    
}

ADP8866_LED &ADP8866_LED::toggle()
{
    _isOn == true ? off() : on();
    return *this;
}

ADP8866_LED &ADP8866_LED::update_on_time(IscOnTime time)
{
    _onTime = time;
    _drv.set_leds_on_time(_onTime);
    return *this;
}

ADP8866_LED &ADP8866_LED::update_off_time(uint8_t time)
{
    _offTime = time;
    _drv.set_led_off_time(_led,_offTime);    
    return *this;
}

ADP8866_LED &ADP8866_LED::update_fade_in_time(IscFadeTime time)
{

    if (LED_PATTERN_FADE == _pattern)
    {
        _drv.set_backlight_fade_in_time(time);
    }
    else
    {
        _drv.set_leds_fade_in_time(time);
    }
    _fadeInTime = time;
    return *this;
}

ADP8866_LED &ADP8866_LED::update_fade_out_time(IscFadeTime time)
{
    
    if (LED_PATTERN_FADE == _pattern)
    {
        _drv.set_backlight_fade_out_time(time);
    }
    else
    {
        _drv.set_leds_fade_out_time(time);
    }
    _fadeOutTime = time;
    return *this;
}
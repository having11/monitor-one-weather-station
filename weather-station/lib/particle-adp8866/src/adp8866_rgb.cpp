#include "adp8866_rgb.h"

using namespace spark;
using namespace particle;



ADP8866_RGB::ADP8866_RGB(IscLed led_r,IscLed led_g,IscLed led_b,ADP8866 *pDrv) : _led_r(led_r),
                                                                                _led_g(led_g),
                                                                                _led_b(led_b),
                                                                                _pDrv(pDrv)
{
    if (_led_r != IscLed::INVALID)
    {
        _pRed = new ADP8866_LED(*_pDrv,_led_r);
    }
    if (_led_g != IscLed::INVALID)
    {
        _pGreen = new ADP8866_LED(*_pDrv,_led_g);
    }
    if (_led_b != IscLed::INVALID)
    {
        _pBlue = new ADP8866_LED(*_pDrv,_led_b);
    }
    setPattern(_pattern);
    brightness(_brightness);
    _pDrv->set_backlight_current(_brightness);
    off();
    _cleanupMemory = true;
}



ADP8866_RGB::ADP8866_RGB(ADP8866_LED *pRed,ADP8866_LED *pGreen,ADP8866_LED *pBlue) : _pRed(pRed),
                                                                                     _pGreen(pGreen),
                                                                                     _pBlue(pBlue)
{
    _pDrv = &_pRed->get_driver();
    setPattern(_pattern);
    brightness(_brightness);
    _pDrv->set_backlight_current(_brightness);
    off();
}


ADP8866_RGB::~ADP8866_RGB()
{
    if (_cleanupMemory)
    {
        if (_pRed)
        {
            delete _pRed;
            _pRed = nullptr;    
        }
        if (_pGreen)
        {
            delete _pGreen;
            _pGreen = nullptr;    
        }
        if (_pBlue)
        {
            delete _pBlue;
            _pBlue = nullptr;    
        }  
    }   
}


void ADP8866_RGB::color(uint8_t r,uint8_t g,uint8_t b)
{
    setColor(static_cast<uint32_t>((r << 16) | (g << 8) | b));
}


void ADP8866_RGB::setColor(uint32_t color)
{
    if (_pattern == LED_PATTERN_FADE)
    {
        if (_pRed)
        {
            if(red(color) != 0x00)
            {
                if (_isOn == true)
                {
                    _pRed->on();
                }
            }
            else
            {
                _pRed->off();
            }
        }
        if (_pGreen)
        {
            if(green(color) != 0x00)
            {
                if (_isOn == true)
                {
                    _pGreen->on();
                }
            }
            else
            {
                _pGreen->off();
            }
        }        
        if (_pBlue)
        {
            if(blue(color) != 0x00)
            {
                if (_isOn == true)
                {
                    _pBlue->on();
                }
            }
            else
            {
                _pBlue->off();
            }
        }
    }
    else
    {
        if (_pRed)
        {
            _pRed->brightness((red(color) * _brightness) / 0xFF);
        }
        if (_pGreen)
        {
            _pGreen->brightness((green(color) * _brightness) / 0xFF);
        }
        if (_pBlue)
        {
            _pBlue->brightness((blue(color)  * _brightness) / 0xFF);
        }
    }    
    _color = color;
}

void ADP8866_RGB::brightness(uint8_t brightness, bool update)
{
#if 0
    uint8_t range = OUTPUT_LEVEL_MIN - OUTPUT_LEVEL_MAX;
    uint8_t value = OUTPUT_LEVEL_MIN - (brightness * range) / 0xFF;
    _pDrv->update_output_level(value);
#else
    if (_pattern == LED_PATTERN_FADE)
    {
        _pDrv->set_backlight_current((brightness * BACKLIGHT_CURRENT_MAX) / 0xFF);
    }
    else
    {
        if (_pRed)
        {
            _pRed->brightness((red(_color) * brightness) / 0xFF);
        }
        if (_pGreen)
        {
            _pGreen->brightness((green(_color) * brightness) / 0xFF);
        }
        if (_pBlue)
        {
            _pBlue->brightness((blue(_color)  * brightness) / 0xFF);
        }
    }
#endif
    _brightness = brightness;
}

void ADP8866_RGB::setPattern(LEDPattern pattern)
{
    if (_pRed)
    {
        _pRed->off();
        _pRed->set_pattern(pattern);
    }
    if (_pGreen)
    {
        _pGreen->off();
        _pGreen->set_pattern(pattern);
    }
    if (_pBlue)
    {
         _pBlue->off();
        _pBlue->set_pattern(pattern); 
    }
    _pattern = pattern;
    setSpeed(_speed); 
    brightness(_brightness); 
    if (_isOn == true)
    {
        on();
    }  
}

void ADP8866_RGB::setSpeed(LEDSpeed speed)
{
    if (_pRed)
    {
        _pRed->set_speed(speed);
    }
    if (_pGreen)
    {
        _pGreen->set_speed(speed);
    }
    if (_pBlue)
    {
        _pBlue->set_speed(speed);
    }
    _speed = speed;
}

void ADP8866_RGB::setPeriod(uint16_t period)
{
    if (_pRed)
    {
        _pRed->set_period(period);
    }
    if (_pGreen)
    {
        _pGreen->set_period(period);
    }
    if (_pBlue)
    {
        _pBlue->set_period(period);
    }   
    _period = period;
}

void ADP8866_RGB::on()
{
    if (_pattern == LED_PATTERN_FADE)
    {
        if (_pRed)
        {
            if(red(_color) != 0x00)
            {
                _pRed->on();
            }
        }
        if (_pGreen)
        {
            if(green(_color) != 0x00)
            {
                _pGreen->on();
            }
        }
        if (_pBlue)
        {
            if(blue(_color) != 0x00)
            {
                _pBlue->on();
            }
        }
        _pDrv->set_backlight(false)
            .set_backlight(true);        
    }
    else
    {
        uint16_t bitMask = 0;
        if (_pRed)
        {
            bitMask |= (1 << static_cast<uint8_t>(_pRed->get_led_index()));
        }        
        if (_pGreen)
        {
            bitMask |= (1 << static_cast<uint8_t>(_pGreen->get_led_index()));
        }        
        if (_pBlue)
        {
            bitMask |= (1 << static_cast<uint8_t>(_pBlue->get_led_index()));
        }
        _pDrv->set_leds_on(bitMask);   
    }
    _isOn = true;
}

void ADP8866_RGB::off()
{
    if (_pattern == LED_PATTERN_FADE)
    {
        if (_pRed)
        {
            _pRed->off();
        }
        if (_pGreen)
        {
            _pGreen->off();
        }
        if (_pBlue)
        {
            _pBlue->off();
        }   
    }
    else
    {
        uint16_t bitMask = 0;
        if (_pRed)
        {
            bitMask |= (1 << static_cast<uint8_t>(_pRed->get_led_index()));
        }        
        if (_pGreen)
        {
            bitMask |= (1 << static_cast<uint8_t>(_pGreen->get_led_index()));
        }        
        if (_pBlue)
        {
            bitMask |= (1 << static_cast<uint8_t>(_pBlue->get_led_index()));
        }   
        _pDrv->set_leds_off(bitMask);
    }
    _isOn = false;
}

void ADP8866_RGB::toggle()
{
    _isOn == true ? off() : on();
}

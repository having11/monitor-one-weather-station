#pragma once

#include "Particle.h"

namespace particle {

class IRGB
{
public:
    IRGB(){};
    virtual ~IRGB(){};

    /**
     * @brief Update RGB color
     * @param[in] color : The value of RGB color
     */
    virtual void setColor(uint32_t color) = 0;

    /**
     * @brief Update RGB color
     * @param[in] r : The red value of RGB color
     * @param[in] g : The green value of RGB color
     * @param[in] b : The blue value of RGB color
     */
    virtual void color(uint8_t r,uint8_t g,uint8_t b) = 0;

    /**
     * @brief Get RGB color
     * @return RGB color
     */
    virtual uint32_t color()
    {
        return _color;
    }

    /**
     * @brief Update brightness
     * @param[in] brightness : The value of brightness
     * @param[in] update : Update brightness immediately
     * @return the instance of IRGB Class
     */ 
    virtual void brightness(uint8_t brightness, bool update = true) = 0;

    /**
     * @brief Get brightness
     * @return brightness
     */
    virtual uint8_t brightness()
    {
        return _brightness;
    }

    /**
     * @brief Update LED pattern
     * @param[in] pattern : The value of pattern
     * @return the instance of ILED Class
     */
    virtual void setPattern(LEDPattern pattern) = 0;

    /**
     * @brief Get pattern
     * @return pattern
     */ 
    virtual LEDPattern pattern()
    {
        return _pattern;
    }

    /**
     * @brief Update speed
     * @param[in] speed : The type of speed
     */
    virtual void setSpeed(LEDSpeed speed) = 0;

    /**
     * @brief Update speed
     * @param[in] period : The value of speed in milliseconds
     */
    virtual void setPeriod(uint16_t period) = 0;

    /**
     * @brief Get speed
     * @return period in milliseconds
     */
    virtual uint16_t period()
    {
        return _period;
    }

    /**
     * @brief Set RGB ON
     */
    virtual void on() = 0;

    /**
     * @brief Set RGB OFF
     */
    virtual void off() = 0;

    /**
     * @brief Toggle RGB state
     */
    virtual void toggle() = 0;
    
    /**
     * @brief Check if the RGB at on state.
     * @return true or false
     */
    virtual bool isOn()
    {
        return _isOn == true;
    }

    /**
     * @brief Check if the RGB at off state.
     * @return true or false
     */ 
    virtual bool isOff()
    {
        return _isOn == false;
    }

    /**
     * @brief Get the red value of RGB color
     * @return The red value
     */ 
    uint8_t red(uint32_t color)
    {
        return static_cast<uint8_t>((color >> 16) & 0xFF);
    }

    /**
     * @brief Get the green value of RGB color
     * @return The green value
     */ 
    uint8_t green(uint32_t color)
    {
        return static_cast<uint8_t>((color >> 8) & 0xFF);
    }    

    /**
     * @brief Get the blue value of RGB color
     * @return The blue value
     */ 
    uint8_t blue(uint32_t color)
    {
        return static_cast<uint8_t>(color & 0xFF);
    }
protected:
    LEDPattern _pattern = LED_PATTERN_SOLID;
    uint8_t _brightness = 0xFF;
    LEDSpeed _speed = LED_SPEED_NORMAL;
    uint32_t _period = 0;
    uint32_t _color = 0xFFFFFF;
    bool _isOn = false; 
};

}
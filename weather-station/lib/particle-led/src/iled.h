#pragma once

#include "Particle.h"

namespace particle
{
    class ILED
    {
    public:
        // Predefined pattern periods
        enum class PatternPeriod {
            SOLID_NORMAL = 0,
            BLINK_SLOW = 500,
            BLINK_NORMAL = 200,
            BLINK_FAST = 100,
            FADE_SLOW = 8000,
            FADE_NORMAL = 4000,
            FADE_FAST = 1000
        };
        ILED(){}
        virtual ~ILED(){}

        /**
         * @brief Update brightness
         * @param[in] brightness : The value of brightness
         * @param[in] update : Update brightness immediately
         * @return the instance of ILED Class
         */
        virtual ILED &brightness(uint8_t brightness, bool update = true) = 0;

        /**
         * @brief Get brightness
         * @return brightness
         */        
        virtual uint8_t brightness() = 0;

        /**
         * @brief Update LED pattern
         * @param[in] pattern : The value of pattern
         * @return the instance of ILED Class
         */
        virtual ILED &set_pattern(LEDPattern pattern) = 0;

        /**
         * @brief Get pattern
         * @return pattern
         */          
        virtual LEDPattern pattern() = 0;

        /**
         * @brief Update speed
         * @param[in] speed : The type of speed
         * @return the instance of ILED Class
         */
        virtual ILED &set_speed(LEDSpeed speed) = 0;

        /**
         * @brief Update speed
         * @param[in] period : The value of speed in milliseconds
         * @return the instance of ILED Class
         */        
        virtual ILED &set_period(uint16_t period) = 0;

        /**
         * @brief Get speed
         * @return period in milliseconds
         */                 
        virtual uint16_t period() = 0;

        /**
         * @brief Set LED ON
         * @return the instance of ILED Class
         */  
        virtual ILED &on() = 0;

        /**
         * @brief Set LED OFF
         * @return the instance of ILED Class
         */  
        virtual ILED &off() = 0;

        /**
         * @brief Toggle LED state
         * @return the instance of ILED Class
         */          
        virtual ILED &toggle() = 0;

        /**
         * @brief Check if the led at on state.
         * @return true or false
         */        
        virtual bool isOn()
        {
            return _isOn;            
        }

        /**
         * @brief Check if the led at off state.
         * @return true or false
         */    
        virtual bool isOff()
        {
            return _isOn == false;
        }

        /**
         * @brief Get period by pattern and speed.
         * @param[in] pattern : The type of pattern
         * @param[in] speed : The type of speed
         * @return period in milliseconds
         */           
        uint16_t pattern_period(int pattern, int speed) 
        {
            switch (pattern) 
            {
            case LED_PATTERN_BLINK:
                // Blinking LED
                if (speed == LED_SPEED_NORMAL) 
                {
                    return (uint16_t)PatternPeriod::BLINK_NORMAL;
                } 
                else if (speed > LED_SPEED_NORMAL) 
                {
                    return (uint16_t)PatternPeriod::BLINK_FAST;
                } 
                else 
                {
                    return (uint16_t)PatternPeriod::BLINK_SLOW;
                }
            case LED_PATTERN_FADE:
                // Breathing LED
                if (speed == LED_SPEED_NORMAL) 
                {
                    return (uint16_t)PatternPeriod::FADE_NORMAL;
                } 
                else if (speed > LED_SPEED_NORMAL) 
                {
                    return (uint16_t)PatternPeriod::FADE_FAST;
                } 
                else 
                {
                    return (uint16_t)PatternPeriod::FADE_SLOW;
                }
            default:
                return 0; // Not applicable
            }
        }
    protected:
        LEDPattern _pattern = LED_PATTERN_SOLID;
        uint8_t _brightness = 0xFF;
        LEDSpeed _speed;
        bool _isOn = false;
    };

}
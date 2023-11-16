/*
 * Copyright (c) 2022 Particle Industries, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#pragma once

#include "iled.h"
#include "adp8866.h"

#define OFF_TIME_DISABLE                                (0)             // off time disable
#define OFF_TIME_0_6_S                                  (6)             // 0.6s off
#define OFF_TIME_1_2_S                                  (12)            // 1.2s off
#define OFF_TIME_1_8_S                                  (18)            // 1.8s off
#define OFF_TIME_DEFAULT                                (OFF_TIME_0_6_S)

namespace particle
{
    class ADP8866_LED : public ILED
    {
    public:
        ADP8866_LED(ADP8866 &drv, IscLed led);
        ~ADP8866_LED();

        /**
         * @brief Update brightness
         * @param[in] brightness : The value of brightness
         * @param[in] update : Update brightness immediately
         * @return the instance of ADP8866_LED Class
         */ 
        ADP8866_LED &brightness(uint8_t brightness, bool update = true);

        /**
         * @brief Get brightness
         * @return brightness
         */    
        uint8_t brightness();

        /**
         * @brief Update LED pattern
         * @param[in] pattern : The value of pattern
         * @return the instance of ADP8866_LED Class
         */ 
        ADP8866_LED &set_pattern(LEDPattern pattern);

        /**
         * @brief Get pattern
         * @return pattern
         */  
        LEDPattern pattern();

        /**
         * @brief Update speed
         * @param[in] speed : The type of speed
         * @return the instance of ADP8866_LED Class
         */ 
        ADP8866_LED &set_speed(LEDSpeed speed);

        /**
         * @brief Update speed
         * @param[in] period : The value of speed in milliseconds
         * @return the instance of ADP8866_LED Class
         */
        ADP8866_LED &set_period(uint16_t period);

        /**
         * @brief Get speed
         * @return period in milliseconds
         */ 
        uint16_t period();

        /**
         * @brief Set LED ON
         * @return the instance of ADP8866_LED Class
         */ 
        ADP8866_LED &on();

        /**
         * @brief Set LED OFF
         * @return the instance of ADP8866_LED Class
         */ 
        ADP8866_LED &off();

        /**
         * @brief Toggle LED state
         * @return the instance of ADP8866_LED Class
         */ 
        ADP8866_LED &toggle();

        /**
         * @brief Get the instance of ADP8866 Class
         * @return the instance
         */
        ADP8866 &get_driver();

        /**
         * @brief Get the channel index of LED
         * @return the channel index
         */
        IscLed get_led_index();
    private:
        /**
         * @brief Update the on time
         * @warning Calling this function to modify the on time will impact other LEDs.
         * @return the instance of ADP8866_LED Class
         */  
        ADP8866_LED &update_on_time(IscOnTime time);

        /**
         * @brief Update the off time
         * @return the instance of ADP8866_LED Class
         */
        ADP8866_LED &update_off_time(uint8_t time);

        /**
         * @brief Update fade-in time
         * @warning When the LED is in the backlight group, calling this function to modify the fade-in time will impact other LEDs.
         * @return the instance of ADP8866_LED Class
         */
        ADP8866_LED &update_fade_in_time(IscFadeTime time);

        /**
         * @brief Update fade-out time
         * @warning When the LED is in the backlight group, calling this function to modify the fade-out time will impact other LEDs.
         * @return the instance of ADP8866_LED Class
         */
        ADP8866_LED &update_fade_out_time(IscFadeTime time);

        ADP8866 &_drv;
        IscLed _led;
        uint16_t _period;
        IscFadeTime _fadeInTime     = IscFadeTime::FADE_1_7_5_S;
        IscFadeTime _fadeOutTime    = IscFadeTime::FADE_1_7_5_S;
        IscOnTime _onTime           = IscOnTime::SC_ON_0_7_5_S;
#if (OFF_TIME_DEFAULT == OFF_TIME_DISABLE)
        uint8_t _offTime            = static_cast<uint8_t>(IscOffTime::SC_OFF_DISABLE);
#elif (OFF_TIME_DEFAULT == OFF_TIME_0_6_S)
        uint8_t _offTime            = static_cast<uint8_t>(IscOffTime::SC_OFF_0_6_S);
#elif (OFF_TIME_DEFAULT == OFF_TIME_1_2_S)
        uint8_t _offTime            = static_cast<uint8_t>(IscOffTime::SC_OFF_1_2_S);
#else    
        uint8_t _offTime            = static_cast<uint8_t>(IscOffTime::SC_OFF_1_8_S);
#endif        
    };

}
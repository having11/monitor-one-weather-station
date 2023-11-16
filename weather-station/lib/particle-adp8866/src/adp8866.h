/** 
 * @file adp8866.h
 * @author Jack Lan (DS Team, Particle)
 * @version 0.0.1
 * @date 06/30/2022
 *
 * @brief adp8866 library for Particle projects
 *
 * @copyright Copyright (c) 2022 Particle Industries, Inc.  All rights reserved.
 */
#pragma once

#include "Particle.h"
#include "adp8866_regs.h"
#include "adp8866_def.h"

namespace particle {
class ADP8866
{
public:
    ADP8866(TwoWire &wire = Wire, pin_t rstPin = PIN_INVALID, pin_t intPin = PIN_INVALID);

    /**
     * @brief Initialize ADP8866
     * @return the instance of ADP8866 Class
     */    
    ADP8866 &begin();

    /**
     * @brief Reset ADP8866
     * @return the instance of ADP8866 Class
     */
    ADP8866 &reset();

    /**
     * @brief Set ADP8866 enter standby
     * @param[in] enable : Enable ADP8866 standby mode
     * @return the instance of ADP8866 Class
     */
    ADP8866 &enter_standby(bool enable);

    /**
     * @brief Read register data
     * @param[in] reg : register address
     * @param[in] val : the data that read from register
     * @param[in] length : the length of data
     * @return read result, success or not
     */
    int read_register(uint8_t reg, uint8_t *val, size_t length = 1);

    /**
     * @brief Write register data
     * @param[in] reg : register address
     * @param[in] val : the data that will be written to register
     * @param[in] length : the length of data
     * @return write result, success or not
     */    
    int write_register(uint8_t reg, const uint8_t *val, size_t length = 1);

    /**
     * @brief Get the initial state
     * @return initial state, done or not
     */
    bool is_inited(void);

    /**
     * @brief Get the manufacture and device Id
     * @return the Manufacture and Device Id
     */
    adp8866_mfdvid_t get_mfdvid(void);

    /**
     * @brief Get interrupt status
     * @return interrupt status
     */
    adp8866_int_stat_t get_interrupt_status();

    /**
     * @brief Update the output level selection
     * @param[in] value : The code of maximum current range
     * @return the instance of ADP8866 Class
     */
    ADP8866 &update_output_level(uint8_t value);

    /**
     * @brief The nine LED channels can be separated into two groups: backlight(BL) and independent sinks (ISC)
     * @param[in] led : The LED channel
     * @param[in] backlightGroup : The backlight(BL) or independent sinks (ISC) Group
     * @return the instance of ADP8866 Class
     */
    ADP8866 &leds_backlight_select(IscLed led,bool backlightGroup = true);

    /**
     * @brief The LED Power Source Selection
     * @param[in] led : The LED channel
     * @param[in] usedChargePump : The powered from the charge pump or the battery/other power source
     * @return the instance of ADP8866 Class
     */
    ADP8866 &select_led_power_source(IscLed led, bool usedChargePump = true);

    /**
     * @brief Master enable for backlight sinks
     * @param[in] enable :  Enables all LED current sinks designated as backlight
     * @return the instance of ADP8866 Class
     */   
    ADP8866 &set_backlight(bool enable);
    
    /**
     * @brief Update the Backlight maximum current
     * @param[in] value : The value of maximum current
     * @return the instance of ADP8866 Class
     */    
    ADP8866 &set_backlight_current(uint8_t value);

    /**
     * @brief Update the Backlight fade-in rate
     * @param[in] value : The backlight fades from 0 to its programmed value when the backlight is turned on
     * @return the instance of ADP8866 Class
     */    
    ADP8866 &set_backlight_fade_in_time(IscFadeTime time);

    /**
     * @brief Update the Backlight fade-out rate
     * @param[in] value : The backlight fades from its current value to the off value. 
     * @return the instance of ADP8866 Class
     */  
    ADP8866 &set_backlight_fade_out_time(IscFadeTime time);

    /**
     * @brief Update the independent sink (ISC) current
     * @param[in] led : The LED channel
     * @param[in] value : The DAC Code
     * @return the instance of ADP8866 Class
     */  
    ADP8866 &set_led_sink_current(IscLed led, uint8_t value);

    /**
     * @brief Turn on/off the led Independent Sink Current
     * @param[in] led : The LED channel
     * @param[in] on : on/off State
     * @return the instance of ADP8866 Class
     */      
    ADP8866 &set_led_on_off(IscLed led, bool on);

    /**
     * @brief Turn on the multi leds Independent Sink Current at the same time
     * @param[in] led : The leds bit masks 
     * @return the instance of ADP8866 Class
     */    
    ADP8866 &set_leds_on(uint16_t ledsBitMask);

    /**
     * @brief Turn off the multi leds Independent Sink Current at the same time
     * @param[in] led : The leds bit masks 
     * @return the instance of ADP8866 Class
     */ 
    ADP8866 &set_leds_off(uint16_t ledsBitMask);

    /**
     * @brief Update the on time. 
     *        If the SCxOFF time is not disabled, then when the independent current sink is enabled, it remains on for the on time selected and then turns off.
     * @param[in] time : The time to keep the LED at ON state
     * @return the instance of ADP8866 Class
     */
    ADP8866 &set_leds_on_time(IscOnTime time);

    /**
     * @brief Update the off time. 
     *        When the SCx off time is disabled, the SC remains on while enabled. When the SC off time is set to any other value, the ISC turns off for the off time and then turns on according to the SCON setting.
     * @param[in] led : The LED channel
     * @param[in] time : The time to keep the LED at OFF state
     * @return the instance of ADP8866 Class
     */
    ADP8866 &set_led_off_time(IscLed led, uint8_t time);

    /**
     * @brief Update Sink current fade-in time
     * @param[in] value : The fade-in times
     * @return the instance of ADP8866 Class
     */
    ADP8866 &set_leds_fade_in_time(IscFadeTime time);
    
    /**
     * @brief Update Sink current fade-out time
     * @param[in] value : The fade-out times
     * @return the instance of ADP8866 Class
     */
    ADP8866 &set_leds_fade_out_time(IscFadeTime time);
private:
    /**
     * @brief Read the manufacture and device Id from ADP8866
     * @return read result, success or not
     */
    int read_mfdvid(void);
    TwoWire &_wire;
    adp8866_mfdvid_t _Id;
    pin_t _rstPin = PIN_INVALID;
    pin_t _intPin = PIN_INVALID;
    bool _inited  = false;
};
}
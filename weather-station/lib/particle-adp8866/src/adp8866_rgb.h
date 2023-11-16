#pragma once

#include "irgb.h"
#include "adp8866_led.h"


namespace particle
{

class ADP8866_RGB : public IRGB
{
public:
    ADP8866_RGB(IscLed led_r,IscLed led_g,IscLed led_b,ADP8866 *pDrv);
    ADP8866_RGB(ADP8866_LED *pRed,ADP8866_LED *pGreen,ADP8866_LED *pBlue);
    ~ADP8866_RGB();
    ADP8866_RGB& operator = (const ADP8866_RGB&) = delete;
    ADP8866_RGB(const ADP8866_RGB&) = delete;

    /**
     * @brief Update RGB color
     * @param[in] color : The value of RGB color
     */
    void setColor(uint32_t color);

    /**
     * @brief Update RGB color
     * @param[in] r : The red value of RGB color
     * @param[in] g : The green value of RGB color
     * @param[in] b : The blue value of RGB color
     */    
    void color(uint8_t r,uint8_t g,uint8_t b);

    /**
     * @brief Update brightness
     * @param[in] brightness : The value of brightness
     * @param[in] update : Update brightness immediately
     * @return the instance of IRGB Class
     */ 
    void brightness(uint8_t brightness, bool update = true);

    /**
     * @brief Update LED pattern
     * @param[in] pattern : The value of pattern
     * @return the instance of ILED Class
     */    
    void setPattern(LEDPattern pattern);

    /**
     * @brief Update speed
     * @param[in] speed : The type of speed
     */
    void setSpeed(LEDSpeed speed);

    /**
     * @brief Update speed
     * @param[in] period : The value of speed in milliseconds
     */    
    void setPeriod(uint16_t period);

    /**
     * @brief Set RGB ON
     */    
    void on();

    /**
     * @brief Set RGB OFF
     */    
    void off();

    /**
     * @brief Toggle RGB state
     */    
    void toggle();
private:
    bool _cleanupMemory = false;
    ADP8866_LED *_pRed = nullptr;
    ADP8866_LED *_pGreen = nullptr;
    ADP8866_LED *_pBlue = nullptr;
    IscLed _led_r = IscLed::INVALID;
    IscLed _led_g = IscLed::INVALID;
    IscLed _led_b = IscLed::INVALID;
    ADP8866 *_pDrv;
};

}
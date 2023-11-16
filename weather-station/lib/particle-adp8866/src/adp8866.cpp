#include "adp8866.h"
#include "check.h"

using namespace spark;
using namespace particle;

RecursiveMutex _mutex;
#define RAII_LOCK()   const std::lock_guard<RecursiveMutex> locker(_mutex)

ADP8866::ADP8866(TwoWire& wire,pin_t rstPin,pin_t intPin):
    _wire(wire),
    _rstPin(rstPin),
    _intPin(intPin)
{
    if(_rstPin != PIN_INVALID)
    {
        pinMode(_rstPin, OUTPUT);
        reset();
    }
    if(_intPin != PIN_INVALID)
    {
        pinMode(_intPin, INPUT);
    }
    begin();    
}

ADP8866 &ADP8866::reset()
{
    RAII_LOCK(); 
    if(_rstPin != PIN_INVALID)
    {
        digitalWriteFast(_rstPin, LOW);
        delay(100);
        digitalWriteFast(_rstPin, HIGH);
    }
    return *this;
}

ADP8866 &ADP8866::begin()
{
    RAII_LOCK(); 
    // Initialize the I2C bus if not already enabled
    if (!_wire.isEnabled()) 
    {
        _wire.begin();
    }    
    // Mode Control
    adp8866_mdcr_t mdcr = 
    {
        .bits = 
        {
            .bl_en = 0,
            .sis_en = 0,
            .gdwn_dis = 0,
            .alt_gsel = 0,
            .nstby = 1,
            .int_cfg = 0
        }
    };
    write_register(MDCR,&mdcr.value,1);

    // Output Level Selection,  control with the LEVEL_SET bits
    adp8866_lvl_sel1_t lvlSel1 = 
    {
        .bits = 
        {
            .level_set = OUTPUT_LEVEL_DEFAULT,
            .d9lvl = 1,
        }
    };
    adp8866_lvl_sel2_t lvlSel2 = 
    {
        .value = 0xFF
    };
    write_register(LVL_SEL1,&lvlSel1.value,1);
    write_register(LVL_SEL2,&lvlSel2.value,1);

    // LED Power Source Selection
    adp8866_pwr_sel1_t pwrSel1 = 
    {
        .value = 0
    };
    adp8866_pwr_sel2_t pwrSel2 = 
    {
        .value = 0
    };
    write_register(PWR_SEL1,&pwrSel1.value,1);
    write_register(PWR_SEL2,&pwrSel2.value,1);    

    // Backlight Configuration
    adp8866_cfgr_t cfgr = 
    {
        .bits = 
        {
            .bl_law = 0x00,
            .cabcfade = 1,
            .d9sel = 1
        }
    };
    write_register(CFGR,&cfgr.value,1);
    // Backlight Select
    adp8866_blsel_t blsel = 
    {
        .value = 0xFF
    };
    write_register(BLSEL,&blsel.value,1);


    adp8866_iscc1_t iscc = 
    {
        .bits = 
        {
            .sc_law = 3,    //  square law DAC, nonlinear time steps (Cubic 11).
            .sc9_en = 0,
        }
    };
    write_register(ISCC1,&iscc.value);    
    read_mfdvid();
    _inited = true;
    return *this;
}


ADP8866 &ADP8866::update_output_level(uint8_t value)
{
    RAII_LOCK();
    adp8866_lvl_sel1_t lvlSel1;
    read_register(LVL_SEL1,&lvlSel1.value); 
    lvlSel1.bits.level_set = value;
    write_register(LVL_SEL1,&lvlSel1.value); 
    return *this;  
}

ADP8866 &ADP8866::enter_standby(bool enable)
{
    RAII_LOCK();
    adp8866_mdcr_t mdcr = 
    {
        .value = 0
    };
    read_register(MDCR,&mdcr.value);
    mdcr.bits.nstby = enable ? 0 : 1;
    write_register(MDCR,&mdcr.value);    
    return *this;
}


bool ADP8866::is_inited(void)
{
    return _inited;
}

adp8866_mfdvid_t ADP8866::get_mfdvid(void)
{
    return _Id;
}

int ADP8866::read_mfdvid(void)
{
    RAII_LOCK();
    read_register(MFDVID,(uint8_t*)&_Id.value,1);
    //Log.info("%s ==> MFID == %d, DEV_ID == %d ",__FUNCTION__,_Id.mf,_Id.dev);
    return SYSTEM_ERROR_NONE;
}


adp8866_int_stat_t ADP8866::get_interrupt_status()
{
    RAII_LOCK();
    adp8866_int_stat_t status = {0};
    read_register(INT_STAT,(uint8_t*)&status.value,1);
    //Log.info("%s: 0x%02X\r\nISCOFF_INT == %d\r\nBLOFF_INT == %d\r\nSHORT_INT == %d\r\nTSD_INT == %d\r\nOVP_INT == %d",__FUNCTION__,status.value,status.bits.iscoff_int,status.bits.bloff_int,status.bits.short_int,status.bits.tsd_int,status.bits.ovp_int);
    return status;
}


int ADP8866::read_register(uint8_t reg, uint8_t* val, size_t length) 
{
	uint8_t regAddress[1] { reg };
    WITH_LOCK(_wire)
    {
        _wire.beginTransmission(Adp8866Address);
        _wire.write(regAddress, 1);
        CHECK_TRUE(_wire.endTransmission(false) == 0, SYSTEM_ERROR_INTERNAL);
        auto remaining = std::min<int>(length, I2C_BUFFER_LENGTH);
        auto readLength = (int)_wire.requestFrom((int)Adp8866Address, remaining);
        if (readLength != remaining) 
        {
            _wire.endTransmission();
            return SYSTEM_ERROR_INTERNAL;
        }
        while (_wire.available() && remaining--) 
        {
            *val++ = _wire.read();
        }
    }
	return SYSTEM_ERROR_NONE;
}

int ADP8866::write_register(uint8_t reg, const uint8_t* val, size_t length) 
{
    int ret = SYSTEM_ERROR_NONE;
    size_t size = length + 1;
    uint8_t* buf = new uint8_t[size];
	buf[0] = reg;
    for (size_t i = 1; i < size; i++)
    {
        buf[i] = *val++;
    }   
    //Log.info("%s reg == 0x%02X , 0x%02X",__FUNCTION__,buf[0],buf[1]);
    WITH_LOCK(_wire)
    {
        _wire.beginTransmission(Adp8866Address);
        _wire.write(buf, size);
	    ret = _wire.endTransmission();
    }
    delete buf;
    return ret;
}

ADP8866 &ADP8866::set_backlight(bool enable)
{
    RAII_LOCK();
    adp8866_mdcr_t mdcr = 
    {
        .value = 0
    };
    read_register(MDCR,&mdcr.value);
    if (mdcr.bits.bl_en == 0)
    {
        mdcr.bits.bl_en = enable == true ? 1 : 0;
        write_register(MDCR,&mdcr.value);
    }    
    return *this;
}

ADP8866 &ADP8866::set_backlight_current(uint8_t value)
{
    RAII_LOCK();
    adp8866_blmx_t blmx = 
    {
        .value = value
    };
    write_register(BLMX,&blmx.value);   
    return *this;
}

ADP8866 &ADP8866::set_backlight_fade_in_time(IscFadeTime time)
{
    RAII_LOCK();
    adp8866_blfr_t blfr =
    {
        .value = 0
    };
    read_register(BLFR, &blfr.value);
    blfr.bits.bl_fi = static_cast<uint8_t>(time);
    write_register(BLFR, &blfr.value);     
    return *this;
}

ADP8866 &ADP8866::set_backlight_fade_out_time(IscFadeTime time)
{
    RAII_LOCK();
    adp8866_blfr_t blfr =
    {
        .value = 0
    };
    read_register(BLFR, &blfr.value);
    blfr.bits.bl_fo = static_cast<uint8_t>(time);
    write_register(BLFR, &blfr.value);   
    return *this;    
}


ADP8866 &ADP8866::leds_backlight_select(IscLed led,bool backlightGroup)
{
    RAII_LOCK();
    int index = static_cast<int>(led);
    if (led == IscLed::LED9)
    {
        adp8866_cfgr_t cfgr =
        {
            .value = 0
        };
        read_register(CFGR, &cfgr.value);
        cfgr.bits.d9sel = backlightGroup == false ? 1 : 0;
        write_register(CFGR, &cfgr.value);
    }
    else
    {
        adp8866_blsel_t blsel =
        {
            .value = 0
        };
        read_register(BLSEL, &blsel.value);
        if(backlightGroup == false)
        {
            blsel.value |= (1 << index);
        }
        else
        {
            blsel.value &= ~(1 << index);
        }
        write_register(BLSEL, &blsel.value);
    }
    return *this;
}


ADP8866 &ADP8866::select_led_power_source(IscLed led,bool usedChargePump)
{
    RAII_LOCK();
    int index = static_cast<int>(led);
    if (led == IscLed::LED9)
    {
        adp8866_pwr_sel1_t pwrSel =
        {
            .value = 0
        };
        pwrSel.bits.d9pwr = usedChargePump == false ? 1 : 0;
        write_register(PWR_SEL1, &pwrSel.value);
    }
    else
    {
        adp8866_pwr_sel2_t pwrSel =
        {
            .value = 0
        };
        read_register(PWR_SEL2, &pwrSel.value);
        if(usedChargePump == false)
        {
            pwrSel.value |= (1 << index);
        }
        else
        {
            pwrSel.value &= ~(1 << index);
        }
        write_register(PWR_SEL2, &pwrSel.value);
    }
    return *this;
}


ADP8866 &ADP8866::set_led_sink_current(IscLed led,uint8_t value)
{
    RAII_LOCK();
    int index = static_cast<int>(led);
    uint regAddr = ISC1 + index;
    write_register(regAddr, &value);    
    return *this;
}

ADP8866 &ADP8866::set_led_on_off(IscLed led,bool on)
{
    RAII_LOCK();
    int index = static_cast<int>(led);
    uint8_t value = 0;
    uint regAddr = ISCC1;
    if (led == IscLed::LED9)
    {
        adp8866_iscc1_t iscc =
        {
            .value = 0
        };
        read_register(ISCC1, &iscc.value);
        iscc.bits.sc9_en = on == true ? 1 : 0;
        value = iscc.value;
    }
    else
    {
        adp8866_iscc2_t iscc =
        {
            .value = 0
        };
        read_register(ISCC2, &iscc.value);
        if(on)
        {
            iscc.value |= (1 << index);
        }
        else
        {
            iscc.value &= ~(1 << index);
        }
        value = iscc.value;
        regAddr = ISCC2;
    }    
    write_register(regAddr, &value);
    return *this;
}


ADP8866 &ADP8866::set_leds_on(uint16_t ledsBitMask)
{
    RAII_LOCK();
    if ((ledsBitMask & 0x100) != 0x0)
    {
        adp8866_iscc1_t iscc =
        {
            .value = 0
        };
        read_register(ISCC1, &iscc.value);
        iscc.bits.sc9_en = 1;
        write_register(ISCC1, &iscc.value);
    }
   
    adp8866_iscc2_t iscc =
    {
        .value = 0
    };
    read_register(ISCC2, &iscc.value);
    iscc.value |= (ledsBitMask & 0xFF);
    write_register(ISCC2, &iscc.value);    
    return *this;
}


ADP8866 &ADP8866::set_leds_off(uint16_t ledsBitMask)
{
    RAII_LOCK();
    if ((ledsBitMask & 0x100) != 0x0)
    {
        adp8866_iscc1_t iscc =
        {
            .value = 0
        };
        read_register(ISCC1, &iscc.value);
        iscc.bits.sc9_en = 0;
        write_register(ISCC1, &iscc.value);
    }
    adp8866_iscc2_t iscc =
    {
        .value = 0
    };
    read_register(ISCC2, &iscc.value);
    iscc.value &= ~(ledsBitMask & 0xFF);
    write_register(ISCC2, &iscc.value);   
    return *this;
}


ADP8866 &ADP8866::set_leds_on_time(IscOnTime time)
{
    RAII_LOCK();
    adp8866_isct1_t isct =
    {
        .value = 0
    };
    read_register(ISCT1, &isct.value);
    isct.bits.scon = static_cast<uint8_t>(time);
    write_register(ISCT1, &isct.value);    
    return *this;    
}

ADP8866 &ADP8866::set_leds_fade_in_time(IscFadeTime time)
{
    RAII_LOCK();
    adp8866_iscf_t iscf =
    {
        .value = 0
    };
    read_register(ISCF, &iscf.value);
    iscf.bits.scfi = static_cast<uint8_t>(time);
    write_register(ISCF, &iscf.value);     
    return *this;    
}

ADP8866 &ADP8866::set_leds_fade_out_time(IscFadeTime time)
{
    RAII_LOCK();
    adp8866_iscf_t iscf =
    {
        .value = 0
    };
    read_register(ISCF, &iscf.value);
    iscf.bits.scfo = static_cast<uint8_t>(time);
    write_register(ISCF, &iscf.value);   
    return *this;    
}


ADP8866 &ADP8866::set_led_off_time(IscLed led, uint8_t time)
{
    RAII_LOCK();
    uint8_t regAddr = 0xFF;
    uint8_t value = 0;
    do
    {           
        if (led < IscLed::LED6)
        {
            if (time <= static_cast<uint8_t>(IscOffTime::SC_OFF_1_8_S))
            {
                if (led == IscLed::LED5)
                {
                    adp8866_isct1_t isct =
                    {
                        .value = 0
                    };
                    read_register(ISCT1, &isct.value);  
                    isct.bits.sc5off = time;
                    regAddr = ISCT1;   
                    value = isct.value;
                }
                else
                {
                    uint8_t offeset = static_cast<uint8_t>(led) * 2;
                    adp8866_isct2_t isct =
                    {
                        .value = 0
                    };
                    read_register(ISCT2, &isct.value);
                    isct.value &= ~(0x03 << offeset);
                    isct.value |= ((time & 0x03) << offeset);
                    value = isct.value;                    
                    regAddr = ISCT2;
                }
            }     
            else
            {
                break;
            }   
        }
        else
        {
            if (time <= static_cast<uint8_t>(IscOffTime::SC_OFF_ADV_OFF))
            {
                regAddr = OFFTIMER6 + (static_cast<uint8_t>(led) - static_cast<uint8_t>(IscLed::LED6));
                value = time;
            }
            else
            {
                break;
            }
        }  
        write_register(regAddr, &value);      
    } while (0);
    return *this;
}


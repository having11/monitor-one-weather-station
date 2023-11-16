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

#define SINK_CURRENT_MAX                    (0x7F)
#define BACKLIGHT_CURRENT_MAX               (0x7F)
#define OUTPUT_LEVEL_DEFAULT                (0x02)      //  25 mA ÷ N = 25.0 mA
#define OUTPUT_LEVEL_MAX                    (0x00)      //  25 mA ÷ N = 31.3 mA
#define OUTPUT_LEVEL_MIN                    (0x3E)      //  25 mA ÷ N = 3.6 mA

#define LED1_BIT_MASK                       (1 << 0)
#define LED2_BIT_MASK                       (1 << 1)
#define LED3_BIT_MASK                       (1 << 2)
#define LED4_BIT_MASK                       (1 << 3)
#define LED5_BIT_MASK                       (1 << 4)
#define LED6_BIT_MASK                       (1 << 5)
#define LED7_BIT_MASK                       (1 << 6)
#define LED8_BIT_MASK                       (1 << 7)
#define LED9_BIT_MASK                       (1 << 8)
#define LED_ALL_BIT_MASK                    (0x1FF)

namespace particle {

enum class IscLed {
    LED1 = 0,
    LED2,
    LED3,
    LED4,
    LED5,
    LED6,
    LED7,
    LED8,
    LED9,
    INVALID,
};



enum class IscOnTime {
    SC_ON_0_0_0_S = 0,          // 0.00 sec
    SC_ON_0_0_5_S,              // 0.05 sec
    SC_ON_0_1_0_S,              // 0.10 sec
    SC_ON_0_1_5_S,              // 0.15 sec
    SC_ON_0_2_0_S,              // 0.20 sec
    SC_ON_0_2_5_S,              // 0.25 sec
    SC_ON_0_3_0_S,              // 0.30 sec
    SC_ON_0_3_5_S,              // 0.35 sec
    SC_ON_0_4_0_S,              // 0.40 sec
    SC_ON_0_4_5_S,              // 0.45 sec
    SC_ON_0_5_0_S,              // 0.50 sec
    SC_ON_0_5_5_S,              // 0.55 sec
    SC_ON_0_6_0_S,              // 0.60 sec
    SC_ON_0_6_5_S,              // 0.65 sec
    SC_ON_0_7_0_S,              // 0.70 sec
    SC_ON_0_7_5_S,              // 0.75 sec
};


enum class IscOffTime {
    /*
        LED1 ~ LED5
    */
    SC_OFF_DISABLE = 0,         // off time disabled
    SC_OFF_0_6_S,               // 0.6 sec
    SC_OFF_1_2_S,               // 1.2 sec
    SC_OFF_1_8_S,               // 1.8 sec
    /*
        LED6 ~ LED9
    */
    SC_OFF_ADV_DISABLE = 0,     // off time disabled
    SC_OFF_ADV_0_0_0_S,         // 0 sec
    SC_OFF_ADV_0_0_1_S,         // 0.1 sec
    // more date ...
    SC_OFF_ADV_1_2_5_S = 0x7E,  // 12.5 sec 
    /*
        Setting SCxOFF to off causes the LED to be held off indefinitely. 
        This is useful for setting up a blink sequence that runs once and then goes to off.
    */
    SC_OFF_ADV_OFF = 0x7F,  // off time off 
};

enum class IscFadeTime {
    FADE_DISABLE = 0,           // disabled
    FADE_0_0_5_S,              // 0.05 sec
    FADE_0_1_0_S,              // 0.10 sec
    FADE_0_1_5_S,              // 0.15 sec
    FADE_0_2_0_S,              // 0.20 sec
    FADE_0_2_5_S,              // 0.25 sec
    FADE_0_3_0_S,              // 0.30 sec
    FADE_0_3_5_S,              // 0.35 sec
    FADE_0_4_0_S,              // 0.40 sec
    FADE_0_4_5_S,              // 0.45 sec
    FADE_0_5_0_S,              // 0.50 sec
    FADE_0_7_5_S,              // 0.75 sec
    FADE_1_0_0_S,              // 1.00 sec
    FADE_1_2_5_S,              // 1.25 sec
    FADE_1_5_0_S,              // 1.50 sec
    FADE_1_7_5_S,              // 1.75 sec
};




typedef union 
{
    uint8_t value;
    struct 
    {
        uint8_t dev:4;
        uint8_t mf:4;
    };    
}adp8866_mfdvid_t;
typedef union 
{
    uint8_t value;
    struct
    {
        uint8_t bl_en:1;
        uint8_t :1;
        uint8_t sis_en:1;
        uint8_t gdwn_dis:1;
        uint8_t alt_gsel:1;
        uint8_t nstby:1;
        uint8_t int_cfg:1;
        uint8_t :1;
    }bits;    
}adp8866_mdcr_t;

typedef union 
{
    uint8_t value;
    struct
    {
        uint8_t :2;
        uint8_t ovp_int:1;
        uint8_t tsd_int:1;
        uint8_t short_int:1;
        uint8_t bloff_int:1;
        uint8_t iscoff_int:1;
        uint8_t :1;
    }bits;    
}adp8866_int_stat_t;
typedef union 
{
    uint8_t value;
    struct
    {
        uint8_t :2;
        uint8_t ovp_ien:1;
        uint8_t tsd_ien:1;
        uint8_t short_ien:1;
        uint8_t bloff_ien:1;
        uint8_t iscoff_ien:1;
        uint8_t :1;
    }bits;    
}adp8866_ien_en_t;

typedef union 
{
    uint8_t value;
    struct
    {
        uint8_t d9offint:1;
        uint8_t :7;
    }bits;    
}adp8866_iscoff_sel1_t;
typedef union 
{
    uint8_t value;
    struct
    {
        uint8_t d1offint:1;
        uint8_t d2offint:1;
        uint8_t d3offint:1;
        uint8_t d4offint:1;
        uint8_t d5offint:1;
        uint8_t d6offint:1;
        uint8_t d7offint:1;
        uint8_t d8offint:1;
    }bits;    
}adp8866_iscoff_sel2_t;

typedef union 
{
    uint8_t value;
    struct
    {
        uint8_t g_force:2;
        uint8_t _1_5X_LIMIT:1;
        uint8_t :5;
    }bits;    
}adp8866_gain_sel_t;

typedef union 
{
    uint8_t value;
    struct
    {
        uint8_t level_set:6;
        uint8_t d9lvl:1;
        uint8_t :1;
    }bits;    
}adp8866_lvl_sel1_t;

typedef union 
{
    uint8_t value;
    struct
    {
        uint8_t d1lvl:1;
        uint8_t d2lvl:1;
        uint8_t d3lvl:1;
        uint8_t d4lvl:1;
        uint8_t d5lvl:1;
        uint8_t d6lvl:1;                
        uint8_t d7lvl:1;
        uint8_t d8lvl:1;
    }bits;    
}adp8866_lvl_sel2_t;

typedef union 
{
    uint8_t value;
    struct
    {
        uint8_t d9pwr:1;
        uint8_t :7;
    }bits;    
}adp8866_pwr_sel1_t;

typedef union 
{
    uint8_t value;
    struct
    {
        uint8_t d1pwr:1;
        uint8_t d2pwr:1;
        uint8_t d3pwr:1;
        uint8_t d4pwr:1;
        uint8_t d5pwr:1;
        uint8_t d6pwr:1;                
        uint8_t d7pwr:1;
        uint8_t d8pwr:1;
    }bits;    
}adp8866_pwr_sel2_t;

typedef union 
{
    uint8_t value;
    struct
    {
        uint8_t :1;
        uint8_t bl_law:2;
        uint8_t cabcfade:1;                
        uint8_t d9sel:1;
        uint8_t :3;
    }bits;    
}adp8866_cfgr_t;

typedef union 
{
    uint8_t value;
    struct
    {
        uint8_t d1sel:1;
        uint8_t d2sel:1;
        uint8_t d3sel:1;
        uint8_t d4sel:1;
        uint8_t d5sel:1;
        uint8_t d6sel:1;                
        uint8_t d7sel:1;
        uint8_t d8sel:1;
    }bits;    
}adp8866_blsel_t;
typedef union 
{
    uint8_t value;
    struct
    {
        uint8_t bl_fi:4;
        uint8_t bl_fo:4;
    }bits;    
}adp8866_blfr_t;

typedef union 
{
    uint8_t value;
    struct
    {
        uint8_t bl_mc:7;
        uint8_t :1;
    }bits;    
}adp8866_blmx_t;

typedef union 
{
    uint8_t value;
    struct
    {
        uint8_t sc_law:2;
        uint8_t sc9_en:1;
        uint8_t :5;
    }bits;    
}adp8866_iscc1_t;

typedef union 
{
    uint8_t value;
    struct
    {
        uint8_t sc1_en:1;
        uint8_t sc2_en:1;
        uint8_t sc3_en:1;
        uint8_t sc4_en:1;
        uint8_t sc5_en:1;
        uint8_t sc6_en:1;
        uint8_t sc7_en:1;
        uint8_t sc8_en:1;
    }bits;    
}adp8866_iscc2_t;

typedef union 
{
    uint8_t value;
    struct
    {
        uint8_t sc5off:2;
        uint8_t :2;
        uint8_t scon:4;
    }bits;    
}adp8866_isct1_t;

typedef union 
{
    uint8_t value;
    struct
    {
        uint8_t sc1off:2;
        uint8_t sc2off:2;
        uint8_t sc3off:2;
        uint8_t sc4off:2;
    }bits;    
}adp8866_isct2_t;

typedef union 
{
    uint8_t value;
    struct
    {
        uint8_t sc6off:7;
        uint8_t :1;
    }bits;    
}adp8866_offtimer6_t;

typedef union 
{
    uint8_t value;
    struct
    {
        uint8_t sc7off:7;
        uint8_t :1;
    }bits;    
}adp8866_offtimer7_t;

typedef union 
{
    uint8_t value;
    struct
    {
        uint8_t sc8off:7;
        uint8_t :1;
    }bits;    
}adp8866_offtimer8_t;

typedef union 
{
    uint8_t value;
    struct
    {
        uint8_t sc9off:7;
        uint8_t :1;
    }bits;    
}adp8866_offtimer9_t;


typedef union 
{
    uint8_t value;
    struct
    {
        uint8_t scfi:4;
        uint8_t scfo:4;
    }bits;    
}adp8866_iscf_t;


typedef union 
{
    uint8_t value;
    struct
    {
        uint8_t scd1:7;
        uint8_t :1;
    }bits;    
}adp8866_isc1_t;

typedef union 
{
    uint8_t value;
    struct
    {
        uint8_t scd2:7;
        uint8_t :1;
    }bits;    
}adp8866_isc2_t;

typedef union 
{
    uint8_t value;
    struct
    {
        uint8_t scd3:7;
        uint8_t :1;
    }bits;    
}adp8866_isc3_t;

typedef union 
{
    uint8_t value;
    struct
    {
        uint8_t scd4:7;
        uint8_t :1;
    }bits;    
}adp8866_isc4_t;

typedef union 
{
    uint8_t value;
    struct
    {
        uint8_t scd5:7;
        uint8_t :1;
    }bits;    
}adp8866_isc5_t;

typedef union 
{
    uint8_t value;
    struct
    {
        uint8_t scd6:7;
        uint8_t :1;
    }bits;    
}adp8866_isc6_t;

typedef union 
{
    uint8_t value;
    struct
    {
        uint8_t scd7:7;
        uint8_t :1;
    }bits;    
}adp8866_isc7_t;

typedef union 
{
    uint8_t value;
    struct
    {
        uint8_t scd8:7;
        uint8_t :1;
    }bits;    
}adp8866_isc8_t;
typedef union 
{
    uint8_t value;
    struct
    {
        uint8_t scd9:7;
        uint8_t :1;
    }bits;    
}adp8866_isc9_t;


typedef union 
{
    uint8_t value;
    struct
    {
        uint8_t d6hb_en:1;
        uint8_t d7hb_en:1;
        uint8_t d8hb_en:1;
        uint8_t d9hb_en:1;
        uint8_t :4;
    }bits;    
}adp8866_hb_sel_t;

typedef union 
{
    uint8_t value;
    struct
    {
        uint8_t scd6_hb:7;
        uint8_t :1;
    }bits;    
}adp8866_isc6_hb_t;
typedef union 
{
    uint8_t value;
    struct
    {
        uint8_t scd7_hb:7;
        uint8_t :1;
    }bits;    
}adp8866_isc7_hb_t;

typedef union 
{
    uint8_t value;
    struct
    {
        uint8_t scd8_hb:7;
        uint8_t :1;
    }bits;    
}adp8866_isc8_hb_t;

typedef union 
{
    uint8_t value;
    struct
    {
        uint8_t scd9_hb:7;
        uint8_t :1;
    }bits;    
}adp8866_isc9_hb_t;

typedef union 
{
    uint8_t value;
    struct
    {
        uint8_t sc6off_hb:7;
        uint8_t :1;
    }bits;    
}adp8866_offtimer6_hb_t;

typedef union 
{
    uint8_t value;
    struct
    {
        uint8_t sc7off_hb:7;
        uint8_t :1;
    }bits;    
}adp8866_offtimer7_hb_t;

typedef union 
{
    uint8_t value;
    struct
    {
        uint8_t sc8off_hb:7;
        uint8_t :1;
    }bits;    
}adp8866_offtimer8_hb_t;

typedef union 
{
    uint8_t value;
    struct
    {
        uint8_t sc9off_hb:7;
        uint8_t :1;
    }bits;    
}adp8866_offtimer9_hb_t;

typedef union 
{
    uint8_t value;
    struct
    {
        uint8_t scon_hb:4;
        uint8_t :4;
    }bits;    
}adp8866_isct_hb_t;

typedef union 
{
    uint8_t value;
    struct
    {
        uint8_t delay6:7;
        uint8_t :1;
    }bits;    
}adp8866_delay6_t;

typedef union 
{
    uint8_t value;
    struct
    {
        uint8_t delay7:7;
        uint8_t :1;
    }bits;    
}adp8866_delay7_t;


typedef union 
{
    uint8_t value;
    struct
    {
        uint8_t delay8:7;
        uint8_t :1;
    }bits;    
}adp8866_delay8_t;


typedef union 
{
    uint8_t value;
    struct
    {
        uint8_t delay9:7;
        uint8_t :1;
    }bits;    
}adp8866_delay9_t;

};
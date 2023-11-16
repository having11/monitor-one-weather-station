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

#include "Particle.h"

namespace particle  {

constexpr uint8_t Adp8866Address = 0x27;

enum Adp8866Register: uint8_t {
    MFDVID                  = 0x00,
    MDCR                    = 0x01,
    INT_STAT                = 0x02,
    INT_EN                  = 0x03,
    ISCOFF_SEL1             = 0x04,
    ISCOFF_SEL2             = 0x05,
    GAIN_SEL                = 0x06,
    LVL_SEL1                = 0x07,
    LVL_SEL2                = 0x08,
    PWR_SEL1                = 0x09,
    PWR_SEL2                = 0x0A,
    CFGR                    = 0x10,
    BLSEL                   = 0x11,
    BLFR                    = 0x12,
    BLMX                    = 0x13,
    ISCC1                   = 0x1A,
    ISCC2                   = 0x1B,
    ISCT1                   = 0x1C,
    ISCT2                   = 0x1D,
    OFFTIMER6               = 0x1E,
    OFFTIMER7               = 0x1F,
    OFFTIMER8               = 0x20,
    OFFTIMER9               = 0x21,
    ISCF                    = 0x22,
    ISC1                    = 0x23,
    ISC2                    = 0x24,
    ISC3                    = 0x25,
    ISC4                    = 0x26,
    ISC5                    = 0x27,
    ISC6                    = 0x28,
    ISC7                    = 0x29,
    ISC8                    = 0x2A,
    ISC9                    = 0x2B,
    HB_SEL                  = 0x2C,
    ISC6_HB                 = 0x2D,
    ISC7_HB                 = 0x2E,
    ISC8_HB                 = 0x2F,
    ISC9_HB                 = 0x30,
    OFFTIMER6_HB            = 0x31,
    OFFTIMER7_HB            = 0x32,
    OFFTIMER8_HB            = 0x33,
    OFFTIMER9_HB            = 0x34,
    ISCT_HB                 = 0x35,
    DELAY6                  = 0x3C,
    DELAY7                  = 0x3D,
    DELAY8                  = 0x3E,
    DELAY9                  = 0x3F,
};

// Manufacturer and Device ID Register
#define ADP8866_MFDVID_MID_SHIFT                (4)
#define ADP8866_MFDVID_MID_MASK                 (0xf << (ADP8866_MFDVID_MID_SHIFT))

#define ADP8866_MFDVID_DID_SHIFT                (0)
#define ADP8866_MFDVID_DID_MASK                 (0xf << (ADP8866_MFDVID_DID_SHIFT))


// Mode Control Register
#define ADP8866_MDCR_INT_CFG_SHIFT              (6)
#define ADP8866_MDCR_INT_CFG_MASK               (0x1 << (ADP8866_MDCR_INT_CFG_SHIFT))

#define ADP8866_MDCR_NSTBY_SHIFT                (5)
#define ADP8866_MDCR_NSTBY_MASK                 (0x1 << (ADP8866_MDCR_NSTBY_SHIFT))

#define ADP8866_MDCR_ALT_GSEL_SHIFT             (4)
#define ADP8866_MDCR_ALT_GSEL_MASK              (0x1 << (ADP8866_MDCR_ALT_GSEL_SHIFT))

#define ADP8866_MDCR_GDWN_DIS_SHIFT             (3)
#define ADP8866_MDCR_GDWN_DIS_MASK              (0x1 << (ADP8866_MDCR_GDWN_DIS_SHIFT))

#define ADP8866_MDCR_SIS_EN_SHIFT               (2)
#define ADP8866_MDCR_SIS_EN_MASK                (0x1 << (ADP8866_MDCR_SIS_EN_SHIFT))

#define ADP8866_MDCR_BL_EN_SHIFT                (0)
#define ADP8866_MDCR_BL_EN_MASK                 (0x1 << (ADP8866_MDCR_BL_EN_SHIFT))


// Interrupt Status and Enable Registers
#define ADP8866_INT_ISCOFF_SHIFT                (6)
#define ADP8866_INT_ISCOFF_MASK                 (0x1 << (ADP8866_INT_ISCOFF_SHIFT))

#define ADP8866_INT_BLOFF_SHIFT                 (5)
#define ADP8866_INT_BLOFF_MASK                  (0x1 << (ADP8866_INT_BLOFF_SHIFT))

#define ADP8866_INT_SHORT_SHIFT                 (4)
#define ADP8866_INT_SHORT_MASK                  (0x1 << (ADP8866_INT_SHORT_SHIFT))

#define ADP8866_INT_TSD_SHIFT                   (3)
#define ADP8866_INT_TSD_MASK                    (0x1 << (ADP8866_INT_TSD_SHIFT))

#define ADP8866_INT_OVP_SHIFT                   (2)
#define ADP8866_INT_OVP_MASK                    (0x1 << (ADP8866_INT_OVP_SHIFT))


// Independent Sink Interrupt Selection 1 Register
#define ADP8866_ISCOFF_SEL1_D9OFFINT_SHIFT      (0)
#define ADP8866_ISCOFF_SEL1_D9OFFINT_MASK       (0x1 << (ADP8866_ISCOFF_SEL1_D9OFFINT_SHIFT))


// Independent Sink Interrupt Selection 2 Register
#define ADP8866_ISCOFF_SEL2_D8OFFINT_SHIFT      (7)
#define ADP8866_ISCOFF_SEL2_D8OFFINT_MASK       (0x1 << (ADP8866_ISCOFF_SEL2_D8OFFINT_SHIFT))

#define ADP8866_ISCOFF_SEL2_D7OFFINT_SHIFT      (6)
#define ADP8866_ISCOFF_SEL2_D7OFFINT_MASK       (0x1 << (ADP8866_ISCOFF_SEL2_D7OFFINT_SHIFT))

#define ADP8866_ISCOFF_SEL2_D6OFFINT_SHIFT      (5)
#define ADP8866_ISCOFF_SEL2_D6OFFINT_MASK       (0x1 << (ADP8866_ISCOFF_SEL2_D6OFFINT_SHIFT))

#define ADP8866_ISCOFF_SEL2_D5OFFINT_SHIFT      (4)
#define ADP8866_ISCOFF_SEL2_D5OFFINT_MASK       (0x1 << (ADP8866_ISCOFF_SEL2_D5OFFINT_SHIFT))

#define ADP8866_ISCOFF_SEL2_D4OFFINT_SHIFT      (3)
#define ADP8866_ISCOFF_SEL2_D4OFFINT_MASK       (0x1 << (ADP8866_ISCOFF_SEL2_D4OFFINT_SHIFT))

#define ADP8866_ISCOFF_SEL2_D3OFFINT_SHIFT      (2)
#define ADP8866_ISCOFF_SEL2_D3OFFINT_MASK       (0x1 << (ADP8866_ISCOFF_SEL2_D3OFFINT_SHIFT))

#define ADP8866_ISCOFF_SEL2_D2OFFINT_SHIFT      (1)
#define ADP8866_ISCOFF_SEL2_D2OFFINT_MASK       (0x1 << (ADP8866_ISCOFF_SEL2_D2OFFINT_SHIFT))

#define ADP8866_ISCOFF_SEL2_D1OFFINT_SHIFT      (0)
#define ADP8866_ISCOFF_SEL2_D1OFFINT_MASK       (0x1 << (ADP8866_ISCOFF_SEL2_D1OFFINT_SHIFT))


// Charge Pump Gain Selection Register
#define ADP8866_GAIN_SEL_1_5X_LIMIT_SHIFT       (2)
#define ADP8866_GAIN_SEL_1_5X_LIMIT_MASK        (0x1 << (ADP8866_GAIN_SEL_1_5X_LIMIT_SHIFT))

#define ADP8866_GAIN_SEL_G_FORCE_SHIFT          (0)
#define ADP8866_GAIN_SEL_G_FORCE_MASK           (0x3 << (ADP8866_GAIN_SEL_G_FORCE_SHIFT))


// Output Level Selection 1 Register
#define ADP8866_LVL_SEL1_D9LVL_SHIFT            (6)
#define ADP8866_LVL_SEL1_D9LVL_MASK             (0x1 << (ADP8866_LVL_SEL1_D9LVL_SHIFT))

#define ADP8866_LVL_SEL1_LEVEL_SET_SHIFT        (0)
#define ADP8866_LVL_SEL1_LEVEL_SET_MASK         (0x3f << (ADP8866_LVL_SEL1_LEVEL_SET_SHIFT))


// Output Level Selection 2 Register
#define ADP8866_LVL_SEL2_D8LVL_SHIFT            (7)
#define ADP8866_LVL_SEL2_D8LVL_MASK             (0x1 << (ADP8866_LVL_SEL2_D8LVL_SHIFT))

#define ADP8866_LVL_SEL2_D7LVL_SHIFT            (6)
#define ADP8866_LVL_SEL2_D7LVL_MASK             (0x1 << (ADP8866_LVL_SEL2_D7LVL_SHIFT))

#define ADP8866_LVL_SEL2_D6LVL_SHIFT            (5)
#define ADP8866_LVL_SEL2_D6LVL_MASK             (0x1 << (ADP8866_LVL_SEL2_D6LVL_SHIFT))

#define ADP8866_LVL_SEL2_D5LVL_SHIFT            (4)
#define ADP8866_LVL_SEL2_D5LVL_MASK             (0x1 << (ADP8866_LVL_SEL2_D5LVL_SHIFT))

#define ADP8866_LVL_SEL2_D4LVL_SHIFT            (3)
#define ADP8866_LVL_SEL2_D4LVL_MASK             (0x1 << (ADP8866_LVL_SEL2_D4LVL_SHIFT))

#define ADP8866_LVL_SEL2_D3LVL_SHIFT            (2)
#define ADP8866_LVL_SEL2_D3LVL_MASK             (0x1 << (ADP8866_LVL_SEL2_D3LVL_SHIFT))

#define ADP8866_LVL_SEL2_D2LVL_SHIFT            (1)
#define ADP8866_LVL_SEL2_D2LVL_MASK             (0x1 << (ADP8866_LVL_SEL2_D2LVL_SHIFT))

#define ADP8866_LVL_SEL2_D1LVL_SHIFT            (0)
#define ADP8866_LVL_SEL2_D1LVL_MASK             (0x1 << (ADP8866_LVL_SEL2_D1LVL_SHIFT))


// LED Power Source Selection 1 Register
#define ADP8866_PWR_SEL1_D9PWR_SHIFT            (0)
#define ADP8866_PWR_SEL1_D9PWR_MASK             (0x1 << (ADP8866_PWR_SEL1_D9PWR_SHIFT))


// LED Power Source Selection 2 Register
#define ADP8866_PWR_SEL2_D8PWR_SHIFT            (7)
#define ADP8866_PWR_SEL2_D8PWR_MASK             (0x1 << (ADP8866_PWR_SEL2_D8PWR_SHIFT))

#define ADP8866_PWR_SEL2_D7PWR_SHIFT            (6)
#define ADP8866_PWR_SEL2_D7PWR_MASK             (0x1 << (ADP8866_PWR_SEL2_D7PWR_SHIFT))

#define ADP8866_PWR_SEL2_D6PWR_SHIFT            (5)
#define ADP8866_PWR_SEL2_D6PWR_MASK             (0x1 << (ADP8866_PWR_SEL2_D6PWR_SHIFT))

#define ADP8866_PWR_SEL2_D5PWR_SHIFT            (4)
#define ADP8866_PWR_SEL2_D5PWR_MASK             (0x1 << (ADP8866_PWR_SEL2_D5PWR_SHIFT))

#define ADP8866_PWR_SEL2_D4PWR_SHIFT            (3)
#define ADP8866_PWR_SEL2_D4PWR_MASK             (0x1 << (ADP8866_PWR_SEL2_D4PWR_SHIFT))

#define ADP8866_PWR_SEL2_D3PWR_SHIFT            (2)
#define ADP8866_PWR_SEL2_D3PWR_MASK             (0x1 << (ADP8866_PWR_SEL2_D3PWR_SHIFT))

#define ADP8866_PWR_SEL2_D2PWR_SHIFT            (1)
#define ADP8866_PWR_SEL2_D2PWR_MASK             (0x1 << (ADP8866_PWR_SEL2_D2PWR_SHIFT))

#define ADP8866_PWR_SEL2_D1PWR_SHIFT            (0)
#define ADP8866_PWR_SEL2_D1PWR_MASK             (0x1 << (ADP8866_PWR_SEL2_D1PWR_SHIFT))


// Configuration Register
#define ADP8866_CFGR_D9SEL_SHIFT                (4)
#define ADP8866_CFGR_D9SEL_MASK                 (0x1 << (ADP8866_CFGR_D9SEL_SHIFT))

#define ADP8866_CFGR_CABCFADE_SHIFT             (3)
#define ADP8866_CFGR_CABCFADE_MASK              (0x1 << (ADP8866_CFGR_CABCFADE_SHIFT))

#define ADP8866_CFGR_BL_LAW_SHIFT               (1)
#define ADP8866_CFGR_BL_LAW_MASK                (0x3 << (ADP8866_CFGR_BL_LAW_SHIFT))


// Backlight Select Register
#define ADP8866_BLSEL_D8SEL_SHIFT               (7)
#define ADP8866_BLSEL_D8SEL_MASK                (0x1 << (ADP8866_BLSEL_D8SEL_SHIFT))

#define ADP8866_BLSEL_D7SEL_SHIFT               (6)
#define ADP8866_BLSEL_D7SEL_MASK                (0x1 << (ADP8866_BLSEL_D7SEL_SHIFT))

#define ADP8866_BLSEL_D6SEL_SHIFT               (5)
#define ADP8866_BLSEL_D6SEL_MASK                (0x1 << (ADP8866_BLSEL_D6SEL_SHIFT))

#define ADP8866_BLSEL_D5SEL_SHIFT               (4)
#define ADP8866_BLSEL_D5SEL_MASK                (0x1 << (ADP8866_BLSEL_D5SEL_SHIFT))

#define ADP8866_BLSEL_D4SEL_SHIFT               (3)
#define ADP8866_BLSEL_D4SEL_MASK                (0x1 << (ADP8866_BLSEL_D4SEL_SHIFT))

#define ADP8866_BLSEL_D3SEL_SHIFT               (2)
#define ADP8866_BLSEL_D3SEL_MASK                (0x1 << (ADP8866_BLSEL_D3SEL_SHIFT))

#define ADP8866_BLSEL_D2SEL_SHIFT               (1)
#define ADP8866_BLSEL_D2SEL_MASK                (0x1 << (ADP8866_BLSEL_D2SEL_SHIFT))

#define ADP8866_BLSEL_D1SEL_SHIFT               (0)
#define ADP8866_BLSEL_D1SEL_MASK                (0x1 << (ADP8866_BLSEL_D1SEL_SHIFT))


// Backlight Fade Register
#define ADP8866_BLFR_BL_FO_SHIFT                (4)
#define ADP8866_BLFR_BL_FO_MASK                 (0xf << (ADP8866_BLFR_BL_FO_SHIFT))

#define ADP8866_BLFR_BL_FI_SHIFT                (0)
#define ADP8866_BLFR_BL_FI_MASK                 (0xf << (ADP8866_BLFR_BL_FI_SHIFT))


// Backlight Maximum Current Register
#define ADP8866_BLMX_BL_MC_SHIFT                (0)
#define ADP8866_BLMX_BL_MC_MASK                 (0x7f << (ADP8866_BLMX_BL_MC_SHIFT))


// Independent Sink Current Control Register 1
#define ADP8866_ISCC1_SC9_EN_SHIFT              (2)
#define ADP8866_ISCC1_SC9_EN_MASK               (0x1 << (ADP8866_ISCC1_SC9_EN_SHIFT))

#define ADP8866_ISCC1_SC_LAW_SHIFT              (0)
#define ADP8866_ISCC1_SC_LAW_MASK               (0x3 << (ADP8866_ISCC1_SC_LAW_SHIFT))


// Independent Sink Current Control Register 2
#define ADP8866_ISCC2_SC8_EN_SHIFT              (7)
#define ADP8866_ISCC2_SC8_EN_MASK               (0x1 << (ADP8866_ISCC2_SC8_EN_SHIFT))

#define ADP8866_ISCC2_SC7_EN_SHIFT              (6)
#define ADP8866_ISCC2_SC7_EN_MASK               (0x1 << (ADP8866_ISCC2_SC7_EN_SHIFT))

#define ADP8866_ISCC2_SC6_EN_SHIFT              (5)
#define ADP8866_ISCC2_SC6_EN_MASK               (0x1 << (ADP8866_ISCC2_SC6_EN_SHIFT))

#define ADP8866_ISCC2_SC5_EN_SHIFT              (4)
#define ADP8866_ISCC2_SC5_EN_MASK               (0x1 << (ADP8866_ISCC2_SC5_EN_SHIFT))

#define ADP8866_ISCC2_SC4_EN_SHIFT              (3)
#define ADP8866_ISCC2_SC4_EN_MASK               (0x1 << (ADP8866_ISCC2_SC4_EN_SHIFT))

#define ADP8866_ISCC2_SC3_EN_SHIFT              (2)
#define ADP8866_ISCC2_SC3_EN_MASK               (0x1 << (ADP8866_ISCC2_SC3_EN_SHIFT))

#define ADP8866_ISCC2_SC2_EN_SHIFT              (1)
#define ADP8866_ISCC2_SC2_EN_MASK               (0x1 << (ADP8866_ISCC2_SC2_EN_SHIFT))

#define ADP8866_ISCC2_SC1_EN_SHIFT              (0)
#define ADP8866_ISCC2_SC1_EN_MASK               (0x1 << (ADP8866_ISCC2_SC1_EN_SHIFT))


// Independent Sink Current Time Register
#define ADP8866_ISCT1_SCON_SHIFT                (4)
#define ADP8866_ISCT1_SCON_MASK                 (0xf << (ADP8866_ISCT1_SCON_SHIFT))

#define ADP8866_ISCT1_SC5OFF_SHIFT              (0)
#define ADP8866_ISCT1_SC5OFF_MASK               (0x3 << (ADP8866_ISCT1_SC5OFF_SHIFT))


// Independent Sink Current Time Register
#define ADP8866_ISCT2_SC4OFF_SHIFT              (6)
#define ADP8866_ISCT2_SC4OFF_MASK               (0x3 << (ADP8866_ISCT2_SC4OFF_SHIFT))

#define ADP8866_ISCT2_SC3OFF_SHIFT              (4)
#define ADP8866_ISCT2_SC3OFF_MASK               (0x3 << (ADP8866_ISCT2_SC3OFF_SHIFT))

#define ADP8866_ISCT2_SC2OFF_SHIFT              (2)
#define ADP8866_ISCT2_SC2OFF_MASK               (0x3 << (ADP8866_ISCT2_SC2OFF_SHIFT))

#define ADP8866_ISCT2_SC1OFF_SHIFT              (0)
#define ADP8866_ISCT2_SC1OFF_MASK               (0x3 << (ADP8866_ISCT2_SC1OFF_SHIFT))


// Independent Sink Off Timer Registers
#define ADP8866_OFFTIMER_SC_OFF_SHIFT           (0)
#define ADP8866_OFFTIMER_SC_OFF_MASK            (0x7f << (ADP8866_OFFTIMER_SC_OFF_SHIFT))


// Independent Sink Current Fade Register
#define ADP8866_ISCF_SCFO_SHIFT                 (4)
#define ADP8866_ISCF_SCFO_MASK                  (0xf << (ADP8866_ISCF_SCFO_SHIFT))

#define ADP8866_ISCF_SCFI_SHIFT                 (0)
#define ADP8866_ISCF_SCFI_MASK                  (0xf << (ADP8866_ISCF_SCFI_SHIFT))


// Sink Current Register LED1 Registers
#define ADP8866_ISC_SCD_SHIFT                   (0)
#define ADP8866_ISC_SCD_MASK                    (0x7f << (ADP8866_ISC_SCD_SHIFT))


// Heartbeat Enable Selection Register
#define ADP8866_HB_SEL_D9HB_EN_SHIFT            (3)
#define ADP8866_HB_SEL_D9HB_EN_MASK             (0x1 << (ADP8866_HB_SEL_D9HB_EN_SHIFT))

#define ADP8866_HB_SEL_D8HB_EN_SHIFT            (2)
#define ADP8866_HB_SEL_D8HB_EN_MASK             (0x1 << (ADP8866_HB_SEL_D8HB_EN_SHIFT))

#define ADP8866_HB_SEL_D7HB_EN_SHIFT            (1)
#define ADP8866_HB_SEL_D7HB_EN_MASK             (0x1 << (ADP8866_HB_SEL_D7HB_EN_SHIFT))

#define ADP8866_HB_SEL_D6HB_EN_SHIFT            (0)
#define ADP8866_HB_SEL_D6HB_EN_MASK             (0x1 << (ADP8866_HB_SEL_D6HB_EN_SHIFT))


// Independent Sink Current LED Registers
#define ADP8866_ISC_HB_SCD_SHIFT                (0)
#define ADP8866_ISC_HB_SCD_MASK                 (0x7f << (ADP8866_ISC_HB_SCD_SHIFT))


// Independent Sink Off Timer Registers
#define ADP8866_OFFTIMER_HB_SC_OFF_SHIFT        (0)
#define ADP8866_OFFTIMER_HB_SC_OFF_MASK         (0x7f << (ADP8866_OFFTIMER_HB_SC_OFF_SHIFT))


// Heartbeat On Time Register
#define ADP8866_ISCT_HB_SCON_HB_SHIFT           (0)
#define ADP8866_ISCT_HB_SCON_HB_MASK            (0xf << (ADP8866_ISCT_HB_SCON_HB_SHIFT))


// Enable Delay Time SC Registers
#define ADP8866_DELAY_SHIFT                     (0)
#define ADP8866_DELAY_MASK                      (0x7f << (ADP8866_DELAY_SHIFT))

} // anonymous namespace

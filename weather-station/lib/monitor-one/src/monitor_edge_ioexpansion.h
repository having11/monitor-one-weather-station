/*
 * Copyright (c) 2023 Particle Industries, Inc.
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

//
// Monitor One IO Expansion Specifc IO
//
#define MONITOREDGE_IOEX_VOLTAGE_IN_PIN         (A6)
#define MONITOREDGE_IOEX_CURRENT_IN_PIN         (A7)
#define MONITOREDGE_IOEX_DIGITAL_IN_PIN         (D5)
#define MONITOREDGE_IOEX_RELAY_OUT_PIN          (NFC_PIN2)

// RS-485 Specific (uses Serial1)
#define MONITOREDGE_IOEX_RS485_RX_PIN           (RX)
#define MONITOREDGE_IOEX_RS485_TX_PIN           (TX)
#define MONITOREDGE_IOEX_RS485_DE_PIN           (D4)

constexpr char MONITOREDGE_IOEX_SKU[]           {"EXP1_IO_BASIC_485CAN"};

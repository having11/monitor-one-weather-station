# ModbusClient

[GitHub release]:   https://github.com/particle-iot/ModbusClient
[license]:          LICENSE

This is an Particle library for communicating with Modbus servers over RS232/485 (via RTU protocol).  It has been forked from [ModbusMaster](https://github.com/4-20ma/ModbusClient) written by Doc Walker.

---

## Features
The following Modbus functions are available:

Discrete Coils/Flags

  - 0x01 - Read Coils
  - 0x02 - Read Discrete Inputs
  - 0x05 - Write Single Coil
  - 0x0F - Write Multiple Coils

Registers

  - 0x03 - Read Holding Registers
  - 0x04 - Read Input Registers
  - 0x06 - Write Single Register
  - 0x10 - Write Multiple Registers
  - 0x16 - Mask Write Register
  - 0x17 - Read Write Multiple Registers

Both full-duplex and half-duplex RS232/485 transceivers are supported. Callback functions are provided to toggle Data Enable (DE) and Receiver Enable (/RE) pins.


## Installation

#### Library Manager
Install the library into your Arduino IDE using the Library Manager (available from IDE version 1.6.2). Open the IDE and click Sketch > Include Library > Manage Libraries&hellip;

Scroll or search for `ModbusClient`, then select the version of the library you want to install. Quit/re-launch the IDE to refresh the list; new versions are automatically added to the list, once released on GitHub.

Refer to Arduino Tutorials > Libraries [Using the Library Manager](https://www.arduino.cc/en/Guide/Libraries#toc3).

## Hardware
This library has been tested with a Particle [Monitor One](https://www.particle.io/devices/monitor-one/) connected via RS485 using a Maxim [MAX3485CSA](https://www.analog.com/en/products/max3485.html) transceiver.

## Example

``` cpp
#include "Particle.h"
#include "ModbusClient.h"

#define MAX485_DE      (D4)

// instantiate ModbusClient object
ModbusClient node;
ModbusClientContext mbContext {};

void setup()
{
  // Use UART-based Serial; initialize Modbus communication baud rate
  Serial1.begin(19200, SERIAL_DATA_BITS_8 | SERIAL_STOP_BITS_1 | SERIAL_PARITY_NO);

  // Communicate Serial1 (can be any Stream class as well)
  node.begin(Serial1);

  // Callbacks allow us to configure the RS485 transceiver correctly
  node.preTransmission([]() {digitalWrite(MAX485_DE, 1);});
  node.postTransmission([]() {digitalWrite(MAX485_DE, 0);});
}


void loop()
{
  static uint32_t i;
  uint8_t j, result;
  uint16_t data[6];

  i++;

  // set word 0 of TX buffer to least-significant word of counter (bits 15..0)
  mbContext.writeBuffer[0] = lowWord(i);

  // set word 1 of TX buffer to most-significant word of counter (bits 31..16)
  mbContext.writeBuffer[1] = highWord(i);

  // server: write TX buffer to (2) 16-bit registers starting at register 0 on server device ID 32
  result = node.writeMultipleRegisters(32, 0, 2, mbContext);

  // server: read (6) 16-bit registers starting at register 2 to RX buffer on server device ID 32
  result = node.readHoldingRegisters(32, 2, 6, mbContext);

  // do something with data if read is successful
  if (result == node.ku8MBSuccess)
  {
    // response contained in context read buffer
    // mbContext.readBuffer[0]
    // mbContext.readBuffer[1]
    // mbContext.readBuffer[2]
    // mbContext.readBuffer[3]
    // mbContext.readBuffer[4]
    // mbContext.readBuffer[5]
  }
}
```

---

### LICENSE

Unless stated elsewhere, file headers or otherwise, all files herein are licensed under an Apache License, Version 2.0. For more information, please read the LICENSE file.

If you have questions about software licensing, please contact Particle [support](https://support.particle.io/).

---

### LICENSE FAQ

**This firmware is released under Apache License, Version 2.0, what does that mean for you?**

 * You may use this commercially to build applications for your devices!  You **DO NOT** need to distribute your object files or the source code of your application under Apache License.  Your source can be released under a non-Apache license.  Your source code belongs to you when you build an application using this library.

**When am I required to share my code?**

 * You are **NOT required** to share your application firmware binaries, source, or object files when linking against libraries or System Firmware licensed under LGPL.

**Why?**

 * This license allows businesses to confidently build firmware and make devices without risk to their intellectual property, while at the same time helping the community benefit from non-proprietary contributions to the shared reference firmware.

**Questions / Concerns?**

 * Particle intends for this firmware to be commercially useful and safe for our community of makers and enterprises.  Please [Contact Us](https://support.particle.io/) if you have any questions or concerns, or if you require special licensing.

_(Note!  This FAQ isn't meant to be legal advice, if you're unsure, please consult an attorney)_

---

### CONNECT

Having problems or have awesome suggestions? Connect with us [here.](https://community.particle.io/)

---

### Revision History

#### 1.0.0
* Initial version

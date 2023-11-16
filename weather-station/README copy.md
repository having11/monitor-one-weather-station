
# Particle Monitor Edge Firmware

Application reference firmware for the Particle Monitor One.

# Resources

- [Latest Release](https://github.com/particle-iot/monitor-edge/releases)
- [Changelog](CHANGELOG.md)

### CREDITS AND ATTRIBUTIONS

The firmware uses the GNU GCC toolchain for ARM Cortex-M processors, standard peripheral libraries and Arduino's implementation of Wiring.

### LICENSE

Unless stated elsewhere, file headers or otherwise, all files herein are licensed under an Apache License, Version 2.0. For more information, please read the LICENSE file.

If you have questions about software licensing, please contact Particle [support](https://support.particle.io/).


### LICENSE FAQ

**This firmware is released under Apache License, Version 2.0, what does that mean for you?**

 * You may use this commercially to build applications for your devices!  You **DO NOT** need to distribute your object files or the source code of your application under Apache License.  Your source can be released under a non-Apache license.  Your source code belongs to you when you build an application using this reference firmware.

**When am I required to share my code?**

 * You are **NOT required** to share your application firmware binaries, source, or object files when linking against libraries or System Firmware licensed under LGPL.

**Why?**

 * This license allows businesses to confidently build firmware and make devices without risk to their intellectual property, while at the same time helping the community benefit from non-proprietary contributions to the shared reference firmware.

**Questions / Concerns?**

 * Particle intends for this firmware to be commercially useful and safe for our community of makers and enterprises.  Please [Contact Us](https://support.particle.io/) if you have any questions or concerns, or if you require special licensing.

_(Note!  This FAQ isn't meant to be legal advice, if you're unsure, please consult an attorney)_


### COMPILE & FLASH WITH WORKBENCH

This application must be built with device OS version 4.0.2 and above.

1. Clone this repository `$ git clone git@github.com:particle-iot/monitor-edge.git && cd ./monitor-edge`
2. Open Particle Workbench
3. Run the `Particle: Import Project` command, follow the prompts, and wait for the project to load
4. Run the `Particle: Configure Workspace for Device` command and select a compatible Device OS version and the `tracker` platform when prompted ([docs](https://docs.particle.io/tutorials/developer-tools/workbench/#cloud-build-and-flash))
5. Connect your device
6. Compile & Flash!

### Expansion Card EEPROM Usage

As this project comes out-of-the-box, there is an expectation that an EEPROM is present and programmed on the expansion card.  This EEPROM is used to help differentiate between various Particle supplied expansion cards.  As there may be very different functionality between card SKUs, the SKU contained in the EEPROM is beneficial to steer Monitor Edge functionality between one card versus another.

The EEPROM has a simple structure programmed into the first couple of 32-byte pages.  Details and helper functions can be found in [lib/edge/src/eeprom_helper.h](lib/edge/src/eeprom_helper.h).  The basic structure is comprised of the following fields:

```cpp
struct ExpansionEeprom1 {
    uint16_t    size;           ///< Size of entire ExpansionEeprom
                                ///< structure. LSB, MSB
    uint8_t     revision;       ///< Revision number of this hardware
                                ///< starting from 1
    char        sku[29];        ///< SKU name with null termination
    uint8_t     serial[16];     ///< 128-bit serial number MSB->LSB
    uint8_t     reserved[16];   ///< Page boundary filler
} __attribute__((packed));
```

* The `size` field is simply filled with `sizeof(ExpansionEeprom1)`.
* `revision` usually follows the hardware revision and set during manufacturing stages.
* The `sku` string field is important for expansion card differentiation and is evaluated during the boot/setup stage of execution.
* `serial` may or not be programmed at manufacturing and is used to identify specific expansion cards.  This field is ignored if the EEPROM has built-in unique serial numbers.

Everything after the 64-byte `ExpansionEeprom1` structure is open for user purposes including calibration tables or manufacturing data.

Common `sku` strings include:
* "EXP1_IO_BASIC_485CAN" is populated for IO expansion cards.
* "EXP1_PROTO" is populated for Particle's protoboard expansion cards.

If no EEPROM SKU is found then Monitor Edge will boot with generic Tracker Edge-like functionality.

Look at source where the EEPROM is read and SKU handled in [src/user_setup.cpp](src/user_setup.cpp) within the `user_init()` function for more detailed implementation details.

### General Directory and File Layout

`src/` and `lib/` directories are typically used for Particle application projects.  Library project files from other repositories and source that is general purpose for most applications are found in the `lib/` directory.  Custom appliction source code that vary from user-to-user, application-to-application are typically located in the `src/` directory.  All source code can be modified for any application use but it is good practice to modify in `src/` and leave `lib/` alone so that it easier to sync future bug fixes as well as features in this project.

In addition to `main.cpp`, there are four files that are left for user customization in the `src/` directory.  They include:
```
src/
   main.cpp             <- Created with very minimal code.  It is the entry point to setup() and loop().
   user_config.h        <- Used to consolidate control over what is compiled in the project. 
   user_setup.cpp       <- Contains common Monitor One elements.
   user_io.cpp          <- Very specific to the IO expansion card.
   user_modbus.cpp      <- Also very specific to the IO expansion card.
```

As mentioned, `main.cpp` contains very minimal code and was historically meant to be a clean slate for new users to develop their applications.  It was coded very minimally so that the user is aware that more complex stuff is happening undernealth but their application source is more navigatable and updates with the Monitor Edge github repository don't result in a bunch of git conflicts.

Monitor One/Edge allows for expansion through the internal headers that Tracker One/Edge didn't really accomodate.  As a result, various expansion cards can be installed which brings up the dilemma of having one reference application serve mutliple off-the-shelf Particle expansion SKUs.  The `user_setup.cpp` file attempts to solve this by initializing common Monitor One peripherals such as the RGB status LED and push button switch.  The user is free to change the functionality of the push button, change RGB status states, as well as control the side mounted RGB LEDs.  Two functions, `user_init()` and `user_loop()` are overriding weak, bare declarations inside of [lib/edge/src/edge.cpp](lib/edge/src/edge.cpp).  If the user wishes to delete `user_setup.cpp`, `user_io.cpp`, and `user_modbus.cpp` there will be no ill consequences and Monitor Edge will revert to basic Tracker Edge functionality.

Both `user_io.cpp` and `user_modbus.cpp` are there to support the IO expansion board.  They add Particle cloud functions, variables, and configuration parameters that are relevant only to the IO expansion card.  They can be deleted entirely if not used or simply kept from compilation setting defines in `user_config.h`.

### CONTRIBUTE

Want to contribute to the Particle tracker edge firmware project? Follow [this link](CONTRIBUTING.md) to find out how.

### CONNECT

Having problems or have awesome suggestions? Connect with us [here.](https://community.particle.io/c/tracking-system).

Enterprise customers can contact [support](https://support.particle.io/).

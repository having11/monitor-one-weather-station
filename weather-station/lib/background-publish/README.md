# background-publish
Particle library to publish data in a non-blocking manor. Creates a vector of 
N Queues with N priority levels that are zero indexed. Lower index (level) is a 
higher priority (i.e 0 index highest priority), and higher indexes have lower 
priority. The highest priority queue is processed first in the thread, and 
then the lower priority queue is processed next, and so forth. A callback can
be called after a request for publishing has finished. The status of that 
particular publish is passed to the callback. The user is responsible for keeping
the lifetime of the data to publish valid until the publish has finished 
(callback fired), since the library does not copy data to send. 
The BackgroundPublish class is a singleton, so you will only be able to create 
one instance of the class.

### Example
Look at the usage.cpp file for a basic example of how to use the library. Merely
call BackgroundPublish::instance() to access the public functions. You'll need
to call init() once on start up, and then publish() when you want to send data.
If you need to flush the queues (before shutting down or going to sleep) 
call cleanup(). It's that simple.

### Unit tests
Directions for running unit tests:
1. `mkdir build`
2. `cd build && cmake ..`
3. `make`
4. `./background-publish-test`

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

#### 1.1.0
* Allow burst sends without a fixed processing interval
* Numerous fixes and cleanup

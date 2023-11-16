/*
 * Project qtest
 * Description:
 * Author:
 * Date:
 */

#include "Particle.h"
#include "DiskQueue.h"

SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(SEMI_AUTOMATIC);

DiskQueue gq(4096,10);

SerialLogHandler logHandler(115200, LOG_LEVEL_TRACE);

// pause user app execution
static void park() {
  while(true) {
    delay(100);
  }
}

// setup() runs once, when the device is first turned on.
void setup() {
  // Put initialization like pinMode and begin functions here.

  delay(3000);
  int rc = gq.start("/my_cache");
  Log.info("gq start: %d", rc);
  if (SYSTEM_ERROR_NONE != rc) {
    park();
  }

}

static uint32_t loop_count = 0;
static uint32_t push_count = 0;
const size_t MAX_DATA_LEN = 1024;
static uint8_t  max_data_buf[MAX_DATA_LEN] = {};
// loop() runs over and over again, as quickly as it can execute.
void loop() {
  String test_str = String::format("%lu",loop_count);
  size_t len = strlen(test_str);
  // The core of your code will likely live here.
  if (!gq.push_back(test_str, len)) {
    Log.warn("pushback failed: %u", len);
    delay(10);
  }
  else {
    push_count += 1;
  }

  if (push_count > 3) {
    max_data_buf[0] = 0;
    size_t read_size = 0;
    if (!gq.peek_front(max_data_buf, read_size)) {
      Log.warn("front failed (%u)", read_size);
    }
    else {
      Log.info("Read %u : %s",read_size, max_data_buf);
    }
  }
  else {
    delay(10);
  }
}
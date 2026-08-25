/*
  =============================================================================
  ESPFlight Open Source Flight Controller Firmware for ESP8266
  =============================================================================

  This lightweight sketch only forwards Arduino setup and loop execution to
  the firmware lifecycle module. All tested behavior remains in firmware.cpp.

  Author: Ali KarimiZadeh
  Author Website: https://karimizadeh.site
  ESPFlight Website: https://espflight.com
  License: MIT
*/

#include "firmware.h"  // Provide the complete ESPFlight setup and loop lifecycle.

void setup() {      // Receive the Arduino framework setup callback.
  firmwareSetup();  // Run the complete preserved ESPFlight initialization sequence.
}  // Finish the Arduino setup callback.

void loop() {      // Receive the Arduino framework loop callback.
  firmwareLoop();  // Run one iteration of the complete preserved ESPFlight runtime loop.
}  // Finish the Arduino loop callback.

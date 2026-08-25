#ifndef INDICATORS_H  // Prevent this indicator header from being included more than once.
#define INDICATORS_H  // Define the include guard used by this indicator header.

#include <Arduino.h>  // Provide GPIO, timing, tone, and delay functions.
#include "config.h"   // Provide the preserved buzzer pin assignment.

#define BUZZER_PIN ESPFLIGHT_BUZZER_PIN  // Preserve the public buzzer pin name used by the failsafe module.

void playStartupMelody();  // Play the original blocking startup melody exactly once during setup.
void errorSignal();        // Display the original I2C error pulse pattern on the status LED.

#endif  // Close the INDICATORS_H include guard.

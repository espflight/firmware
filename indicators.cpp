#include <Arduino.h>         // Provide GPIO, timing, tone, and delay functions.
#include "indicators.h"      // Provide the public startup melody and error indicator declarations.
#include "flight_control.h"  // Provide the original LED state and error-pattern runtime variables.
#include "imu.h"             // Provide the current MPU6050 I2C error counter.

#define NOTE_E7 2637  // Preserve the E7 frequency used by the original startup melody.
#define NOTE_G7 3136  // Preserve the G7 frequency used by the original startup melody.
#define NOTE_A7 3520  // Preserve the A7 frequency used by the original startup melody.
#define NOTE_B7 3951  // Preserve the B7 frequency used by the original startup melody.
#define NOTE_C8 4186  // Preserve the C8 frequency used by the original startup melody.

int melody[] = {  // Store the startup melody frequencies and zero-valued rests in playback order.
  // Store the original startup melody without changing its note order.
  NOTE_E7, NOTE_E7, 0, NOTE_E7,        // Preserve the first four original melody entries.
  0, NOTE_C8, NOTE_E7, 0,              // Preserve the next four original melody entries.
  NOTE_G7, 0, 0, 0,                    // Preserve the original G7 note and following rests.
  NOTE_G7, NOTE_A7, NOTE_G7, NOTE_E7,  // Preserve the original descending phrase.
  NOTE_C8, NOTE_B7, NOTE_A7            // Preserve the original final phrase.
};                                     // Finish defining the original startup melody.

int noteDurations[] = {  // Store the duration divisor corresponding to each startup melody entry.
  // Store the original duration divisor for every melody entry.
  8, 8, 8, 8,  // Preserve the first four eighth-note durations.
  8, 8, 8, 8,  // Preserve the next four eighth-note durations.
  8, 8, 8, 8,  // Preserve the next four eighth-note durations.
  8, 8, 8, 8,  // Preserve the next four eighth-note durations.
  8, 8, 8      // Preserve the final three eighth-note durations.
};             // Finish defining the original note durations.

const int melodyLength = sizeof(melody) / sizeof(int);  // Calculate the original startup melody entry count.

void playStartupMelody() {                                       // Play the startup melody using the same logic and timing as the tested firmware.
  for (int thisNote = 0; thisNote < melodyLength; thisNote++) {  // Visit every note and rest in the original order.
    int duration = 1000 / noteDurations[thisNote];               // Convert the stored divisor into the note duration in milliseconds.
    int freq = melody[thisNote];                                 // Read the frequency of the current note or rest.
    if (freq == 0) {                                             // Detect a rest entry in the melody.
      delay(duration);                                           // Wait for the original rest duration.
    } else {                                                     // Handle an audible melody note.
      tone(BUZZER_PIN, freq, duration);                          // Generate the original tone on the preserved buzzer pin.
      delay(duration * 1.3);                                     // Preserve the original pause between notes.
      noTone(BUZZER_PIN);                                        // Stop the tone before advancing to the next entry.
    }                                                            // Finish handling the current note or rest.
  }                                                              // Finish playing every startup melody entry.
}  // Finish the original startup melody routine.

void errorSignal() {         // Display the current consecutive I2C error indication.
  if (i2cError == 0U) {      // Detect recovery after one complete valid MPU6050 sample.
    digitalWrite(LED, LOW);  // Turn the error indicator off immediately after recovery.
    error_counter = 0U;      // Reset the pulse counter for the next independent error sequence.
    error_led = 0U;          // Record that the error LED is currently off.
    return;                  // Skip the flashing sequence while communication is healthy.
  }                          // Finish handling the healthy I2C state.

  if (i2cError >= 100U) {     // Detect a critical consecutive I2C error condition.
    digitalWrite(LED, HIGH);  // Keep the status LED continuously on for the critical condition.
    return;                   // Skip the normal flashing sequence while the condition remains critical.
  }                           // Finish handling the critical I2C error condition.

  if (millis() >= error_timer) {   // Check whether the next original 250-millisecond indicator step has arrived.
    error_timer = millis() + 250;  // Schedule the next original LED indicator step.

    if (i2cError > 0 && error_counter > i2cError + 3) {  // Detect the end of the current error flash sequence.
      error_counter = 0;                                 // Restart the original error flash sequence.
    }                                                    // Finish checking whether the flash sequence must restart.

    if (error_counter < i2cError && error_led == 0 && i2cError > 0) {  // Determine whether the LED should begin the next error pulse.
      digitalWrite(LED, HIGH);                                         // Turn the status LED on for the current error pulse.
      error_led = 1;                                                   // Record that the error LED is currently on.
    } else {                                                           // Handle the off step or the pause after the pulse sequence.
      digitalWrite(LED, LOW);                                          // Turn the status LED off.
      error_counter++;                                                 // Advance the original error flash counter.
      error_led = 0;                                                   // Record that the error LED is currently off.
    }                                                                  // Finish updating the original error LED state.
  }                                                                    // Finish the scheduled error indicator update.
}  // Finish the original error indicator routine.

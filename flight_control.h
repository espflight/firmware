#ifndef FLIGHT_CONTROL_H  // Start the FLIGHT_CONTROL_H include guard.
#define FLIGHT_CONTROL_H  // Define the FLIGHT_CONTROL_H preprocessor symbol.

#include <Arduino.h>  // Provide Arduino types, GPIO functions, timing functions, and board definitions.
#include "config.h"   // Provide the preserved hardware and timing configuration values.
#include <stdint.h>   // Import declarations supplied by <stdint.h>.
#include <stdbool.h>  // Import declarations supplied by <stdbool.h>.

// LED pin
extern const int LED;  // Declare the shared LED symbol defined by another module.

// Motor pins
extern const int M1;  // Declare the shared M1 symbol defined by another module.
extern const int M2;  // Declare the shared M2 symbol defined by another module.
extern const int M3;  // Declare the shared M3 symbol defined by another module.
extern const int M4;  // Declare the shared M4 symbol defined by another module.

extern unsigned long flightStartTime;  // Store the start of the current ARMED + effective-throttle-above-1100 timing segment.
extern unsigned long totalFlightTime;  // Accumulate qualifying real-flight milliseconds across the current board power cycle.
extern bool flightTimerRunning;        // Track whether the ARMED + effective-throttle-above-1100 timing condition is currently active.
extern bool flightSessionStarted;      // Track whether this powered board has started its cumulative timing session.
extern uint32_t flightSessionId;       // Publish the current powered-session identifier to telemetry.
extern uint32_t bootSessionId;         // Publish a unique identifier for the current firmware boot.

// WiFi status
extern unsigned long lastWiFiCheck;            // Execute this statement as part of the current operation.
extern const unsigned long wifiCheckInterval;  // Execute this statement as part of the current operation.
extern unsigned long wifiDisconnectedSince;    // Execute this statement as part of the current operation.
extern bool wifiWasConnected;                  // Declare the shared wifiWasConnected symbol defined by another module.


// Loop / timing variables
extern unsigned long Time;               // Execute this statement as part of the current operation.
extern unsigned long loopStartTime;      // Execute this statement as part of the current operation.
extern unsigned long lastLoopStartTime;  // Execute this statement as part of the current operation.
extern const unsigned long loopTime;     // Execute this statement as part of the current operation.
extern unsigned long nowMilis;           // Execute this statement as part of the current operation.

// Debug / testing
extern unsigned long total;         // Execute this statement as part of the current operation.
extern unsigned int count;          // Declare the shared count symbol defined by another module.
extern unsigned long lastBeepTime;  // Execute this statement as part of the current operation.

// -------------------------------
// Global battery voltage (Volts)
// Updated by failsafe logic
// -------------------------------

extern volatile float battery_voltage;  // Declare the shared battery_voltage symbol defined by another module.

// RC channels
extern int rc_throttle;  // Declare the shared rc_throttle symbol defined by another module.
extern int rc_roll;      // Declare the shared rc_roll symbol defined by another module.
extern int rc_pitch;     // Declare the shared rc_pitch symbol defined by another module.
extern int rc_yaw;       // Declare the shared rc_yaw symbol defined by another module.

extern int throttle;  // Declare the shared mixer throttle defined by the flight-control module.

// Raw RC channel values
extern int32_t channel_1, channel_2, channel_3, channel_4;  // Declare the shared channel_4 symbol defined by another module.

// PID runtime variables
extern float roll_level_adjust;   // Declare the shared roll_level_adjust symbol defined by another module.
extern float pitch_level_adjust;  // Declare the shared pitch_level_adjust symbol defined by another module.

// Misc flight variables
extern float angle_roll_acc;   // Declare the shared angle_roll_acc symbol defined by another module.
extern float angle_pitch_acc;  // Declare the shared angle_pitch_acc symbol defined by another module.
extern float angle_pitch;      // Declare the shared angle_pitch symbol defined by another module.
extern float angle_roll;       // Declare the shared angle_roll symbol defined by another module.

// I2C and misc (only non-IMU globals)
extern uint8_t start;          // Declare the shared start symbol defined by another module.
extern uint8_t error_counter;  // Declare the shared error_counter symbol defined by another module.
extern uint8_t error_led;      // Declare the shared error_led symbol defined by another module.

extern int16_t esc_1;  // Declare the shared esc_1 symbol defined by another module.
extern int16_t esc_2;  // Declare the shared esc_2 symbol defined by another module.
extern int16_t esc_3;  // Declare the shared esc_3 symbol defined by another module.
extern int16_t esc_4;  // Declare the shared esc_4 symbol defined by another module.


extern uint32_t error_timer;  // Declare the shared error_timer symbol defined by another module.


// Configure the original LED and motor pins before the flight controller starts.
void pinConfig();  // Configure the original LED and four motor output pins.

// Run one complete 250-Hz attitude-control, failsafe, mixer, and PWM update.
void flightControl250Hz();  // Execute one preserved flight-control iteration.

bool flightControlImuReady(uint32_t nowMs);  // Report whether command-time MPU6050 data and attitude are fresh and safe for ARM.
bool flightControlRequestArm(uint32_t nowMs);  // Enter ARMED-ready state while keeping every motor PWM output at zero.
void flightControlRequestDisarm();  // Cancel assist/PID and force immediate zero-PWM DISARM.
void flightControlEmergencyStop();  // Disarm, stop the flight timer, and force every motor PWM output to zero.

void inputReceiver();  // Invoke or continue inputReceiver for the current operation.

// Flight timer functions
void startFlightTimer();               // Start actual powered-flight timing; ARM-ready idle time is not counted.
void stopFlightTimer();                // Pause timing and preserve cumulative armed duration.
void flightControlEndSession();        // Preserve compatibility; link loss no longer resets the powered timer session.
unsigned long getFlightTimeSeconds();  // Get cumulative armed seconds for the current board power cycle.

#endif  // Close the FLIGHT_CONTROL_H include guard.

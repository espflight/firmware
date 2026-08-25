#ifndef ALTITUDE_H
#define ALTITUDE_H

#include <Arduino.h>           // Provide timing and fixed-width Arduino types.
#include <Adafruit_VL53L0X.h>  // Provide the optional VL53L0X time-of-flight sensor driver.

// Assisted flight deliberately keeps the same control relationship that was
// previously flight-tested: the application ramps channel_3 to 1500, the
// aircraft lifts on raw throttle, and only after real height is detected does
// altitude PID add its correction around that same 1500 base command.
enum AltitudeAssistMode : uint8_t {
  ALTITUDE_ASSIST_OFF = 0,
  ALTITUDE_ASSIST_TAKEOFF = 1,
  ALTITUDE_ASSIST_HOLD = 2,
  ALTITUDE_ASSIST_LANDING = 3,
};

extern Adafruit_VL53L0X lox;             // Own the single optional VL53L0X driver instance.
extern unsigned long lastSensorRead;     // Store the newest scheduled physical range-read time.
extern const unsigned long sensorInterval;  // Preserve the 50-millisecond physical measurement interval.
extern uint16_t lastDistance;            // Preserve the previous accepted raw distance.
extern uint16_t distance;                // Store the newest accepted raw distance in millimeters.
extern float filtered_distance;          // Publish the continuously filtered altitude in millimeters.
extern bool sensorTimeout;               // Report whether recent physical measurements became invalid/stale.
extern bool altHoldEnabled;              // Report whether the app requested the tested altitude-assist path.
extern bool alt_flag;                    // Report whether altitude PID is currently allowed to modify throttle.
extern bool vl53_available;              // Report whether the optional sensor was detected at boot.

bool altitudeInit();                                  // Detect the optional VL53L0X and start continuous ranging without blocking manual flight when absent.
void readAltitude(unsigned long nowMs);               // Preserve the old 20-Hz physical read plus 250-Hz-style Kalman behavior.
bool altitudeSensorReady(unsigned long nowMs);        // Require multiple recent valid measurements before accepting Takeoff.
bool altitudeSensorHealthy(unsigned long nowMs);      // Keep active assistance alive across brief bad samples and fail only after the real freshness timeout.
bool altitudeRequestTakeoff(unsigned long nowMs);     // Arm altitude assistance while leaving the initial throttle ramp entirely driven by the application.
bool altitudeRequestLanding(unsigned long nowMs);     // Switch active Hold into the slower app-driven descent while firmware verifies real touchdown with VL53L0X.
void altitudeUpdateControlState(unsigned long nowMs); // Transition Takeoff -> Hold and protect active assistance from sensor loss or excessive height.
void altitudeResetAssist();                           // Clear altitude-assist state and altitude PID runtime values.
bool altitudePidActive();                             // Report whether the legacy altitude PID correction must be added to throttle.
bool altitudeLandingShouldDisarm();                   // Allow final DISARM only after minimum throttle and independent near-floor VL53L0X samples confirm touchdown.
AltitudeAssistMode altitudeAssistMode();              // Return the current assisted-flight phase for telemetry and command validation.
const char *altitudeAssistModeName();                  // Return a stable short mode name for telemetry.

#endif

#ifndef FAILSAFE_H  // Prevent this header from being included more than once.
#define FAILSAFE_H  // Define the include guard used by this header.

#include <Arduino.h>  // Provide Arduino types, constants, and timing functions.

enum FailsafeReason : uint8_t {          // Define independent bit flags for every failsafe source.
  FAILSAFE_REASON_NONE = 0,              // Indicate that no failsafe reason is currently latched.
  FAILSAFE_REASON_LINK_LOSS = 1U << 0,   // Indicate that valid control packets have timed out.
  FAILSAFE_REASON_LOW_BATTERY = 1U << 1, // Indicate that the tested low-voltage condition is latched.
  FAILSAFE_REASON_ALTITUDE_ASSIST = 1U << 2  // Indicate that optional altitude assistance lost trustworthy height control.
};                                       // Finish the failsafe reason enumeration.

extern const float CAL_FACTOR;                  // Expose the tested ESP supply-voltage calibration factor.
extern const float FAILSAFE_VOLTAGE;            // Expose the tested ESP supply-voltage trigger threshold.
extern const float FAILSAFE_DESCEND_RATE_HIGH;  // Expose the first-stage landing reduction rate in legacy throttle-command units per second.
extern const float FAILSAFE_DESCEND_RATE_LOW;   // Expose the slower final-stage landing reduction rate in legacy throttle-command units per second.
extern bool failsafe_active;                    // Expose whether any failsafe reason is currently active.
extern uint8_t failsafe_reason;                 // Expose the currently latched failsafe reason bit mask.

void failsafeInit();                                                            // Reset all failsafe runtime state and force safe motor outputs.
void failsafeNotifyValidControlPacket(uint32_t nowMs, int16_t packetThrottle);  // Record one complete validated control packet and its exact throttle command.
void failsafeNotifyControlDisconnect();                                         // Mark the active control client as disconnected.
void failsafeNotifyEffectiveThrottle(int16_t effectiveThrottle);                    // Refresh the last trustworthy actual mixer throttle during healthy assisted flight.
void failsafeTriggerAltitudeAssist(uint32_t nowMs, int16_t effectiveThrottle);       // Start the existing gradual landing from the actual throttle after an altitude-assist fault.
void failsafeStartCommandedLanding(uint32_t nowMs, int16_t effectiveThrottle);     // Start the same deterministic descent for an explicit Landing command without latching a fault reason.
void failsafeUpdate(uint32_t nowMs);                                            // Update link monitoring, voltage monitoring, landing, alarms, and outputs.
void failsafeLanding();                                                         // Preserve compatibility with the existing flight-control call site.
bool failsafeCanArm(uint32_t nowMs);                                            // Report whether all failsafe pre-arm requirements are satisfied.
bool failsafeLinkHealthy(uint32_t nowMs);                                       // Report whether the control link is currently fresh and valid.
bool failsafeLandingActive();                                                   // Report whether any gradual failsafe landing ramp is still running.
bool failsafeBatteryLockoutActive();                                            // Report whether low battery must block software restart and re-arming.

#endif  // Close the FAILSAFE_H include guard.

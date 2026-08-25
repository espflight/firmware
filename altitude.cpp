#include "altitude.h"      // Provide the public VL53L0X and assisted-flight state.
#include "flight_control.h"// Read armed state, receiver throttle, and effective mixer throttle.
#include "pid.h"           // Control the preserved altitude PID runtime state.
#include "failsafe.h"      // Hand sensor/height faults to the existing deterministic landing controller.

// Altitude Kalman filter state and implementation are kept inside the altitude module because no other firmware module owns or consumes this filter.
float Q = 0.3f;  // Preserve the process noise from the previously flight-tested altitude firmware.
float R = 9.0f;  // Preserve the measurement noise from the previously flight-tested altitude firmware.
float P = 1.0f;  // Start each filter session with the original estimate covariance.
float X = 0.0f;  // Start without a committed altitude estimate until the first valid range sample arrives.
float K = 0.0f;  // Start the Kalman gain at zero before the first update.

float kalmanUpdate(float measurement) {  // Apply the exact scalar Kalman update used by the previous working altitude implementation.
  P = P + Q;                             // Increase uncertainty by the preserved process-noise amount.
  K = P / (P + R);                       // Calculate the measurement weighting from the current uncertainty and preserved measurement noise.
  X = X + K * (measurement - X);         // Move the state estimate toward the newest retained VL53L0X measurement.
  P = (1.0f - K) * P;                    // Reduce uncertainty after incorporating the measurement.
  return X;                              // Return the newest smoothed altitude estimate in millimeters.
}

void kalmanReset(float initialValue) {  // Reset the filter without changing the tested Q/R tuning values.
  P = 1.0f;                             // Restore the original covariance used by the working firmware.
  X = initialValue;                     // Start from the supplied real range value or zero when no valid sample is available yet.
  K = 0.0f;                             // Clear the previous Kalman gain.
}

Adafruit_VL53L0X lox = Adafruit_VL53L0X();  // Define the single optional time-of-flight sensor object.
unsigned long lastSensorRead = 0U;           // Start physical ranging timing from boot.
const unsigned long sensorInterval = 50U;    // Preserve the previously tested 20-Hz VL53L0X physical read interval.
uint16_t lastDistance = 0U;                   // Start without a previous accepted measurement.
uint16_t distance = 0U;                       // Start without a current accepted measurement.
float filtered_distance = 0.0f;              // Start without a filtered height estimate.
bool sensorTimeout = true;                    // Treat the sensor as not ready until fresh valid measurements arrive.
bool altHoldEnabled = false;                  // Start with altitude assistance disabled.
bool alt_flag = false;                        // Start with altitude PID disconnected from throttle.
bool vl53_available = false;                  // Start assuming the optional sensor is absent until boot detection succeeds.

namespace {
constexpr uint16_t kMinimumValidDistanceMm = 30U;      // Preserve the lower validity boundary used by the previous working firmware.
constexpr uint16_t kMaximumValidDistanceMm = 1000U;    // Preserve the upper validity boundary used by the previous working firmware.
constexpr uint16_t kHoldActivationHeightMm = 200U;     // Preserve the previous real-lift threshold before altitude PID becomes active.
constexpr int16_t kTakeoffActivationThrottle = 1500;   // Match the previous app flow exactly: altitude PID takes ownership only after the visible throttle ramp has reached 1500.
constexpr float kAssistHeightGuardMm = 850.0f;         // Stop assisted upward flight well before the VL53L0X validity ceiling if height control ever runs away.
constexpr uint32_t kSensorFreshTimeoutMs = 300U;       // Allow several missed 50-ms measurements before declaring an active-flight sensor fault.
constexpr uint8_t kReadySamplesRequired = 3U;          // Require three independent valid physical measurements before Takeoff can be accepted.
constexpr uint32_t kTakeoffLiftTimeoutMs = 5000U;      // Abort assisted Takeoff if the aircraft still has not reached the real-lift threshold after five seconds.
constexpr uint16_t kLandingTargetFloorMm = 50U;           // Preserve the old minimum landing setpoint while keeping it inside the sensor's usable floor range.
constexpr uint16_t kLandingImmediateDisarmHeightMm = 50U;  // Cut all motors once Landing reaches five centimeters above the recorded pre-takeoff floor reference.
constexpr uint8_t kLandingImmediateDisarmSamplesRequired = 2U; // Require two independent physical range samples so one isolated ToF glitch cannot stop the motors early.
constexpr uint16_t kLandingGroundMarginMm = 45U;          // Treat the aircraft as essentially on the floor only after it returns close to the measured pre-takeoff height.
constexpr uint16_t kLandingMinimumTouchdownMm = 75U;      // Never use an unrealistically tiny touchdown threshold even when the sensor sits very close to the frame bottom.
constexpr uint16_t kLandingMaximumTouchdownMm = 160U;     // Prevent a noisy or unusually high pre-takeoff reading from declaring touchdown too far above the floor.
constexpr int16_t kLandingTouchdownThrottleMax = 1125;    // Require the app landing ramp to be near its low end before sensor proximity can confirm touchdown.
constexpr uint8_t kLandingTouchdownSamplesRequired = 4U;  // Require four independent physical VL53L0X samples near the floor before allowing automatic motor shutdown.
constexpr uint32_t kLandingTimeoutMs = 12000U;            // Hand an assisted landing to the normal failsafe descent if it cannot reach the floor in a reasonable time.

AltitudeAssistMode gAssistMode = ALTITUDE_ASSIST_OFF;  // Track the high-level app-requested assisted-flight phase.
uint32_t gLastValidRangeMs = 0U;                       // Store the time of the newest accepted physical range sample.
uint8_t gConsecutiveValidSamples = 0U;                 // Count consecutive independent valid measurements until initial Takeoff readiness is established.
bool gTakeoffSensorReadyLatched = false;                // Keep Takeoff readiness stable across isolated bad VL53L0X samples and clear it only after a real freshness timeout.
bool gFaultLandingRequested = false;                   // Prevent repeatedly retriggering the same sensor/height safety landing.
uint32_t gTakeoffStartedMs = 0U;                        // Remember when the board accepted Takeoff so failed lift cannot run the motors indefinitely.
uint32_t gLandingStartedMs = 0U;                           // Remember when Landing began so a blocked or abnormal descent cannot continue forever.
float gGroundReferenceMm = 0.0f;                           // Preserve the filtered pre-takeoff floor distance for robust touchdown detection.
uint32_t gPhysicalRangeSampleCounter = 0U;                 // Count accepted physical range samples so touchdown confirmation never reuses one sample at 250 Hz.
uint32_t gLandingLastCheckedSampleCounter = 0U;            // Remember which physical sample was last considered by touchdown detection.
uint8_t gLandingTouchdownSampleCount = 0U;                 // Count consecutive independent near-floor measurements during Landing.
bool gLandingTouchdownConfirmed = false;                   // Latch true only after the aircraft is physically close to the recorded floor reference.
uint8_t gLandingImmediateDisarmSampleCount = 0U;            // Count independent samples at or below the five-centimeter emergency landing cutoff.

void requestAltitudeSafetyLanding(unsigned long nowMs) {  // Transfer control to the existing failsafe landing if assisted height becomes untrustworthy.
  if (gFaultLandingRequested || start != 2U) return;       // Trigger only once while the aircraft is actually armed.
  gFaultLandingRequested = true;                           // Latch this transfer so later 250-Hz cycles cannot restart the landing.
  alt_flag = false;                                        // Disconnect stale altitude PID correction immediately.
  altHoldEnabled = false;                                  // Prevent the normal hold code from re-entering after the safety transfer.
  resetAltitudePidRuntime();                               // Remove every stale altitude PID term before failsafe owns throttle.
  failsafeTriggerAltitudeAssist(nowMs, static_cast<int16_t>(constrain(throttle, 1050, 1900)));  // Begin descent from the actual effective mixer throttle.
}
}  // Close private altitude implementation details.

bool altitudeInit() {                          // Detect and initialize the optional sensor without making it mandatory for manual flight.
  Serial.println("Checking VL53L0X...");       // Announce optional altitude-sensor detection.
  if (!lox.begin(0x29, false, &Wire)) {        // Try the same default I2C address and bus used by the previously working firmware.
    Serial.println(F("VL53L0X NOT detected - manual flight remains available"));  // Report optional-sensor absence without blocking firmware setup.
    vl53_available = false;                    // Keep all assisted commands unavailable.
    sensorTimeout = true;                      // Keep sensor readiness false.
    return false;                              // Report that only manual flight is available.
  }
  lox.startRangeContinuous();                  // Preserve continuous ranging mode from the working implementation.
  vl53_available = true;                       // Publish sensor availability to network validation and telemetry.
  sensorTimeout = true;                        // Wait for real valid samples before declaring readiness.
  lastSensorRead = 0U;                         // Allow the first scheduled range check immediately.
  gLastValidRangeMs = 0U;                      // Require fresh post-initialization data.
  gConsecutiveValidSamples = 0U;               // Require the configured number of independent samples.
  gTakeoffSensorReadyLatched = false;            // Require a fresh three-sample readiness sequence after sensor initialization.
  distance = 0U;                               // Clear any stale raw distance.
  lastDistance = 0U;                           // Clear previous raw distance.
  filtered_distance = 0.0f;                    // Clear previous filtered height.
  kalmanReset(0.0f);                           // Reset the preserved filter around the first real range value.
  Serial.println(F("VL53L0X OK"));            // Confirm optional altitude assistance is available.
  return true;                                 // Report successful optional-sensor setup.
}

void readAltitude(unsigned long nowMs) {                       // Preserve old range scheduling and filter update frequency semantics.
  if (!vl53_available) return;                                 // Do nothing when the optional sensor was not detected.
  if (static_cast<uint32_t>(nowMs - lastSensorRead) >= sensorInterval) {  // Poll for one new physical measurement every 50 milliseconds.
    lastSensorRead = nowMs;                                    // Start the next physical-read interval from this attempt.
    if (lox.isRangeComplete()) {                               // Read only after the continuous-ranging driver reports a complete measurement.
      const int newDistance = static_cast<int>(lox.readRange());  // Read the newest physical distance in millimeters.
      if (newDistance > static_cast<int>(kMinimumValidDistanceMm) && newDistance < static_cast<int>(kMaximumValidDistanceMm)) {  // Accept the same useful range as the old firmware.
        lastDistance = distance;                               // Preserve the previous accepted raw distance for diagnostics.
        distance = static_cast<uint16_t>(newDistance);         // Publish the newest valid physical measurement.
        sensorTimeout = false;                                 // Mark the sensor healthy after this accepted measurement.
        gLastValidRangeMs = nowMs;                             // Record freshness using this independent physical sample.
        if (gConsecutiveValidSamples < 255U) gConsecutiveValidSamples++;  // Count consecutive independent valid readings without overflow.
        if (gConsecutiveValidSamples >= kReadySamplesRequired) gTakeoffSensorReadyLatched = true;  // Latch readiness after the initial stable three-sample sequence.
        gPhysicalRangeSampleCounter++;                      // Advance one counter only for a genuinely new accepted physical VL53L0X sample.
        if (X == 0.0f) X = static_cast<float>(distance);       // Preserve the previous implementation's first-range filter initialization.
      } else {                                                 // Reject an isolated out-of-range physical result without immediately making a healthy sensor unusable for Takeoff.
        if (!gTakeoffSensorReadyLatched) gConsecutiveValidSamples = 0U;  // Before readiness is established, still require three consecutive valid samples.
      }
    }
  }

  if (distance > 0U) {                                        // Run the Kalman update whenever one valid retained distance exists.
    filtered_distance = kalmanUpdate(static_cast<float>(distance));  // Preserve the previous behavior: repeatedly filter the retained distance between 20-Hz physical reads.
  }
  if (gLastValidRangeMs == 0U || static_cast<uint32_t>(nowMs - gLastValidRangeMs) > kSensorFreshTimeoutMs) {  // Detect stale ranging even when no invalid packet explicitly arrived.
    sensorTimeout = true;                                      // Publish stale sensor state to assisted-flight safety and telemetry.
    gConsecutiveValidSamples = 0U;                             // Require fresh consecutive samples before a later Takeoff request.
    gTakeoffSensorReadyLatched = false;                        // Clear readiness only after a real freshness timeout, not after one noisy measurement.
  }
}

bool altitudeSensorHealthy(unsigned long nowMs) {  // Judge active-flight sensor health from true freshness instead of one isolated invalid physical result.
  return vl53_available && gLastValidRangeMs != 0U && static_cast<uint32_t>(nowMs - gLastValidRangeMs) <= kSensorFreshTimeoutMs;
}

bool altitudeSensorReady(unsigned long nowMs) {  // Report stable Takeoff readiness after the initial sample qualification while still requiring genuinely fresh range data.
  return altitudeSensorHealthy(nowMs) && !sensorTimeout && gTakeoffSensorReadyLatched;
}

bool altitudeRequestTakeoff(unsigned long nowMs) {  // Prepare exactly the old two-stage process while leaving the initial throttle ramp to the app.
  if (start != 2U || failsafe_active) return false;                                                // Require a genuinely armed, non-failsafe aircraft before any assisted upward request.
  if (gAssistMode == ALTITUDE_ASSIST_TAKEOFF || gAssistMode == ALTITUDE_ASSIST_HOLD) return true;  // Make repeated Takeoff requests idempotent before transient sensor/throttle values are rechecked.
  if (!altitudeSensorReady(nowMs) || channel_3 > 1050) return false;                               // Validate fresh altitude data and minimum real throttle only for a genuinely new Takeoff request.
  if (gAssistMode != ALTITUDE_ASSIST_OFF) return false;                                            // Do not restart Takeoff during an existing Landing.
  altHoldEnabled = true;                           // Allow the old height threshold to arm altitude PID later.
  alt_flag = false;                                // Keep altitude PID completely disconnected during the initial app throttle ramp.
  gAssistMode = ALTITUDE_ASSIST_TAKEOFF;           // Publish the first assisted-flight phase.
  gFaultLandingRequested = false;                  // Clear any stale fault-transfer marker from an earlier disarmed session.
  gTakeoffStartedMs = nowMs;                       // Start the no-lift timeout only after the board has accepted this Takeoff request.
  gLandingStartedMs = 0U;                           // Clear any landing timeout left from an earlier assisted sequence.
  gGroundReferenceMm = static_cast<float>(distance); // Capture the newest raw physical floor distance before lift so low-altitude Landing thresholds cannot be biased by Kalman startup lag.
  gLandingLastCheckedSampleCounter = gPhysicalRangeSampleCounter;  // Start touchdown sample tracking from the next new physical reading.
  gLandingTouchdownSampleCount = 0U;                // Clear any stale near-floor sample count.
  gLandingTouchdownConfirmed = false;               // Require a new physical touchdown confirmation for this flight.
  gLandingImmediateDisarmSampleCount = 0U;          // Require fresh five-centimeter cutoff samples for this assisted flight.
  resetAltitudePidRuntime();                        // Start altitude PID from clean runtime state exactly as the old flow expected.
  return true;                                     // Acknowledge that the app may now ramp throttle toward 1500.
}

bool altitudeRequestLanding(unsigned long nowMs) {  // Allow the app to ramp throttle/setpoint downward only from an active Takeoff/Hold session.
  if (start != 2U || !altitudeSensorHealthy(nowMs) || failsafe_active) return false;  // Accept Landing across isolated bad samples, but never with truly stale range data or active failsafe.
  if (gAssistMode == ALTITUDE_ASSIST_LANDING) return true;                           // Make repeated Landing requests idempotent.
  if (gAssistMode != ALTITUDE_ASSIST_HOLD && gAssistMode != ALTITUDE_ASSIST_TAKEOFF) return false;  // Reject Landing when no assisted flight is active.
  gAssistMode = ALTITUDE_ASSIST_LANDING;               // Tell the control loop that the app will now lower channel_3 from 1500 toward 1050.
  altHoldEnabled = true;                               // Keep altitude assistance enabled throughout the sensor-guided descent.
  gLandingStartedMs = nowMs;                           // Start a bounded assisted-landing window.
  gLandingLastCheckedSampleCounter = gPhysicalRangeSampleCounter;  // Count only future independent physical samples toward touchdown.
  gLandingTouchdownSampleCount = 0U;                   // Require a fresh sequence of near-floor readings.
  gLandingTouchdownConfirmed = false;                  // Do not allow disarm until VL53L0X confirms the aircraft is actually back at the floor.
  gLandingImmediateDisarmSampleCount = 0U;             // Start the five-centimeter motor-cut confirmation from the next physical sample.
  return true;                                         // Acknowledge that the application may begin its controlled landing throttle ramp.
}

void altitudeUpdateControlState(unsigned long nowMs) {  // Apply only the state transitions needed around the old proven throttle-plus-altitude-PID relationship.
  if (start != 2U) {                                    // Clear assist state whenever the aircraft is no longer armed.
    if (gAssistMode != ALTITUDE_ASSIST_OFF || alt_flag || altHoldEnabled) altitudeResetAssist();  // Remove stale assisted state after any disarm path.
    return;                                             // No assisted logic is allowed while fully disarmed.
  }
  if (failsafe_active || failsafeLandingActive()) {     // Existing link/battery failsafe always has higher authority than optional altitude assistance.
    altitudeResetAssist();                              // Disconnect altitude PID while the failsafe landing owns throttle.
    return;                                             // Leave the existing failsafe path untouched.
  }
  if (gAssistMode == ALTITUDE_ASSIST_OFF) return;       // Preserve ordinary manual flight when no high-level assist command is active.

  if (!altitudeSensorHealthy(nowMs)) {                  // Allow brief isolated bad readings, but never continue after the real 300-ms freshness timeout.
    requestAltitudeSafetyLanding(nowMs);                // Transfer to the deterministic failsafe descent from the current effective throttle.
    return;                                             // Do not calculate or retain an altitude correction after sensor loss.
  }

  if (filtered_distance >= kAssistHeightGuardMm) {      // Catch any runaway assisted climb independently from PID tuning.
    requestAltitudeSafetyLanding(nowMs);                // Descend before reaching the upper validity edge or an indoor ceiling.
    return;                                             // Prevent another upward assisted throttle calculation this cycle.
  }

  if (gAssistMode == ALTITUDE_ASSIST_TAKEOFF) {         // Stage one: the application is moving the visible throttle stick from minimum to 1500.
    alt_flag = false;                                   // Keep altitude PID entirely out of the initial raw-throttle lift phase.
    pid_alt_setpoint = 0.0f;                            // Keep the altitude target inactive before real lift is confirmed.
    if (static_cast<uint32_t>(nowMs - gTakeoffStartedMs) > kTakeoffLiftTimeoutMs) {  // Prevent a damaged or restrained aircraft from sitting at the Takeoff base throttle forever.
      requestAltitudeSafetyLanding(nowMs);              // Hand the no-lift condition to the existing deterministic failsafe descent/disarm path.
      return;                                           // Never continue raw assisted Takeoff after the timeout expires.
    }
    if (filtered_distance >= static_cast<float>(kHoldActivationHeightMm) && channel_3 >= kTakeoffActivationThrottle) {  // Wait until the aircraft is actually airborne and the app has reached the old 1500 base throttle.
      alt_flag = true;                                  // Connect the old altitude PID correction only after real lift.
      pid_alt_setpoint = static_cast<float>(channel_3 - 1000);  // Restore the exact old relationship: channel_3=1500 creates the 500-mm altitude target.
      pid_i_mem_alt = 0.0f;                              // Start Hold without carrying any integral term from the raw-throttle stage.
      pid_last_alt_d_error = pid_alt_setpoint - filtered_distance;  // Prime derivative history to the current altitude error so PID engagement cannot create a one-cycle D-kick to maximum throttle.
      pid_output_alt = 0.0f;                             // Let the next normal PID calculation build only the required current correction.
      gAssistMode = ALTITUDE_ASSIST_HOLD;               // Publish the stable altitude-hold phase.
      gTakeoffStartedMs = 0U;                            // Clear the no-lift timeout after successful real-airborne transition into Hold.
    }
    return;                                             // Finish the Takeoff stage without altering raw throttle otherwise.
  }

  if (gAssistMode == ALTITUDE_ASSIST_HOLD) {            // Stage two: app throttle remains fixed at 1500 while firmware PID controls actual motor throttle.
    alt_flag = true;                                    // Keep altitude PID connected.
    pid_alt_setpoint = static_cast<float>(channel_3 - 1000);  // Preserve the old live relationship; the app lock at 1500 therefore holds a 500-mm target.
    return;                                             // Leave throttle correction to the old PID calculation.
  }

  if (gAssistMode == ALTITUDE_ASSIST_LANDING) {         // Stage three: app ramps throttle downward while altitude PID guides the aircraft all the way back near the recorded floor.
    if (static_cast<uint32_t>(nowMs - gLandingStartedMs) > kLandingTimeoutMs) {  // Never let a blocked or abnormal assisted landing continue indefinitely.
      requestAltitudeSafetyLanding(nowMs);              // Transfer the remaining descent to the existing deterministic failsafe path.
      return;                                            // Stop normal assisted landing control after the timeout transfer.
    }

    alt_flag = !gLandingTouchdownConfirmed;              // Keep altitude PID active until physical touchdown has been independently confirmed.
    if (alt_flag) {                                      // Shape descent using the same tested altitude PID rather than cutting it at the old 200-mm activation threshold.
      int32_t landingTargetMm = static_cast<int32_t>(channel_3) - 1000;  // Preserve the original throttle-to-altitude relationship while the app performs its visible landing ramp.
      if (landingTargetMm < static_cast<int32_t>(kLandingTargetFloorMm)) landingTargetMm = kLandingTargetFloorMm;  // Keep the target inside the useful low-altitude region.
      if (landingTargetMm > 500) landingTargetMm = 500;                  // Never raise the landing target above the normal 500-mm hold point.

      const float newTargetMm = static_cast<float>(landingTargetMm);     // Convert the bounded target once for PID state updates.
      const float targetDeltaMm = newTargetMm - pid_alt_setpoint;        // Measure only the commanded setpoint movement since the last control cycle.
      pid_last_alt_d_error += targetDeltaMm;                              // Cancel derivative kick from setpoint motion while preserving derivative response to real altitude movement.
      pid_alt_setpoint = newTargetMm;                                     // Apply the new descending target after derivative-history compensation.
    }

    float touchdownThresholdMm = gGroundReferenceMm + static_cast<float>(kLandingGroundMarginMm);  // Build touchdown threshold from the actual pre-takeoff sensor-to-floor distance.
    if (touchdownThresholdMm < static_cast<float>(kLandingMinimumTouchdownMm)) touchdownThresholdMm = static_cast<float>(kLandingMinimumTouchdownMm);  // Keep the threshold realistic for low-mounted sensors.
    if (touchdownThresholdMm > static_cast<float>(kLandingMaximumTouchdownMm)) touchdownThresholdMm = static_cast<float>(kLandingMaximumTouchdownMm);  // Reject an excessively high threshold caused by a noisy ground reference.

    if (gPhysicalRangeSampleCounter != gLandingLastCheckedSampleCounter) {  // Evaluate landing proximity only once per newly accepted physical range measurement.
      gLandingLastCheckedSampleCounter = gPhysicalRangeSampleCounter;       // Mark this physical sample as consumed by landing proximity detection.

      const bool atImmediateDisarmHeight = distance > 0U && distance <= kLandingImmediateDisarmHeightMm;                  // Follow the requested rule literally: a fresh raw VL53L0X reading at or below 50 mm means the aircraft is within five centimeters of the floor.
      if (atImmediateDisarmHeight) {                                                                                       // Count only independent physical samples inside the five-centimeter cutoff region.
        if (gLandingImmediateDisarmSampleCount < 255U) gLandingImmediateDisarmSampleCount++;                              // Increase the cutoff confirmation count without overflow.
      } else {                                                                                                             // Any sample clearly above five centimeters breaks the cutoff confirmation sequence.
        gLandingImmediateDisarmSampleCount = 0U;                                                                           // Require a fresh two-sample sequence before emergency motor shutdown.
      }                                                                                                                    // Finish updating the five-centimeter cutoff confirmation.
      if (gLandingImmediateDisarmSampleCount >= kLandingImmediateDisarmSamplesRequired) {                                  // Stop immediately after two real VL53L0X samples confirm the aircraft is within five centimeters of the floor.
        resetAltitudePidRuntime();                                                                                          // Remove every altitude PID term before motor power is cut.
        altitudeResetAssist();                                                                                              // Clear assisted-flight state so no later control path can re-enable altitude corrections.
        flightControlEmergencyStop();                                                                                       // Disarm and write zero PWM to all four motors in this same control cycle.
        return;                                                                                                             // Do not execute any remaining landing PID or touchdown logic after the hard cutoff.
      }                                                                                                                    // Finish the immediate five-centimeter landing shutdown.

      const bool nearFloor = filtered_distance <= touchdownThresholdMm;     // Preserve the older near-floor touchdown logic only as a secondary fallback below the new five-centimeter cutoff.
      const bool throttleNearlyLow = channel_3 <= kLandingTouchdownThrottleMax;  // Also require the app ramp to be near its low-throttle phase.
      if (nearFloor && throttleNearlyLow) {                                  // Count only independent samples satisfying both physical and command conditions.
        if (gLandingTouchdownSampleCount < 255U) gLandingTouchdownSampleCount++;  // Increase the confirmation count without overflow.
      } else {                                                               // Any clearly non-touchdown sample breaks the confirmation sequence.
        gLandingTouchdownSampleCount = 0U;                                    // Require a fresh consecutive sequence before shutdown.
      }
      if (gLandingTouchdownSampleCount >= kLandingTouchdownSamplesRequired) { // Confirm touchdown only after several real VL53L0X measurements.
        gLandingTouchdownConfirmed = true;                                    // Latch physical touchdown for the final low-throttle shutdown phase.
        alt_flag = false;                                                      // Disconnect altitude PID only after the aircraft is genuinely near the floor.
        resetAltitudePidRuntime();                                             // Remove residual altitude correction before final motor stop.
      }
    }

    if (channel_3 <= 1050) {                           // Never allow a completed application Landing ramp to leave the motors idling indefinitely at 1050 if the five-centimeter range cutoff was missed.
      resetAltitudePidRuntime();                       // Remove every altitude PID term before the motor-off transition.
      altitudeResetAssist();                           // Clear assisted-flight ownership so no later control path can restore powered throttle.
      flightControlEmergencyStop();                    // Disarm and force PWM on all four motors to zero immediately.
      return;                                          // Stop this control update before any later mixer logic can write 1050 again.
    }
  }
}

void altitudeResetAssist() {          // Return assisted flight to a clean manual/disarmed baseline.
  altHoldEnabled = false;             // Require a new explicit Takeoff command before altitude PID can re-enter.
  alt_flag = false;                    // Disconnect altitude PID from throttle.
  gAssistMode = ALTITUDE_ASSIST_OFF;   // Publish manual/off assisted state.
  gFaultLandingRequested = false;      // Clear the one-shot fault transfer only after assist itself is reset.
  gTakeoffStartedMs = 0U;                // Clear any unfinished Takeoff timeout from the previous assisted session.
  gLandingStartedMs = 0U;                // Clear the assisted-landing timeout state.
  gGroundReferenceMm = 0.0f;              // Require a fresh pre-takeoff floor reference on the next assisted flight.
  gLandingLastCheckedSampleCounter = gPhysicalRangeSampleCounter;  // Prevent stale physical samples from counting in a future landing.
  gLandingTouchdownSampleCount = 0U;       // Clear any old touchdown confirmation sequence.
  gLandingTouchdownConfirmed = false;      // Clear the touchdown latch for the next assisted flight.
  gLandingImmediateDisarmSampleCount = 0U; // Clear the five-centimeter hard-cut confirmation state.
  resetAltitudePidRuntime();               // Remove stale altitude PID state from the next session.
}

bool altitudePidActive() {  // Tell the flight controller whether the legacy altitude correction must be added around channel_3.
  return alt_flag && altHoldEnabled && (gAssistMode == ALTITUDE_ASSIST_HOLD || gAssistMode == ALTITUDE_ASSIST_LANDING);
}

bool altitudeLandingShouldDisarm() {  // Finish assisted Landing only after the app reaches minimum throttle and VL53L0X has independently confirmed the aircraft is physically back at the floor.
  return gAssistMode == ALTITUDE_ASSIST_LANDING && start == 2U && gLandingTouchdownConfirmed && channel_3 <= 1050;
}

AltitudeAssistMode altitudeAssistMode() {  // Publish the current assisted phase without exposing private mutable state.
  return gAssistMode;
}

const char *altitudeAssistModeName() {  // Return one stable telemetry label for the application and diagnostics.
  switch (gAssistMode) {
    case ALTITUDE_ASSIST_TAKEOFF: return "takeoff";
    case ALTITUDE_ASSIST_HOLD: return "hold";
    case ALTITUDE_ASSIST_LANDING: return "landing";
    default: return "off";
  }
}

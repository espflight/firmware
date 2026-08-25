#include <Arduino.h>         // Provide Arduino timing, GPIO, PWM, and ESP helper functions.
#include <math.h>            // Provide the finite-value validation function used for voltage samples.
#include "failsafe.h"        // Provide the public failsafe API and reason definitions.
#include "indicators.h"      // Provide the preserved buzzer pin used for non-blocking failsafe warnings.
#include "flight_control.h"  // Provide flight state, receiver channels, motor pins, and flight timer functions.
#include "pid.h"             // Provide PID runtime variables that must be reset during safety transitions.

const float CAL_FACTOR = 1.082f;                  // Preserve the voltage calibration factor validated on the tested hardware.
const float FAILSAFE_VOLTAGE = 2.6f;              // Preserve the tested regulated-supply voltage that indicates an exhausted battery.
const float FAILSAFE_DESCEND_RATE_HIGH = 120.0f;  // Reduce throttle at the normal failsafe descent rate above the soft-landing threshold.
const float FAILSAFE_DESCEND_RATE_LOW = 60.0f;    // Reduce throttle much more slowly during the final soft-landing stage.
bool failsafe_active = false;                     // Start with no active failsafe reason.
uint8_t failsafe_reason = FAILSAFE_REASON_NONE;   // Start with an empty failsafe reason bit mask.

namespace {  // Keep all internal failsafe implementation details private to this file.

constexpr uint32_t kControlTimeoutMs = 300U;         // Declare link loss after six missed 50-millisecond control packets.
constexpr uint32_t kExpectedControlPeriodMs = 50U;   // Preserve the last valid throttle if receiver values are reset after one expected packet interval.
constexpr uint32_t kVoltageSampleIntervalMs = 100U;  // Measure the tested ESP supply voltage ten times per second.
constexpr uint8_t kVoltageSamplesPerReading = 5U;    // Average five immediate VCC samples for each voltage update.
constexpr uint8_t kLowVoltageConfirmSamples = 3U;    // Require three consecutive low readings before battery landing starts.
constexpr float kMinimumPlausibleVoltage = 0.5f;     // Reject only near-zero readings while still treating severe voltage collapse as critical.
constexpr float kMaximumPlausibleVoltage = 5.0f;     // Reject impossible high readings that indicate a measurement failure.
constexpr int16_t kReceiverMinimum = 1000;           // Define the minimum valid receiver command.
constexpr int16_t kReceiverNeutral = 1500;           // Define the neutral roll, pitch, and yaw command.
constexpr int16_t kLandingMinimumThrottle = 1050;    // Define the minimum powered throttle used during the landing delay.
constexpr int16_t kSoftLandingThrottle = 1200;       // Begin the slower final descent stage once failsafe throttle reaches this value.
constexpr int16_t kLandingMaximumThrottle = 1900;    // Limit the captured landing throttle to the normal flight maximum.
constexpr uint32_t kLandingStopDelayMs = 1000U;      // Keep minimum landing throttle for one seconds before motor shutdown.
constexpr uint32_t kMaximumLandingStepMs = 100U;     // Limit one landing update step after an unexpected scheduling delay.
constexpr uint32_t kBuzzerIntervalMs = 500U;         // Emit one warning beep every half second while failsafe is active.
constexpr uint16_t kBuzzerFrequencyHz = 4000U;       // Use the existing four-kilohertz failsafe warning tone.
constexpr uint16_t kBuzzerDurationMs = 100U;         // Keep each warning tone short and non-blocking.

volatile uint32_t gLastValidControlMs = 0U;         // Store the arrival time of the newest complete valid control packet.
volatile bool gControlPacketReceived = false;       // Remember whether at least one complete valid control packet arrived.
volatile bool gControlDisconnectRequested = false;  // Remember an explicit disconnect from the active control client.

uint32_t gLastVoltageSampleMs = 0U;   // Store the time of the newest accepted battery-related voltage sample.
uint8_t gLowVoltageSampleCount = 0U;  // Count consecutive accepted voltage samples at or below the trigger.
bool gBatteryVoltageValid = false;    // Remember whether at least one plausible voltage sample has been accepted.

bool gLinkFailsafeLatched = false;                              // Keep link-loss landing active until deliberate safe recovery completes.
bool gBatteryFailsafeLatched = false;                           // Keep low-battery failsafe active until the controller is restarted.
bool gFailsafeLandingRunning = false;                           // Indicate that the shared time-based failsafe landing ramp is running.
bool gFailsafeLandingComplete = false;                          // Indicate that the shared failsafe landing has reached final motor shutdown.
float gLandingThrottle = static_cast<float>(kReceiverMinimum);  // Store the landing throttle independently from loop rate.
uint32_t gLastLandingUpdateMs = 0U;                             // Store the time used to calculate the next time-based throttle reduction.
uint32_t gMinimumThrottleReachedMs = 0U;                        // Store the exact time at which landing throttle first reached its minimum.
bool gMinimumThrottleReached = false;                           // Remember whether the minimum landing throttle timestamp is valid.
int16_t gLastHealthyFlightThrottle = kReceiverMinimum;          // Preserve the last trustworthy in-flight throttle before control packets disappear.
bool gLastHealthyFlightThrottleValid = false;                   // Remember whether a trustworthy in-flight throttle snapshot has been captured.
bool gExternalLandingThrottleValid = false;                        // Remember whether an external safety subsystem supplied the exact current mixer throttle.
int16_t gExternalLandingThrottle = kReceiverMinimum;                 // Store the supplied actual throttle for the next shared failsafe landing entry.

void resetPidRuntimeState() {     // Reset every attitude PID memory and output so no stale correction survives a full shutdown.
  pid_i_mem_roll = 0.0f;          // Clear the roll integral accumulator.
  pid_last_roll_d_error = 0.0f;   // Clear the previous roll derivative error.
  pid_output_roll = 0.0f;         // Clear the current roll PID output.
  pid_i_mem_pitch = 0.0f;         // Clear the pitch integral accumulator.
  pid_last_pitch_d_error = 0.0f;  // Clear the previous pitch derivative error.
  pid_output_pitch = 0.0f;        // Clear the current pitch PID output.
  pid_i_mem_yaw = 0.0f;           // Clear the yaw integral accumulator.
  pid_last_yaw_d_error = 0.0f;    // Clear the previous yaw derivative error.
  pid_output_yaw = 0.0f;          // Clear the current yaw PID output.
}  // Finish resetting all PID runtime state.

void forceSafeReceiverState() {    // Replace every stale receiver value with a deterministic safe value.
  rc_throttle = kReceiverMinimum;  // Force the stored throttle command to its minimum.
  rc_roll = kReceiverNeutral;      // Force the stored roll command to neutral.
  rc_pitch = kReceiverNeutral;     // Force the stored pitch command to neutral.
  rc_yaw = kReceiverNeutral;       // Force the stored yaw command to neutral.
  channel_1 = kReceiverNeutral;    // Force the active roll channel to neutral.
  channel_2 = kReceiverNeutral;    // Force the active transformed pitch channel to neutral.
  channel_3 = kReceiverMinimum;    // Force the active throttle channel to its minimum.
  channel_4 = kReceiverNeutral;    // Force the active yaw channel to neutral.
  throttle = kReceiverMinimum;     // Force the mixer throttle input to its minimum.
}  // Finish forcing safe receiver values.

void neutralizeReceiverAxesForFailsafeLanding() {  // Remove all directional commands without disturbing the active landing throttle.
  rc_roll = kReceiverNeutral;                       // Force the stored roll command to neutral during failsafe landing.
  rc_pitch = kReceiverNeutral;                      // Force the stored pitch command to neutral during failsafe landing.
  rc_yaw = kReceiverNeutral;                        // Force the stored yaw command to neutral during failsafe landing.
  channel_1 = kReceiverNeutral;                     // Force the active roll channel to neutral during failsafe landing.
  channel_2 = kReceiverNeutral;                     // Force the active transformed pitch channel to neutral during failsafe landing.
  channel_4 = kReceiverNeutral;                     // Force the active yaw channel to neutral during failsafe landing.
}  // Finish neutralizing directional receiver commands for failsafe landing.

bool isControlLinkHealthy(uint32_t nowMs);  // Forward-declare link health for the healthy-throttle snapshot helper.

void publishLandingThrottle(int16_t landingThrottle) {                                                        // Apply the failsafe throttle to every receiver and mixer path that may otherwise overwrite it.
  const int16_t safeThrottle = constrain(landingThrottle, kLandingMinimumThrottle, kLandingMaximumThrottle);  // Keep the landing command inside the powered flight range.
  rc_throttle = safeThrottle;                                                                                 // Override the raw application throttle with the failsafe landing command.
  channel_3 = safeThrottle;                                                                                   // Override the active receiver throttle channel with the failsafe landing command.
  throttle = safeThrottle;                                                                                    // Override the motor mixer base throttle with the failsafe landing command.
}  // Finish publishing the landing throttle to every control path.

int16_t readCurrentPlausibleThrottle() {                                                   // Select the freshest currently available powered-flight throttle for non-link-loss fallback use.
  if (rc_throttle >= kLandingMinimumThrottle && rc_throttle <= kLandingMaximumThrottle) {  // Prefer the newest validated application throttle command.
    return static_cast<int16_t>(rc_throttle);                                              // Return the freshest raw application throttle.
  }                                                                                        // Finish checking the raw application throttle.
  if (channel_3 >= kLandingMinimumThrottle && channel_3 <= kLandingMaximumThrottle) {      // Fall back to the active receiver throttle channel.
    return static_cast<int16_t>(channel_3);                                                // Return the active receiver throttle channel.
  }                                                                                        // Finish checking the active receiver throttle channel.
  if (throttle >= kLandingMinimumThrottle && throttle <= kLandingMaximumThrottle) {        // Use the mixer throttle only as the final live fallback.
    return static_cast<int16_t>(throttle);                                                 // Return the current mixer throttle.
  }                                                                                        // Finish checking the mixer throttle.
  return kLandingMinimumThrottle;                                                          // Use the minimum powered throttle only when every live source is already invalid or reset.
}  // Finish selecting a plausible current flight throttle.

void captureHealthyFlightThrottle(int16_t packetThrottle) {                                           // Preserve the exact throttle carried by a complete valid control packet.
  if (start != 2U || gFailsafeLandingRunning) {                                                       // Capture only during normal armed flight before a failsafe landing begins.
    return;                                                                                           // Keep the previous trustworthy snapshot unchanged.
  }                                                                                                   // Finish checking whether a new snapshot is allowed.
  if (packetThrottle < kReceiverMinimum || packetThrottle > 2000) {                                   // Defensively reject a value outside the validated receiver protocol range.
    return;                                                                                           // Preserve the previous trustworthy snapshot if the caller is ever given invalid data.
  }                                                                                                   // Finish validating the packet-backed throttle.
  gLastHealthyFlightThrottle = constrain(packetThrottle, kReceiverMinimum, kLandingMaximumThrottle);  // Save the exact newest packet throttle while respecting the mixer flight ceiling.
  gLastHealthyFlightThrottleValid = true;                                                             // Mark the packet-backed in-flight throttle snapshot as usable.
}  // Finish capturing the exact valid-packet in-flight throttle.

void preserveThrottleDuringTimeoutWindow(uint32_t nowMs) {                                                                                                 // Prevent receiver timeout defaults from dropping throttle before the failsafe timeout is confirmed.
  if (start != 2U || gFailsafeLandingRunning || !gLastHealthyFlightThrottleValid || !gControlPacketReceived) {                                             // Require normal armed flight and a trustworthy packet snapshot.
    return;                                                                                                                                                // Leave normal control processing unchanged.
  }                                                                                                                                                        // Finish checking the pre-timeout hold requirements.
  if (gControlDisconnectRequested) {                                                                                                                       // Let an explicit disconnect enter the landing path immediately.
    return;                                                                                                                                                // Avoid applying a separate pre-timeout hold during explicit disconnect handling.
  }                                                                                                                                                        // Finish checking the explicit disconnect state.
  const uint32_t packetAgeMs = static_cast<uint32_t>(nowMs - gLastValidControlMs);                                                                         // Calculate the age of the newest complete valid control packet.
  if (packetAgeMs < kExpectedControlPeriodMs || packetAgeMs > kControlTimeoutMs) {                                                                         // Hold only after one expected packet interval and before confirmed timeout.
    return;                                                                                                                                                // Leave fresh control or confirmed failsafe handling unchanged.
  }                                                                                                                                                        // Finish checking the packet-gap hold window.
  const bool everyThrottlePathReset = throttle < kLandingMinimumThrottle && channel_3 < kLandingMinimumThrottle && rc_throttle < kLandingMinimumThrottle;  // Detect a receiver module resetting all throttle values to motor-off defaults.
  if (!everyThrottlePathReset) {                                                                                                                           // Avoid overriding legitimate live throttle processing.
    return;                                                                                                                                                // Preserve the current normal-flight throttle values.
  }                                                                                                                                                        // Finish checking for a receiver reset.
  publishLandingThrottle(gLastHealthyFlightThrottle);                                                                                                      // Restore the last packet-backed throttle until timeout starts the gradual landing ramp.
}  // Finish preserving throttle during the packet-loss confirmation window.

void forceMotorStopNow() {       // Stop every motor immediately and make the shutdown state deterministic.
  forceSafeReceiverState();      // Remove all stale receiver and throttle commands.
  resetPidRuntimeState();        // Remove every stored attitude PID correction.
  flightControlEmergencyStop();  // Use the single flight-control-owned motor shutdown path.
}  // Finish the immediate motor shutdown operation.

bool isControlLinkHealthy(uint32_t nowMs) {                                           // Evaluate control freshness using overflow-safe unsigned subtraction.
  const bool packetReceived = gControlPacketReceived;                                 // Snapshot whether any complete valid control packet has arrived.
  const bool disconnectRequested = gControlDisconnectRequested;                       // Snapshot the active-controller disconnect state.
  const uint32_t lastPacketMs = gLastValidControlMs;                                  // Snapshot the newest valid control packet timestamp.
  const uint32_t packetAgeMs = static_cast<uint32_t>(nowMs - lastPacketMs);           // Calculate packet age safely across millis overflow.
  return packetReceived && !disconnectRequested && packetAgeMs <= kControlTimeoutMs;  // Accept only a present, connected, and fresh control stream.
}  // Finish evaluating control-link health.

float readTestedSupplyVoltage() {                                                                       // Read the tested ESP supply-voltage proxy used by this hardware.
  uint32_t rawSum = 0U;                                                                                 // Start the raw VCC accumulator at zero.
  for (uint8_t sampleIndex = 0U; sampleIndex < kVoltageSamplesPerReading; sampleIndex++) {              // Collect the configured number of immediate samples.
    rawSum += static_cast<uint32_t>(ESP.getVcc());                                                      // Add one ESP supply-voltage sample in millivolt-like units.
  }                                                                                                     // Finish collecting immediate voltage samples.
  const float rawAverage = static_cast<float>(rawSum) / static_cast<float>(kVoltageSamplesPerReading);  // Calculate the average raw sample.
  return rawAverage * CAL_FACTOR / 1000.0f;                                                             // Apply the tested calibration factor and convert the result to volts.
}  // Finish reading the tested supply-voltage proxy.

bool updateBatteryVoltage(uint32_t nowMs) {                                                        // Update the global voltage only at the configured non-blocking interval.
  if (static_cast<uint32_t>(nowMs - gLastVoltageSampleMs) < kVoltageSampleIntervalMs) {            // Check whether the next sample time has arrived.
    return false;                                                                                  // Report that no new voltage decision sample was produced.
  }                                                                                                // Finish the voltage sampling interval check.
  gLastVoltageSampleMs = nowMs;                                                                    // Record the time of this voltage sampling attempt.
  const float measuredVoltage = readTestedSupplyVoltage();                                         // Read one averaged and calibrated voltage value.
  if (!isfinite(measuredVoltage)) {                                                                // Reject NaN and infinite values before safety logic uses them.
    return false;                                                                                  // Report that the attempted voltage sample was invalid.
  }                                                                                                // Finish the finite-value validation.
  if (measuredVoltage < kMinimumPlausibleVoltage || measuredVoltage > kMaximumPlausibleVoltage) {  // Reject values outside the plausible powered-controller range.
    return false;                                                                                  // Report that the attempted voltage sample was invalid.
  }                                                                                                // Finish the plausible-range validation.
  battery_voltage = measuredVoltage;                                                               // Publish the accepted voltage for telemetry and failsafe decisions.
  gBatteryVoltageValid = true;                                                                     // Remember that a trustworthy voltage sample now exists.
  return true;                                                                                     // Report that a new accepted voltage decision sample is available.
}  // Finish the periodic voltage update.

void enterFailsafeLanding(uint32_t nowMs, uint8_t reasonMask) {                                             // Latch a failsafe reason and initialize one shared deterministic landing ramp.
  const bool linkLossRequested = (reasonMask & static_cast<uint8_t>(FAILSAFE_REASON_LINK_LOSS)) != 0U;      // Detect whether link loss is part of the requested landing reason.
  const bool lowBatteryRequested = (reasonMask & static_cast<uint8_t>(FAILSAFE_REASON_LOW_BATTERY)) != 0U;  // Detect whether low battery is part of the requested landing reason.
  if (linkLossRequested) {                                                                                  // Apply link-loss-specific latching.
    gLinkFailsafeLatched = true;                                                                            // Prevent restored packets from automatically returning control during or after landing.
  }                                                                                                         // Finish applying link-loss-specific landing preparation.
  if (lowBatteryRequested) {                                                                                // Apply low-battery-specific latching.
    gBatteryFailsafeLatched = true;                                                                         // Prevent voltage rebound from cancelling the low-battery decision.
  }                                                                                                         // Finish applying low-battery-specific landing preparation.
  failsafe_reason = static_cast<uint8_t>(failsafe_reason | reasonMask);                                     // Add every requested landing reason to the public reason bit mask.
  neutralizeReceiverAxesForFailsafeLanding();                                                                // Reject Roll, Pitch, and Yaw commands for every failsafe reason before the landing begins.
  if (gFailsafeLandingRunning || gFailsafeLandingComplete) {                                                // Avoid restarting or accelerating a landing that is already active or completed.
    return;                                                                                                 // Preserve the original landing throttle timeline and final shutdown state.
  }                                                                                                         // Finish checking for an existing shared failsafe landing.
  if (start != 2U) {                                                                                        // Detect a failsafe request while the aircraft is not actively armed.
    gFailsafeLandingComplete = true;                                                                        // Mark the landing as complete because no powered descent is required.
    forceMotorStopNow();                                                                                    // Keep all motor outputs deterministically off in the disarmed state.
    return;                                                                                                 // Leave without starting a throttle ramp.
  }                                                                                                         // Finish handling a failsafe request while disarmed.
  gFailsafeLandingRunning = true;                                                                           // Start the shared low-battery or link-loss landing state.
  gFailsafeLandingComplete = false;                                                                         // Clear the final-shutdown marker for this landing.
  const int16_t landingStartThrottle = gExternalLandingThrottleValid                       // Prefer an exact actual mixer throttle supplied by an external safety subsystem.
                                         ? gExternalLandingThrottle                                  // Use the current assisted-flight mixer throttle without stepping back to the app's nominal 1500 command.
                                         : (linkLossRequested && gLastHealthyFlightThrottleValid     // Otherwise preserve the existing link-loss preference for the newest trustworthy in-flight snapshot.
                                              ? gLastHealthyFlightThrottle                            // Use the preserved healthy in-flight throttle for link loss.
                                              : readCurrentPlausibleThrottle());                      // Fall back to the existing bounded live throttle selection for other reasons.
  gExternalLandingThrottleValid = false;                                                             // Consume the one-shot external throttle override after selecting the landing start value.
  gLandingThrottle = constrain(static_cast<float>(landingStartThrottle), static_cast<float>(kLandingMinimumThrottle), static_cast<float>(kLandingMaximumThrottle));  // Capture a bounded starting throttle.
  gLastLandingUpdateMs = nowMs;                                                                                                                                      // Start time-based throttle integration from the current time.
  gMinimumThrottleReached = gLandingThrottle <= static_cast<float>(kLandingMinimumThrottle);                                                                         // Detect whether landing already starts at minimum throttle.
  gMinimumThrottleReachedMs = gMinimumThrottleReached ? nowMs : 0U;                                                                                                  // Store a fresh minimum-throttle timestamp or clear it.
  publishLandingThrottle(static_cast<int16_t>(gLandingThrottle + 0.5f));                                                                                             // Publish the captured landing throttle to every receiver and mixer path.
}  // Finish entering the shared failsafe landing state.

void updateLowBatteryTrigger(uint32_t nowMs, bool voltageUpdated) {                  // Confirm low voltage with consecutive accepted samples before latching.
  if (!voltageUpdated || gBatteryFailsafeLatched || !gBatteryVoltageValid) {         // Skip trigger evaluation without a new valid sample or after latching.
    return;                                                                          // Leave the existing low-voltage confirmation state unchanged.
  }                                                                                  // Finish the low-battery trigger precondition check.
  if (battery_voltage <= FAILSAFE_VOLTAGE) {                                         // Check whether the newest accepted voltage is at or below the tested threshold.
    if (gLowVoltageSampleCount < kLowVoltageConfirmSamples) {                        // Prevent the confirmation counter from overflowing.
      gLowVoltageSampleCount++;                                                      // Count this consecutive low-voltage decision sample.
    }                                                                                // Finish incrementing the low-voltage confirmation counter.
  } else {                                                                           // Handle a voltage sample above the trigger before failsafe is latched.
    gLowVoltageSampleCount = 0U;                                                     // Cancel the unconfirmed low-voltage sequence.
  }                                                                                  // Finish processing the newest voltage decision sample.
  if (start == 2U && gLowVoltageSampleCount >= kLowVoltageConfirmSamples) {          // Start landing only while armed after three consecutive low samples.
    enterFailsafeLanding(nowMs, static_cast<uint8_t>(FAILSAFE_REASON_LOW_BATTERY));  // Start or join the shared landing ramp with the low-battery reason.
  }                                                                                  // Finish checking whether battery landing must start.
}  // Finish updating the low-battery trigger.

void updateFailsafeLanding(uint32_t nowMs) {                                                                         // Apply the shared loop-rate-independent throttle ramp and final motor stop.
  if (!gFailsafeLandingRunning || gFailsafeLandingComplete) {                                                        // Skip landing updates when no active ramp remains.
    return;                                                                                                          // Preserve the current non-landing or completed state.
  }                                                                                                                  // Finish the active shared landing check.
  if (start != 2U) {                                                                                                 // Detect a pilot disarm or a higher-priority shutdown during failsafe landing.
    gFailsafeLandingRunning = false;                                                                                 // Stop the landing ramp because flight is already disarmed.
    gFailsafeLandingComplete = true;                                                                                 // Mark failsafe landing as finished by immediate shutdown.
    forceMotorStopNow();                                                                                             // Enforce zero PWM outputs immediately.
    return;                                                                                                          // Leave the landing update after the immediate stop.
  }                                                                                                                  // Finish handling an externally disarmed landing.
  neutralizeReceiverAxesForFailsafeLanding();                                                                        // Reject Roll, Pitch, and Yaw commands throughout every active failsafe landing.
  uint32_t elapsedMs = static_cast<uint32_t>(nowMs - gLastLandingUpdateMs);                                          // Calculate elapsed landing time safely across millis overflow.
  gLastLandingUpdateMs = nowMs;                                                                                      // Save the current time for the next landing integration step.
  if (elapsedMs > kMaximumLandingStepMs) {                                                                           // Detect an unexpectedly delayed landing update.
    elapsedMs = kMaximumLandingStepMs;                                                                               // Limit one throttle change to avoid an abrupt large motor step.
  }                                                                                                                  // Finish limiting the landing integration step.
  if (!gMinimumThrottleReached) {                                                                                    // Continue reducing throttle until the configured landing minimum is reached.
    const bool softLandingStage = gLandingThrottle <= static_cast<float>(kSoftLandingThrottle);                      // Select the slower descent rate only after reaching the soft-landing threshold.
    const float descendRate = softLandingStage ? FAILSAFE_DESCEND_RATE_LOW : FAILSAFE_DESCEND_RATE_HIGH;             // Use the correct rate for the current landing stage.
    const float throttleReduction = descendRate * static_cast<float>(elapsedMs) / 1000.0f;                           // Convert elapsed milliseconds into a stage-specific throttle reduction.
    gLandingThrottle -= throttleReduction;                                                                           // Apply the time-based throttle reduction.
    if (!softLandingStage && gLandingThrottle < static_cast<float>(kSoftLandingThrottle)) {                          // Prevent the faster first stage from stepping past the soft-landing threshold.
      gLandingThrottle = static_cast<float>(kSoftLandingThrottle);                                                   // Clamp exactly to the threshold so the next update starts the slower final stage.
    }                                                                                                                // Finish protecting the transition between landing stages.
    if (gLandingThrottle <= static_cast<float>(kLandingMinimumThrottle)) {                                           // Detect the first crossing of minimum landing throttle.
      gLandingThrottle = static_cast<float>(kLandingMinimumThrottle);                                                // Clamp landing throttle exactly to the configured minimum.
      gMinimumThrottleReached = true;                                                                                // Mark the minimum landing throttle timestamp as valid.
      gMinimumThrottleReachedMs = nowMs;                                                                             // Start the final landing delay from this exact moment.
    }                                                                                                                // Finish processing the minimum-throttle crossing.
  }                                                                                                                  // Finish reducing landing throttle.
  publishLandingThrottle(static_cast<int16_t>(gLandingThrottle + 0.5f));                                             // Publish the current rounded landing throttle to every receiver and mixer path.
  if (gMinimumThrottleReached && static_cast<uint32_t>(nowMs - gMinimumThrottleReachedMs) >= kLandingStopDelayMs) {  // Check whether the final minimum-throttle delay has elapsed.
    gFailsafeLandingRunning = false;                                                                                 // Stop the shared failsafe landing ramp.
    gFailsafeLandingComplete = true;                                                                                 // Latch completion so restored commands cannot restart the motors.
    forceMotorStopNow();                                                                                             // Disarm and force every motor PWM output to zero.
  }                                                                                                                  // Finish checking the final landing shutdown delay.
}  // Finish updating the shared failsafe landing.

void resetCompletedLinkLandingState() {                     // Reset only reusable landing state after a safely recovered link-loss event.
  gFailsafeLandingComplete = false;                         // Allow a future armed link-loss event to start a new landing ramp.
  gLandingThrottle = static_cast<float>(kReceiverMinimum);  // Reset the stored landing throttle to motor off.
  gLastLandingUpdateMs = 0U;                                // Clear the previous landing integration timestamp.
  gMinimumThrottleReachedMs = 0U;                           // Clear the minimum-throttle timestamp.
  gMinimumThrottleReached = false;                          // Mark the minimum-throttle timestamp as invalid.
  gLastHealthyFlightThrottle = kReceiverMinimum;            // Clear the throttle snapshot retained from the completed landing.
  gLastHealthyFlightThrottleValid = false;                  // Require a new healthy armed-flight snapshot before another link-loss landing.
}  // Finish resetting reusable link-loss landing state.

void clearRecoveredLinkFailsafe() {                                                // Clear only the link-loss reason after deliberate safe recovery.
  gLinkFailsafeLatched = false;                                                    // Release the internal link-loss latch.
  const uint8_t linkReasonMask = static_cast<uint8_t>(FAILSAFE_REASON_LINK_LOSS);  // Convert the link reason to an explicit byte mask.
  const uint8_t clearLinkMask = static_cast<uint8_t>(~linkReasonMask);             // Build a byte mask that clears only the link-loss bit.
  failsafe_reason = static_cast<uint8_t>(failsafe_reason & clearLinkMask);         // Preserve every other failsafe reason while clearing link loss.
  if (!gBatteryFailsafeLatched) {                                                  // Reset shared landing state only when no permanent battery fault remains.
    resetCompletedLinkLandingState();                                              // Prepare the failsafe manager for a future independent link-loss landing.
  }                                                                                // Finish checking whether shared landing state can be reused.
}  // Finish clearing a recovered link failsafe.

void updateLinkFailsafe(uint32_t nowMs) {                                            // Detect stale control data, start gradual landing, and manage safe recovery.
  const bool linkHealthy = isControlLinkHealthy(nowMs);                              // Evaluate the newest control-link state.
  if (!linkHealthy) {                                                                // Handle a missing, disconnected, or stale control stream.
    if (start == 2U && !gLinkFailsafeLatched) {                                      // Latch link loss only when the aircraft was actively armed.
      enterFailsafeLanding(nowMs, static_cast<uint8_t>(FAILSAFE_REASON_LINK_LOSS));  // Start the same gradual landing ramp used by low battery.
    } else if (!gLinkFailsafeLatched) {                                              // Handle missing control while the aircraft is not actively armed.
      start = 0U;                                                                    // Cancel any incomplete pre-arm state created before valid control is ready.
    }                                                                                // Finish selecting the armed or disarmed link-loss behavior.
  }                                                                                  // Finish handling an unhealthy control link.
  if (!gLinkFailsafeLatched) {                                                       // Skip recovery processing when no armed link-loss event is latched.
    return;                                                                          // Leave link processing without changing normal flight state.
  }                                                                                  // Finish checking the link-loss latch.
  if (gFailsafeLandingRunning) {                                                     // Keep gradual landing active even if control packets return during descent.
    neutralizeReceiverAxesForFailsafeLanding();                                      // Prevent restored packets from changing direction before landing completes.
    return;                                                                          // Leave recovery processing until motors are fully stopped.
  }                                                                                  // Finish preserving the active link-loss landing.
  const bool safeRecoveryThrottle = rc_throttle <= 1050;                             // Require the application throttle to be safely low before recovery.
  const bool recoveryReady = linkHealthy && safeRecoveryThrottle;                    // Require one fresh valid stream and low throttle before recovery.
  const bool batteryAllowsRecovery = !gBatteryFailsafeLatched;                       // Prevent link recovery from clearing a simultaneous battery failsafe.
  if (start == 0U && recoveryReady && batteryAllowsRecovery) {                       // Permit recovery only after landing has stopped and the aircraft remains disarmed.
    clearRecoveredLinkFailsafe();                                                    // Clear link loss without restoring the previous armed state.
    return;                                                                          // Leave the aircraft disarmed so the pilot must perform the normal arm sequence again.
  }                                                                                  // Finish evaluating safe link recovery.
  forceMotorStopNow();                                                               // Keep all motor outputs at zero after landing while recovery remains incomplete.
}  // Finish updating link failsafe.

void updateFailsafeAlarm(uint32_t nowMs) {                                // Generate a non-blocking repeated warning while any reason is latched.
  if (failsafe_reason == FAILSAFE_REASON_NONE) {                          // Skip the warning tone when no failsafe reason exists.
    return;                                                               // Leave alarm processing immediately.
  }                                                                       // Finish checking whether an alarm is required.
  if (static_cast<uint32_t>(nowMs - lastBeepTime) < kBuzzerIntervalMs) {  // Enforce the configured non-blocking beep interval.
    return;                                                               // Wait for the next allowed warning time.
  }                                                                       // Finish checking the warning interval.
  lastBeepTime = nowMs;                                                   // Record the start time of this warning beep.
  tone(BUZZER_PIN, kBuzzerFrequencyHz, kBuzzerDurationMs);                // Start a short asynchronous warning tone.
}  // Finish updating the failsafe alarm.

void refreshPublicFailsafeState() {                           // Keep the legacy public boolean synchronized with the reason bit mask.
  failsafe_active = failsafe_reason != FAILSAFE_REASON_NONE;  // Mark failsafe active whenever at least one reason is latched.
}  // Finish synchronizing the public failsafe state.

}  // Close the private implementation namespace.

void failsafeInit() {                                       // Reset all failsafe state and establish deterministic motor-off startup values.
  failsafe_active = false;                                  // Clear the legacy public active flag.
  failsafe_reason = FAILSAFE_REASON_NONE;                   // Clear every public failsafe reason bit.
  gLastValidControlMs = 0U;                                 // Clear the newest valid control packet timestamp.
  gControlPacketReceived = false;                           // Require the first complete valid control packet.
  gControlDisconnectRequested = false;                      // Clear any stale explicit disconnect request.
  gLastVoltageSampleMs = 0U;                                // Allow periodic voltage sampling to begin from a known state.
  gLowVoltageSampleCount = 0U;                              // Clear the consecutive low-voltage sample counter.
  gBatteryVoltageValid = false;                             // Require the first plausible voltage sample.
  gLinkFailsafeLatched = false;                             // Clear the internal link-loss latch.
  gBatteryFailsafeLatched = false;                          // Clear the internal low-battery latch at controller startup.
  gFailsafeLandingRunning = false;                          // Mark the shared failsafe landing ramp as inactive.
  gFailsafeLandingComplete = false;                         // Clear the shared failsafe landing completion marker.
  gLandingThrottle = static_cast<float>(kReceiverMinimum);  // Reset the internal landing throttle to motor off.
  gLastLandingUpdateMs = 0U;                                // Clear the previous landing integration timestamp.
  gMinimumThrottleReachedMs = 0U;                           // Clear the minimum-throttle timestamp.
  gMinimumThrottleReached = false;                          // Mark the minimum-throttle timestamp as invalid.
  gLastHealthyFlightThrottle = kReceiverMinimum;            // Reset the preserved in-flight throttle snapshot to motor off.
  gLastHealthyFlightThrottleValid = false;                  // Require a fresh healthy armed-flight throttle snapshot.
  gExternalLandingThrottleValid = false;                       // Clear any stale external safety throttle override.
  gExternalLandingThrottle = kReceiverMinimum;                    // Reset the external safety throttle value to motor off.
  battery_voltage = 0.0f;                                   // Clear the published voltage until the first plausible reading arrives.
  lastBeepTime = 0U;                                        // Reset the non-blocking warning tone timer.
  forceMotorStopNow();                                      // Force deterministic disarmed state and zero motor PWM outputs.
}  // Finish initializing the failsafe system.

void failsafeNotifyValidControlPacket(uint32_t nowMs, int16_t packetThrottle) {  // Record one complete validated packet and its exact throttle command.
  gLastValidControlMs = nowMs;                                                   // Publish the timestamp of the newest complete valid control packet.
  gControlPacketReceived = true;                                                 // Allow arming as soon as one current complete control frame exists.
  gControlDisconnectRequested = false;                                           // Allow a new active controller to recover an explicit disconnect.
  captureHealthyFlightThrottle(packetThrottle);                                  // Snapshot the throttle from this exact packet instead of a possibly one-loop-old mixer value.
}  // Finish recording a valid control packet.

void failsafeNotifyEffectiveThrottle(int16_t effectiveThrottle) {  // Track the actual mixer throttle so assisted flight cannot fall back to a stale nominal app command.
  if (start != 2U || failsafe_active || gFailsafeLandingRunning) return;   // Capture only during normal healthy armed flight.
  if (effectiveThrottle < kReceiverMinimum || effectiveThrottle > kLandingMaximumThrottle) return;  // Reject impossible actual throttle values.
  gLastHealthyFlightThrottle = effectiveThrottle;                           // Store the newest actual mixer base throttle.
  gLastHealthyFlightThrottleValid = true;                                   // Publish this exact throttle as trustworthy for a later link-loss transition.
}

void failsafeTriggerAltitudeAssist(uint32_t nowMs, int16_t effectiveThrottle) {  // Reuse the existing deterministic landing for an altitude-assist safety fault.
  if (start != 2U || gFailsafeLandingRunning || gFailsafeLandingComplete) return; // Start only once while active flight still requires a powered descent.
  gExternalLandingThrottle = constrain(effectiveThrottle, kLandingMinimumThrottle, kLandingMaximumThrottle);  // Preserve the actual mixer throttle at the instant altitude assistance becomes unsafe.
  gExternalLandingThrottleValid = true;                                      // Tell shared landing initialization to use this exact throttle instead of the app's nominal channel value.
  enterFailsafeLanding(nowMs, static_cast<uint8_t>(FAILSAFE_REASON_ALTITUDE_ASSIST));  // Latch the new reason and begin the existing two-stage descent.
  refreshPublicFailsafeState();                                               // Publish the newly latched reason immediately.
}

void failsafeStartCommandedLanding(uint32_t nowMs, int16_t effectiveThrottle) {  // Start a user-requested deterministic descent without turning that normal request into a latched fault.
  if (start != 2U || gFailsafeLandingRunning) return;                           // Start only once while the aircraft is still armed and no shared landing already owns throttle.
  if (gFailsafeLandingComplete && failsafe_reason == FAILSAFE_REASON_NONE && !gLinkFailsafeLatched && !gBatteryFailsafeLatched) {  // Re-open the reusable shared landing state after a previous clean commanded landing.
    resetCompletedLinkLandingState();                                                                                             // Clear only reusable landing bookkeeping; never clear real fault reasons here.
  }                                                                                                                               // Finish preparing a previously completed clean landing state.
  if (gFailsafeLandingComplete) return;                                                                                           // Never restart a completed real failsafe landing that still requires its recovery path.
  gExternalLandingThrottle = constrain(effectiveThrottle, kLandingMinimumThrottle, kLandingMaximumThrottle);                     // Preserve the actual current mixer throttle as the commanded descent start point.
  gExternalLandingThrottleValid = true;                                                                                          // Tell the shared landing initializer to consume this exact throttle.
  enterFailsafeLanding(nowMs, static_cast<uint8_t>(FAILSAFE_REASON_NONE));                                                       // Reuse the proven two-stage descent without latching link, battery, or altitude-fault reason bits.
  refreshPublicFailsafeState();                                                                                                  // Keep the public failsafe flag false for an ordinary user-requested Landing.
}

void failsafeNotifyControlDisconnect() {                                           // Record an explicit disconnect from the active control client.
  gControlDisconnectRequested = true;                                              // Make link health fail immediately without waiting for packet timeout.
  const uint32_t nowMs = millis();                                                 // Capture one monotonic timestamp for disconnect-triggered landing setup.
  if (start == 2U) {                                                               // Detect an explicit disconnect while the aircraft is actively armed.
    enterFailsafeLanding(nowMs, static_cast<uint8_t>(FAILSAFE_REASON_LINK_LOSS));  // Start the same gradual landing ramp used by low battery.
  } else {                                                                         // Handle an explicit disconnect while the aircraft is already disarmed.
    start = 0U;                                                                    // Cancel any incomplete pre-arm state before control recovery begins.
    forceMotorStopNow();                                                           // Keep all motor outputs deterministically off while disarmed.
  }                                                                                // Finish selecting armed or disarmed disconnect behavior.
  refreshPublicFailsafeState();                                                    // Publish the new link-loss state before returning from the disconnect callback.
}  // Finish recording the active controller disconnect.

void failsafeUpdate(uint32_t nowMs) {                       // Run every failsafe detector and action without blocking the flight loop.
  preserveThrottleDuringTimeoutWindow(nowMs);               // Prevent receiver reset values from causing a sudden drop before link loss is confirmed.
  const bool voltageUpdated = updateBatteryVoltage(nowMs);  // Update the tested voltage proxy at its configured interval.
  updateLinkFailsafe(nowMs);                                // Process control timeout before any lower-priority battery landing action.
  updateLowBatteryTrigger(nowMs, voltageUpdated);           // Confirm and latch low battery using consecutive accepted samples.
  updateFailsafeLanding(nowMs);                             // Apply the shared time-based throttle ramp or final motor shutdown.
  if (gFailsafeLandingComplete && start == 0U && failsafe_reason == FAILSAFE_REASON_NONE && !gLinkFailsafeLatched && !gBatteryFailsafeLatched) {  // Detect completion of an ordinary user-commanded landing.
    resetCompletedLinkLandingState();                         // Re-arm only the reusable landing machinery while leaving the aircraft itself safely DISARMED.
  }                                                           // Finish clean commanded-landing bookkeeping.
  refreshPublicFailsafeState();                             // Synchronize the legacy public active flag with all latched reasons.
  updateFailsafeAlarm(nowMs);                               // Emit a short non-blocking warning for any latched failsafe reason.
}  // Finish the complete failsafe update.

void failsafeLanding() {     // Preserve the original function name for the existing call in flight control.
  failsafeUpdate(millis());  // Run the new complete failsafe manager using the current monotonic time.
}  // Finish the backward-compatible failsafe wrapper.

bool failsafeCanArm(uint32_t nowMs) {                                                                // Evaluate every pre-arm requirement owned by the failsafe manager.
  if (!isControlLinkHealthy(nowMs)) {                                                                // Reject arming without a fresh complete control stream.
    return false;                                                                                    // Keep the aircraft disarmed while control data is missing or stale.
  }                                                                                                  // Finish the live control-link pre-arm check.
  if (failsafe_reason != FAILSAFE_REASON_NONE || gLinkFailsafeLatched || gBatteryFailsafeLatched) {  // Reject arming while any failsafe remains latched.
    return false;                                                                                    // Keep the aircraft disarmed until the permitted recovery path completes.
  }                                                                                                  // Finish the latched failsafe pre-arm check.
  if (gBatteryVoltageValid && battery_voltage <= FAILSAFE_VOLTAGE) {                                 // Reject arming when the tested voltage proxy is already critically low.
    return false;                                                                                    // Keep the aircraft disarmed until a safe battery is installed and the controller restarts.
  }                                                                                                  // Finish the low-voltage pre-arm check.
  return true;                                                                                       // Permit an explicit ARM request after every link, battery, and latch requirement is healthy.
}  // Finish evaluating failsafe pre-arm requirements.

bool failsafeLinkHealthy(uint32_t nowMs) {  // Expose current link health for diagnostics or telemetry.
  return isControlLinkHealthy(nowMs);       // Return the internal overflow-safe control freshness result.
}  // Finish exposing control-link health.

bool failsafeLandingActive() {                                  // Expose whether any gradual failsafe landing is currently active.
  return gFailsafeLandingRunning && !gFailsafeLandingComplete;  // Return true for both low-battery and link-loss landing ramps.
}  // Finish exposing shared gradual failsafe landing activity.


bool failsafeBatteryLockoutActive() {                      // Expose the low-battery latch without permitting another module to clear it.
  return gBatteryFailsafeLatched ||                        // Keep lockout active after the confirmed low-battery trigger.
         (failsafe_reason & FAILSAFE_REASON_LOW_BATTERY);  // Also honor the published low-battery reason bit.
}  // Finish exposing the persistent low-battery lockout state.

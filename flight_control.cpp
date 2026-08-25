#include "flight_control.h"  // Provide the preserved shared flight state and control API.

// WiFi status
unsigned long lastWiFiCheck = 0;               // Store the most recent Wi-Fi status-check timestamp.
const unsigned long wifiCheckInterval = 3000;  // Check Wi-Fi connection state every three seconds.
unsigned long wifiDisconnectedSince = 0;       // Store when the current Wi-Fi outage began.
bool wifiWasConnected = false;                 // Track whether the previous Wi-Fi status check was connected.

// Loop / timing variables
unsigned long Time = 0;                                    // Execute this statement as part of the current operation.
unsigned long loopStartTime = 0;                           // Execute this statement as part of the current operation.
unsigned long lastLoopStartTime = 0;                       // Execute this statement as part of the current operation.
const unsigned long loopTime = ESPFLIGHT_CONTROL_LOOP_US;  // Use the configured 4000-microsecond period for the 250-Hz control loop.
unsigned long nowMilis = 0;                                // Execute this statement as part of the current operation.

// Debug / testing
unsigned long total = 0;         // Execute this statement as part of the current operation.
unsigned int count = 0;          // Initialize count with its required starting value.
unsigned long lastBeepTime = 0;  // Execute this statement as part of the current operation.

// -------------------------------
// Global battery voltage storage
// -------------------------------

volatile float battery_voltage = 0.0;  // Initialize battery_voltage with its required starting value.

// RC channels
int rc_throttle = 1000;  // Initialize rc_throttle with its required starting value.
int rc_roll = 1500;      // Initialize rc_roll with its required starting value.
int rc_pitch = 1500;     // Initialize rc_pitch with its required starting value.
int rc_yaw = 1500;       // Initialize rc_yaw with its required starting value.

int throttle = 1000;  // Initialize the mixer throttle at the motor-off command.

// Raw RC channel values
int32_t channel_1 = 1500;  // Initialize the roll receiver channel at neutral.
int32_t channel_2 = 1500;  // Initialize the transformed pitch receiver channel at neutral.
int32_t channel_3 = 1000;  // Initialize the throttle receiver channel at motor off.
int32_t channel_4 = 1500;  // Initialize the yaw receiver channel at neutral.

// PID runtime variables
float roll_level_adjust = 0;   // Initialize roll_level_adjust with its required starting value.
float pitch_level_adjust = 0;  // Initialize pitch_level_adjust with its required starting value.

// Misc flight variables
float angle_roll_acc = 0;   // Initialize angle_roll_acc with its required starting value.
float angle_pitch_acc = 0;  // Initialize angle_pitch_acc with its required starting value.
float angle_pitch = 0;      // Initialize angle_pitch with its required starting value.
float angle_roll = 0;       // Initialize angle_roll with its required starting value.

// I2C and misc (only non-IMU globals)
uint8_t start = 0;          // Initialize start with its required starting value.
uint8_t error_counter = 0;  // Initialize error_counter with its required starting value.
uint8_t error_led = 0;      // Initialize error_led with its required starting value.

int16_t esc_1 = 1000;  // Initialize motor command one at the firmware motor-off value.
int16_t esc_2 = 1000;  // Initialize motor command two at the firmware motor-off value.
int16_t esc_3 = 1000;  // Initialize motor command three at the firmware motor-off value.
int16_t esc_4 = 1000;  // Initialize motor command four at the firmware motor-off value.


uint32_t error_timer = 0;  // Initialize error_timer with its required starting value.

unsigned long flightStartTime = 0;  // Store the start of the current qualifying flight-time segment (ARMED with effective throttle above 1100).
unsigned long totalFlightTime = 0;  // Accumulate only qualifying flight milliseconds across the current board power cycle.
bool flightTimerRunning = false;    // Track whether the ARMED + effective-throttle-above-1100 timing condition is currently active.
bool flightSessionStarted = false;  // Track whether this powered board has started its cumulative timing session.
uint32_t flightSessionId = 0U;      // Identify the current powered board timing session for the application.
uint32_t bootSessionId = 0U;        // Identify this firmware boot so application logs cannot be counted twice.

// Explicit ARM/DISARM is owned by the high-level command path. Throttle never toggles the ARM state.
static uint32_t gLastValidImuSampleMs = 0U;  // Record the newest fresh MPU6050 sample for command-time pre-arm validation.
static bool gValidImuSampleSeen = false;     // Require at least one complete runtime MPU6050 sample before ARM can be accepted.
static bool gPoweredFlightActive = false;    // Remember that this ARM cycle has actually driven the motors above the motor-start threshold; keep safety landing authority independent from timer pause/resume.

// Define pins
const int LED = ESPFLIGHT_LED_PIN;     // Initialize LED with its required starting value.
const int M1 = ESPFLIGHT_MOTOR_1_PIN;  // Initialize M1 with its required starting value.
const int M2 = ESPFLIGHT_MOTOR_2_PIN;  // Initialize M2 with its required starting value.
const int M3 = ESPFLIGHT_MOTOR_3_PIN;  // Initialize M3 with its required starting value.
const int M4 = ESPFLIGHT_MOTOR_4_PIN;  // Initialize M4 with its required starting value.

void pinConfig() {        // Begin the pinConfig function implementation.
	pinMode(LED, OUTPUT);   // Configure the status LED pin as a digital output.
	pinMode(M1, OUTPUT);    // Configure motor output one before writing its safe level.
	pinMode(M2, OUTPUT);    // Configure motor output two before writing its safe level.
	pinMode(M3, OUTPUT);    // Configure motor output three before writing its safe level.
	pinMode(M4, OUTPUT);    // Configure motor output four before writing its safe level.
	digitalWrite(M1, LOW);  // Force motor output one low during startup.
	digitalWrite(M2, LOW);  // Force motor output two low during startup.
	digitalWrite(M3, LOW);  // Force motor output three low during startup.
	digitalWrite(M4, LOW);  // Force motor output four low during startup.
}  // Close the current declaration or execution block.

void inputReceiver() {                                // Begin the inputReceiver function implementation.
	channel_1 = rc_roll;                                // Update channel_1 with the value calculated on this line.
	channel_2 = map(rc_pitch, 1000, 2000, 2000, 1000);  // Update channel_2 with the value calculated on this line.
	channel_3 = rc_throttle;                            // Update channel_3 with the value calculated on this line.
	channel_4 = rc_yaw;                                 // Update channel_4 with the value calculated on this line.
}  // Close the current declaration or execution block.

// ====== Start or resume a qualifying real-flight timing segment ======
void startFlightTimer() {    // Begin or resume timing only while the real-flight timing condition is satisfied.
	if (flightTimerRunning) {  // Ignore duplicate start requests while the qualifying timing segment is already active.
		return;                  // Keep the active timing segment unchanged.
	}                          // Finish guarding against duplicate starts.

	if (!flightSessionStarted) {    // Create a new session only once after the board powers up.
		flightSessionId++;            // Assign a new identifier before publishing the new flight session.
		if (flightSessionId == 0U) {  // Detect the extremely rare unsigned wrap to reserved zero.
			flightSessionId = 1U;       // Keep zero reserved for the pre-first-flight startup state.
		}                             // Finish protecting the session identifier.
		totalFlightTime = 0U;         // Reset cumulative time only for the first qualifying (>1100) flight segment after this power cycle.
		flightSessionStarted = true;  // Keep later qualifying segments/reconnects inside this powered timing session until the board power-cycles.
	}                               // Finish new-session initialization.

	flightStartTime = millis();  // Start a new qualifying timing segment within the current powered session.
	flightTimerRunning = true;   // Mark the current qualifying flight-time segment as active.
}  // Finish starting or resuming the flight timer.

// ====== Pause flight timing when its real-flight condition is no longer true ======
void stopFlightTimer() {                                        // Pause timing while preserving the connected-session total.
	if (flightTimerRunning) {                                     // Stop only when a qualifying timing segment is active.
		totalFlightTime += (uint32_t)(millis() - flightStartTime);  // Add this armed segment without resetting earlier flight time.
		flightTimerRunning = false;                                 // Mark flight timing as paused until ARMED + effective throttle > 1100 becomes true again.
	}                                                             // Finish the guarded timer pause.
}  // Finish pausing the current flight timer.

// ====== Preserve the current powered flight session ======
void flightControlEndSession() {  // Keep compatibility with callers that report a control-link loss.
	// Deliberately do nothing. WebSocket disconnects, stale packets, menus, and
	// application route changes are not proof that the battery was removed. The
	// global timer state resets naturally only when the ESP8266 power cycles.
}  // Finish preserving the powered flight session.

void flightControlEmergencyStop() {  // Apply one deterministic emergency shutdown path for every subsystem.
	start = 0U;                        // Force the aircraft into the disarmed state.
	esc_1 = 1000;                      // Force motor command one to the firmware motor-off value.
	esc_2 = 1000;                      // Force motor command two to the firmware motor-off value.
	esc_3 = 1000;                      // Force motor command three to the firmware motor-off value.
	esc_4 = 1000;                      // Force motor command four to the firmware motor-off value.
	analogWrite(M1, 0);                // Immediately disable PWM output for motor one.
	analogWrite(M2, 0);                // Immediately disable PWM output for motor two.
	analogWrite(M3, 0);                // Immediately disable PWM output for motor three.
	analogWrite(M4, 0);                // Immediately disable PWM output for motor four.
	stopFlightTimer();                 // Stop the flight timer after motor outputs are safe.
	gPoweredFlightActive = false;          // End current-arm powered-flight authority so a later failsafe or ARM cycle cannot inherit it.
}  // Finish the centralized emergency motor shutdown.

// ====== Get cumulative real flight time for the current board power cycle ======
unsigned long getFlightTimeSeconds() {                              // Return one stable cumulative duration to telemetry.
	unsigned long elapsedMilliseconds = totalFlightTime;              // Begin with every completed qualifying flight segment.
	if (flightTimerRunning) {                                         // Include the currently active qualifying segment when present.
		elapsedMilliseconds += (uint32_t)(millis() - flightStartTime);  // Add current elapsed time without mutating stored state.
	}                                                                 // Finish including the active segment.
	return elapsedMilliseconds / 1000UL;                              // Convert after summing to avoid per-segment truncation loss.
}  // Finish returning the cumulative current-flight duration.


#include <math.h>      // Provide square-root, sine, arc-sine, and finite-value functions used by attitude control.
#include "imu.h"       // Provide the original MPU6050 readings and setup-independent runtime values.
#include "pid.h"       // Provide the original Roll, Pitch, and Yaw PID controller state.
#include "failsafe.h"  // Provide arming validation and gradual failsafe landing management.
#include "altitude.h"  // Provide the previously flight-tested VL53L0X altitude-hold relationship and assisted-flight state.


bool flightControlImuReady(uint32_t nowMs) {  // Report whether a fresh trustworthy MPU6050 sample is available for an ARM request.
	if (!gValidImuSampleSeen) return false;      // Never arm before the first complete runtime MPU6050 sample.
	if (i2cError != 0U) return false;            // Reject ARM while any consecutive MPU6050 communication error is present.
	if (static_cast<uint32_t>(nowMs - gLastValidImuSampleMs) > 100U) return false;  // Require a sample no older than one tenth of a second.
	if (!isfinite(angle_roll) || !isfinite(angle_pitch)) return false;               // Reject broken attitude estimates.
	if (fabsf(angle_roll) > 60.0f || fabsf(angle_pitch) > 60.0f) return false;         // Reject ARM while the aircraft is already at an unsafe tilt.
	return true;                                // Publish a healthy pre-arm IMU state.
}  // Finish command-time IMU readiness validation.

bool flightControlRequestArm(uint32_t nowMs) {  // Enter ARMED-ready state without starting any motor.
	if (start == 2U) return true;                // Make repeated ARM requests idempotent.
	if (start != 0U) return false;               // Reject unexpected intermediate flight states.
	if (channel_3 > 1050) return false;          // Require the real received throttle to be at minimum.
	if (failsafe_active || failsafeLandingActive()) return false;  // Never arm while a safety landing owns the aircraft.
	if (!failsafeCanArm(nowMs)) return false;     // Preserve link, battery, and failsafe pre-arm requirements.
	if (!flightControlImuReady(nowMs)) return false;  // Require a fresh verified MPU6050 sample and safe attitude.

	altitudeResetAssist();                        // Begin every manual ARM from a clean assisted-flight state.
	resetPidRuntime();                            // Keep Roll/Pitch/Yaw PID empty while ARMED but idle.
	gPoweredFlightActive = false;                    // A new ARM is only READY; no powered-flight authority exists until throttle actually rises above 1050.
	stopFlightTimer();                               // Ensure time cannot carry over while the new ARM remains below the >1100 flight-time threshold.
	start = 2U;                                   // Publish ARMED-ready state; motor PWM remains zero until throttle rises above 1050.
	angle_pitch = angle_pitch_acc;                // Align integrated Pitch to the current accelerometer estimate.
	angle_roll = angle_roll_acc;                  // Align integrated Roll to the current accelerometer estimate.
	return true;                                  // Report that the explicit ARM command was accepted.
}  // Finish explicit motor-off ARM transition.

void flightControlRequestDisarm() {  // Cancel every flight mode and force immediate motor shutdown.
	altitudeResetAssist();              // Prevent Takeoff/Hold/Landing from restoring throttle after DISARM.
	resetPidRuntime();                  // Remove all attitude PID output and memory before cutting motor power.
	flightControlEmergencyStop();       // Set DISARMED state and write zero PWM to all four motors immediately.
}  // Finish explicit DISARM transition.

void flightControl250Hz() {  // Begin the flightControl250Hz function implementation.
	static bool pidDerivativePrimePending = false;  // Request one derivative-history prime after ARM, throttle activation, failsafe entry, or any invalid MPU sample.
	static bool failsafeControlWasLocked = false;   // Remember the previous failsafe directional-lock state so entry cannot create a derivative kick.
	constexpr int32_t kMotorStartThrottle = 1050;  // Keep motors and normal attitude PID fully off until real throttle rises above this boundary.
	const bool imuSampleValid = gyroSignalen();     // Read one complete MPU6050 sample and remember whether fresh sensor data is available.
	if (imuSampleValid) {                        // Publish runtime IMU freshness for asynchronous ARM-command validation.
		gLastValidImuSampleMs = nowMilis;             // Record the newest complete sample timestamp.
		gValidImuSampleSeen = true;                    // Remember that runtime MPU6050 data has been proven valid.
	}                                               // Finish updating command-time IMU readiness state.

	if (imuSampleValid) {  // Update attitude state only from a complete fresh MPU6050 sample.
		// ================================
		// Gyro PID input smoothing (optimized for 250Hz loop)
		// ================================
		const float gyroScale = 1.0f / 32.8f;  // 1000 dps full scale → deg/sec
		const float alpha = 0.3f;              // New data weight
		const float oneMinusAlpha = 0.7f;      // Old data weight

		gyro_roll_input = (gyro_roll_input * oneMinusAlpha) + ((float)gyro_roll * gyroScale * alpha);     // Update gyro_roll_input with the value calculated on this line.
		gyro_pitch_input = (gyro_pitch_input * oneMinusAlpha) + ((float)gyro_pitch * gyroScale * alpha);  // Update gyro_pitch_input with the value calculated on this line.
		gyro_yaw_input = (gyro_yaw_input * oneMinusAlpha) + ((float)gyro_yaw * gyroScale * alpha);        // Update gyro_yaw_input with the value calculated on this line.

		// ================================
		// Gyro angle integration (250 Hz = 0.004s)
		// ================================
		const float loopHz = 250.0f;                 // Initialize loopHz with its required starting value.
		const float dt = 1.0f / loopHz;              // 0.004 sec
		const float dt_deg = dt * gyroScale;         // Deg per LSB per loop
		const float deg2rad = 3.14159265f / 180.0f;  // Conversion factor
		const float dt_rad = dt_deg * deg2rad;       // Rad per LSB per loop

		// Integrate gyro rates to update pitch and roll angles
		angle_pitch += (float)gyro_pitch * dt_deg;  // Update pitch angle (deg)
		angle_roll += (float)gyro_roll * dt_deg;    // Update roll angle (deg)

		// ================================
		// Yaw cross-axis correction (for drift compensation)
		// ================================
		const float yawRotation = (float)gyro_yaw * dt_rad;  // Small yaw rotation (rad)

		angle_pitch -= angle_roll * sinf(yawRotation);  // Correct pitch by yaw effect
		angle_roll += angle_pitch * sinf(yawRotation);  // Correct roll by yaw effect

		// Accelerometer angle calculations.
		const float accX = static_cast<float>(acc_x);                         // Convert the Roll-axis accelerometer sample before squaring to prevent signed integer overflow.
		const float accY = static_cast<float>(acc_y);                         // Convert the Pitch-axis accelerometer sample before squaring to prevent signed integer overflow.
		const float accZ = static_cast<float>(acc_z);                         // Convert the vertical accelerometer sample before squaring to prevent signed integer overflow.
		const float accVector = sqrtf((accX * accX) + (accY * accY) + (accZ * accZ));  // Calculate the total acceleration magnitude using floating-point arithmetic.
		acc_total_vector = static_cast<int32_t>(accVector);                   // Store the bounded magnitude in the existing shared integer variable without changing downstream behavior.

		if (abs(acc_y) < acc_total_vector) {                                 // Prevent asin from receiving a ratio outside its valid range.
			angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296;  //Calculate the pitch angle.
		}                                                                    // Close the current declaration or execution block.
		if (abs(acc_x) < acc_total_vector) {                                 // Prevent asin from receiving a ratio outside its valid range.
			angle_roll_acc = asin((float)acc_x / acc_total_vector) * 57.296;   //Calculate the roll angle.
		}                                                                    // Close the current declaration or execution block.

		angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;  //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
		angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;     //Correct the drift of the gyro roll angle with the accelerometer roll angle.

		pitch_level_adjust = angle_pitch * 15;  //Calculate the pitch angle correction.
		roll_level_adjust = angle_roll * 15;    //Calculate the roll angle correction.


		if (!auto_level) {         // Evaluate this condition before executing its protected branch.
			pitch_level_adjust = 0;  // Update pitch_level_adjust with the value calculated on this line.
			roll_level_adjust = 0;   // Update roll_level_adjust with the value calculated on this line.
		}                          // Close the current declaration or execution block.
	}  // Finish updating sensor-derived attitude state from a valid MPU6050 sample.

	// ARM/DISARM is command-driven. Throttle is now only a motor/flight input and can never toggle the ARM state.

	// Treat excessive tilt or a broken attitude estimate as a hard emergency stop, not as a normal state change that continues through the rest of the mixer cycle.
	const bool attitudeEstimateInvalid = !isfinite(angle_roll) || !isfinite(angle_pitch);  // Detect NaN or infinite attitude state before any PID or assisted-flight code can use it.
	const bool excessiveTilt = fabsf(angle_roll) > 60.0f || fabsf(angle_pitch) > 60.0f;    // Detect either Roll or Pitch exceeding the configured sixty-degree safety limit.
	if (attitudeEstimateInvalid || excessiveTilt) {                                        // Give the hard attitude safety limit absolute priority over Landing, Hold, failsafe shaping, and the normal mixer.
		altitudeResetAssist();                                                               // Disconnect altitude assistance so no later loop can restore an assisted throttle correction.
		resetPidRuntime();                                                                   // Clear Roll/Pitch/Yaw PID outputs and memory before motor power is removed.
		flightControlEmergencyStop();                                                        // Set start=0 and write zero PWM to all four motors immediately.
		return;                                                                              // Exit this 250-Hz cycle now so no code below can overwrite the emergency motor-off state.
	}                                                                                     // Finish the hard attitude emergency-stop check.

	// Update failsafe first, then the optional altitude-assist state, before building any setpoint so every safety transition takes control in this same 250-Hz cycle.
	failsafeUpdate(nowMilis);  // Detect link loss or low battery and update the existing deterministic landing throttle.
	altitudeUpdateControlState(nowMilis);  // Preserve Takeoff -> Hold -> Landing transitions and transfer sensor/height faults to failsafe immediately.
	if (altitudeLandingShouldDisarm()) {  // Keep the legacy touchdown endpoint only as a secondary fallback if the five-centimeter hard cutoff did not already stop the aircraft.
		altitudeResetAssist();               // Disconnect altitude PID and clear its runtime state before motor shutdown.
		resetPidRuntime();                   // Remove any remaining attitude PID correction before the final motor cutoff.
		flightControlEmergencyStop();        // Disarm and force every motor PWM output to zero immediately.
		return;                              // Leave this control cycle so no later mixer path can write a powered command after shutdown.
	}
	if (start != 2U) {                     // Detect any stop completed inside failsafe or altitude processing during this same control cycle.
		resetPidRuntime();                   // Keep attitude PID memory empty while the aircraft is no longer armed.
		flightControlEmergencyStop();        // Guarantee zero PWM even if the subsystem changed only start/state and did not itself write motor-off outputs.
		return;                              // Exit immediately so no later PID or mixer code can restore motor power after disarm.
	}
	// Latch real powered-flight entry independently from the flight timer. The timer uses a stricter >1100 threshold, while failsafe must remember that motors have already been active even if timing later pauses.
	if (!gPoweredFlightActive && start == 2U && channel_3 > kMotorStartThrottle) {  // Detect the first real motor-drive request of this ARM cycle.
		gPoweredFlightActive = true;                                               // Preserve powered-flight history until explicit/hard DISARM.
	}                                                                            // Finish current-arm powered-flight latching.

	const bool failsafeControlLocked = failsafeLandingActive();  // Lock all directional control for every active failsafe landing, including altitude-assist safety transfer.
	constexpr int16_t kFailsafePidDisableThrottle = 1100;        // Treat this landing throttle and anything lower as touchdown where PID authority must be removed.
	const bool failsafePidDisabled = failsafeControlLocked && throttle <= kFailsafePidDisableThrottle;  // Disable stabilization motor differences at and below touchdown throttle.
	if (failsafeControlLocked && !failsafeControlWasLocked) {     // Detect the exact control cycle in which an automatic failsafe landing first takes over.
		pidDerivativePrimePending = true;                           // Prime derivative history against the new zero-attitude failsafe setpoints before calculating PID.
	}                                                             // Finish protecting the failsafe setpoint transition from D-kick.
	failsafeControlWasLocked = failsafeControlLocked;             // Remember the current failsafe lock state for the next 250Hz control cycle.

	// The PID set point in degrees per second is determined by the roll receiver input.
	// Dividing the full 500-point stick deviation by 3 gives a maximum commanded rate of about 166.7 degrees per second.
	if (failsafeControlLocked) {                                             // Ignore all pilot direction commands for every active failsafe landing.
		pid_roll_setpoint = -(angle_roll * 15.0f) / 3.0f;                  // Command the existing Roll controller toward a zero-degree level attitude.
		pid_pitch_setpoint = -(angle_pitch * 15.0f) / 3.0f;                // Command the existing Pitch controller toward a zero-degree level attitude.
		pid_yaw_setpoint = 0.0f;                                           // Hold zero commanded Yaw rate throughout failsafe descent.
	} else {                                                               // Preserve the tested normal-flight receiver-to-setpoint behavior outside failsafe.
		pid_roll_setpoint = 0;                                              // Start the Roll setpoint from the centered receiver command.
		// Use the direct centered receiver deviation; no additional software dead band is applied here.
		if (channel_1 > 1500) pid_roll_setpoint = channel_1 - 1500;         // Positive deviation from center stick.
		else if (channel_1 < 1500) pid_roll_setpoint = channel_1 - 1500;    // Negative deviation from center stick.

		pid_roll_setpoint -= roll_level_adjust;                            // Subtract the normal-flight angle correction from the receiver Roll command.
		pid_roll_setpoint /= 3.0f;                                         // Scale the Roll command to degrees per second.

		pid_pitch_setpoint = 0;                                            // Start the Pitch setpoint from the centered receiver command.
		// Use the direct centered receiver deviation; no additional software dead band is applied here.
		if (channel_2 > 1500) pid_pitch_setpoint = channel_2 - 1500;        // Positive deviation from center stick.
		else if (channel_2 < 1500) pid_pitch_setpoint = channel_2 - 1500;   // Negative deviation from center stick.

		pid_pitch_setpoint -= pitch_level_adjust;                          // Subtract the normal-flight angle correction from the receiver Pitch command.
		pid_pitch_setpoint /= 3.0f;                                        // Scale the Pitch command to degrees per second.

		pid_yaw_setpoint = 0;                                              // Start the Yaw rate setpoint at zero.
		// Use the direct centered receiver deviation; no additional software dead band is applied here.
		if (channel_3 > 1050) {                                            // Only allow normal-flight Yaw input while throttle is above motor-off level.
			if (channel_4 > 1500) pid_yaw_setpoint = (channel_4 - 1500) / 2.0f;       // Positive Yaw deviation.
			else if (channel_4 < 1500) pid_yaw_setpoint = (channel_4 - 1500) / 2.0f;  // Negative Yaw deviation.
		}                                                                  // Finish the normal-flight Yaw setpoint calculation.
	}                                                                    // Finish selecting failsafe-locked or normal-flight setpoints.

	// Keep attitude PID completely inactive on the ground until throttle rises above 1050. Failsafe and assisted Landing retain stabilization authority while they explicitly own descent.
	const bool normalThrottlePidReady = channel_3 > kMotorStartThrottle;                 // Activate normal Roll/Pitch/Yaw stabilization only after the pilot/app raises throttle above 1050.
	const bool assistedPidOverride = altitudePidActive();                                // Keep attitude stabilization available during an active assisted Hold/Landing even if its visible throttle target reaches the low region.
	const bool safetyPidOverride = failsafeControlLocked && gPoweredFlightActive;        // Keep failsafe stabilization only after this ARM cycle has actually powered the motors; timer pause/resume must not remove safety authority.
	const bool attitudePidAllowed = start == 2U && !failsafePidDisabled && (normalThrottlePidReady || assistedPidOverride || safetyPidOverride);  // Combine the three explicit sources of PID authority.
	if (attitudePidAllowed) {                                                            // Calculate attitude PID only after one of the valid in-flight authority conditions is true.
		if (imuSampleValid) {                                                            // Use only fresh sensor data for this control-cycle PID calculation.
			if (pidDerivativePrimePending) {                                              // Detect the first valid PID cycle after motor activation, ARM, failsafe entry, or an MPU interruption.
				primePidDerivativeHistory();                                               // Align previous errors to current errors so enabling PID cannot create a D-kick.
				pidDerivativePrimePending = false;                                         // Resume normal derivative calculations from the following valid cycle.
			}                                                                            // Finish the one-cycle derivative-history prime.
			calculatePid();                                                              // Calculate Roll, Pitch, and Yaw corrections from the accepted sensor sample.
		} else {                                                                        // Handle one failed or short MPU6050 transaction without reusing stale PID corrections.
			pid_output_roll = 0.0f;                                                      // Remove stale Roll motor correction for this invalid-sample cycle.
			pid_output_pitch = 0.0f;                                                     // Remove stale Pitch motor correction for this invalid-sample cycle.
			pid_output_yaw = 0.0f;                                                       // Remove stale Yaw motor correction for this invalid-sample cycle.
			pidDerivativePrimePending = true;                                            // Require derivative re-priming when the next fresh MPU sample arrives.
		}                                                                               // Finish selecting valid-sample or invalid-sample PID behavior.
	} else {                                                                           // Handle disarmed, low-throttle armed, or failsafe-touchdown states.
		resetPidRuntime();                                                               // Prevent integral accumulation and guarantee all four attitude PID outputs remain exactly zero.
		pidDerivativePrimePending = start == 2U && !failsafePidDisabled;                 // While armed at low throttle, keep a derivative prime pending for the exact cycle throttle later crosses above 1050.
	}  // Finish gating attitude PID by throttle, assisted flight, failsafe touchdown, and fresh sensor data.

	// Preserve the old altitude behavior exactly: raw application throttle remains the base command and altitude PID is only an additive correction after real lift.
	if (!failsafe_active) {                                      // Leave throttle entirely under failsafe authority once any failsafe reason is active.
		throttle = channel_3;                                      // Stage-one Takeoff uses the app's visible 1000 -> 1500 throttle ramp directly.
		if (altitudePidActive()) {                                 // Connect altitude correction only after the sensor confirms real lift and the app has reached the 1500 base command.
			calculateAltitudePid(filtered_distance);                   // Calculate the preserved P=0.35, I=0.006, D=85 correction toward the active millimeter setpoint.
			throttle += static_cast<int>(pid_output_alt);              // Recreate the previous working relationship: effective throttle = channel_3 + altitude PID correction.
			throttle = constrain(throttle, 1100, 1900);                // Preserve the old assisted throttle bounds.
		} else {                                                   // Keep altitude PID completely inactive during manual flight and the raw-throttle Takeoff stage.
			resetAltitudePidRuntime();                                 // Prevent altitude integral or derivative memory from building before Hold begins.
		}
		failsafeNotifyEffectiveThrottle(static_cast<int16_t>(constrain(throttle, 1000, 1900)));  // Preserve the actual mixer throttle so link loss during Hold cannot jump back to nominal app throttle.
	}

	// ARMED is only a ready state. Normal motor PWM remains fully off until throttle rises above 1050.
	const bool normalMotorDriveRequested = channel_3 > kMotorStartThrottle;  // Require explicit pilot/app throttle above the idle boundary before normal motor output may begin.
	const bool safetyMotorDriveRequested = failsafeControlLocked && gPoweredFlightActive;  // A safety landing may retain motor authority only after this ARM cycle has actually powered the motors; ARMED-idle must never be spun up by failsafe.
	const bool motorDriveAllowed = start == 2U && (normalMotorDriveRequested || safetyMotorDriveRequested);  // Separate ARM state from actual powered-motor state.

	// Count flight time only while the aircraft is genuinely ARMED and the effective throttle driving the flight is above 1100. This pauses automatically during low-throttle ARMED periods without losing the current power-session total.
	constexpr int16_t kFlightTimerThrottle = 1100;                                      // Define the requested real-flight timing threshold independently from the 1050 motor-start boundary.
	const int16_t effectiveTimerThrottle = static_cast<int16_t>(constrain(               // Select the throttle source that actually owns motor power in this control state.
		failsafeControlLocked ? throttle : channel_3, 1000, 2000));                        // During failsafe use its effective descent throttle; otherwise use the real received application/pilot throttle.
	const bool flightTimeShouldRun = start == 2U && gPoweredFlightActive &&              // Require a real ARMED powered-flight state before any time can accumulate.
		effectiveTimerThrottle > kFlightTimerThrottle;                                      // Count only strictly above 1100, exactly as requested.
	if (flightTimeShouldRun) startFlightTimer();                                         // Start or resume this qualifying segment.
	else stopFlightTimer();                                                              // Pause immediately when the qualifying condition becomes false.

	if (motorDriveAllowed) {                 // Build powered motor commands only after a real flight or safety path requests motor authority.
		if (throttle > 1900) throttle = 1900;  // Limit throttle to leave room for full control at high throttle.

		if (failsafePidDisabled) {                                              // Detect touchdown-stage failsafe where stabilization must no longer create motor differences.
			const int16_t equalLandingCommand = constrain(throttle, 1050, 1100);  // Keep the final equal motor command inside the intended powered landing range.
			esc_1 = equalLandingCommand;                                           // Drive motor 1 with exactly the same touchdown command.
			esc_2 = equalLandingCommand;                                           // Drive motor 2 with exactly the same touchdown command.
			esc_3 = equalLandingCommand;                                           // Drive motor 3 with exactly the same touchdown command.
			esc_4 = equalLandingCommand;                                           // Drive motor 4 with exactly the same touchdown command.
		} else {                                                                // Preserve the tested mixer and PID authority during normal powered flight.
			esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw;  // Front-right motor (CCW).
			esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw;  // Rear-right motor (CW).
			esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw;  // Rear-left motor (CCW).
			esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw;  // Front-left motor (CW).

			if (esc_1 < 1100) esc_1 = 1050;  // Preserve the tested minimum running command only after motors are intentionally active.
			if (esc_2 < 1100) esc_2 = 1050;  // Preserve the tested minimum running command only after motors are intentionally active.
			if (esc_3 < 1100) esc_3 = 1050;  // Preserve the tested minimum running command only after motors are intentionally active.
			if (esc_4 < 1100) esc_4 = 1050;  // Preserve the tested minimum running command only after motors are intentionally active.

			if (esc_1 > 2000) esc_1 = 2000;  // Limit motor 1 to the maximum command.
			if (esc_2 > 2000) esc_2 = 2000;  // Limit motor 2 to the maximum command.
			if (esc_3 > 2000) esc_3 = 2000;  // Limit motor 3 to the maximum command.
			if (esc_4 > 2000) esc_4 = 2000;  // Limit motor 4 to the maximum command.
		}                                                                        // Finish selecting failsafe touchdown or normal powered mixer output.
	} else {                           // DISARMED or ARMED-idle: keep every motor physically off.
		esc_1 = 1000;                    // Map motor 1 to zero PWM.
		esc_2 = 1000;                    // Map motor 2 to zero PWM.
		esc_3 = 1000;                    // Map motor 3 to zero PWM.
		esc_4 = 1000;                    // Map motor 4 to zero PWM.
	}  // Finish the powered/off motor-command selection.

	// Map the legacy 1000-to-2000 motor-command scale into the configured brushed-motor PWM range.
	analogWrite(M1, map(esc_1, 1000, 2000, 0, ESPFLIGHT_MOTOR_PWM_RANGE));  // Apply the mapped PWM duty command to motor 1.
	analogWrite(M2, map(esc_2, 1000, 2000, 0, ESPFLIGHT_MOTOR_PWM_RANGE));  // Apply the mapped PWM duty command to motor 2.
	analogWrite(M3, map(esc_3, 1000, 2000, 0, ESPFLIGHT_MOTOR_PWM_RANGE));  // Apply the mapped PWM duty command to motor 3.
	analogWrite(M4, map(esc_4, 1000, 2000, 0, ESPFLIGHT_MOTOR_PWM_RANGE));  // Apply the mapped PWM duty command to motor 4.
}  // Close the current declaration or execution block.

# ESPFlight Firmware v1.0.0 — Validation Baseline

This document describes the validated behavior of the ESPFlight Firmware v1.0.0 release baseline.
It records the release-level safety and control invariants without changing the tested altitude PID gains,
Takeoff/Hold relationship, five-centimeter Landing cutoff, mixer signs, or normal PID tuning.

## Protocol

- `ESPFLIGHT_PROTOCOL_VERSION = 2`
- ESPFlight Application accepts protocol version 2 only.
- High-level flight commands use `flight_command` / `flight_command_ack` with `request_id`.
- Supported commands: `arm`, `disarm`, `takeoff`, `landing`.
- Application ARM state remains telemetry-authoritative.

## ARM / DISARM behavior

- ARM is explicit; throttle position never toggles ARM/DISARM.
- ARM requires throttle <= 1050, healthy pre-arm/failsafe checks, and fresh/safe IMU state.
- Successful ARM enters an ARMED-ready state only.
- ARMED-ready with throttle <= 1050 keeps Roll/Pitch/Yaw PID reset and all four PWM outputs at zero.
- Normal motors and attitude PID become active only after real throttle rises above 1050.
- Flight-time telemetry counts only ARMED segments whose effective throttle is strictly above 1100 and pauses immediately at 1100 or below.
- Returning throttle to <= 1050 stops normal motor PWM again but does not DISARM.
- Explicit DISARM requires throttle <= 1050 and immediately resets assist/PID, sets state DISARMED, and writes PWM=0 to all four motors.
- A failsafe cannot spin motors from an ARMED-ready session that never entered powered flight.

## Flight timer

- ARM-ready time is not counted as flight time.
- Timing starts when the Drone is ARMED and effective throttle first rises strictly above 1100.
- While the Drone remains ARMED, timing pauses whenever effective throttle is 1100 or below.
- Timing resumes when effective throttle rises strictly above 1100 again.
- DISARM stops timing.
- The reported value therefore represents accumulated powered-flight time rather than total ARMED duration.

## Altitude assist compatibility

- Takeoff still requires telemetry-confirmed ARMED state and low throttle.
- ESPFlight Application retains the tested throttle ramp toward 1500.
- Altitude PID still engages after the existing real-lift condition and targets the existing 500-mm relationship.
- Landing retains the five-centimeter raw VL53L0X hard-cut and zero-PWM shutdown path.

## Validation checks

The release baseline was checked for the following control invariants:

```text
EXPLICIT_ARM_MOTOR_OFF_OK
ARMED_IDLE_FAILSAFE_MOTOR_OFF_OK
THROTTLE_STARTS_MOTORS_OK
LOW_THROTTLE_STOPS_MOTORS_WITHOUT_DISARM_OK
EXPLICIT_DISARM_PWM_ZERO_OK
```

Additional source-level checks covered C++ syntax structure, motor-output ownership, command routing,
and protocol consistency.

These checks complement, but do not replace, an Arduino IDE/ESP8266 release compile and propeller-off
hardware validation on the target board. The release candidate should also be validated by real flight
testing with the intended ESPFlight Application version before public distribution.

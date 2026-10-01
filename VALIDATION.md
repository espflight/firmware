# ESPFlight Firmware v1.0.0 — Validation Baseline

This document records the validation baseline for the published ESPFlight Firmware v1.0.0 release.

It describes the release-level control and safety invariants used by the v1.0 platform without changing the tested altitude PID gains, Takeoff/Hold relationship, five-centimeter Landing cutoff, mixer signs, or normal PID tuning.

The published v1.0 platform baseline is:

- ESPFlight Firmware v1.0.0
- ESPFlight Hardware Reference v1.0
- ESPFlight Application v1.0.0
- ESPFlight Protocol 2

## Protocol

- `ESPFLIGHT_PROTOCOL_VERSION = 2`
- ESPFlight Application v1.0.0 accepts protocol version 2.
- High-level flight commands use `flight_command` / `flight_command_ack` with `request_id`.
- Supported commands are `arm`, `disarm`, `takeoff`, and `landing`.
- Application ARM state remains telemetry-authoritative.

## ARM / DISARM behavior

- ARM is explicit; throttle position does not toggle ARM/DISARM.
- ARM requires throttle <= 1050, healthy pre-arm/failsafe checks, and fresh/safe IMU state.
- Successful ARM enters an ARMED-ready state only.
- ARMED-ready with throttle <= 1050 keeps Roll/Pitch/Yaw PID reset and all four PWM outputs at zero.
- Normal motor output and attitude PID become active only after real throttle rises above 1050.
- Flight-time telemetry counts only ARMED segments whose effective throttle is strictly above 1100 and pauses at 1100 or below.
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

## Altitude-assist compatibility

- Takeoff requires telemetry-confirmed ARMED state and low throttle.
- ESPFlight Application v1.0.0 retains the tested throttle ramp toward 1500.
- Altitude PID engages after the existing real-lift condition and targets the existing 500-mm relationship.
- Landing retains the five-centimeter raw VL53L0X hard-cut and zero-PWM shutdown path.
- The Landing-ramp documentation correction for the published v1.0.0 tag is recorded in `ERRATA_v1.0.0.md`.

## Validation checks recorded for the release baseline

The release baseline includes the following control-invariant checks:

```text
EXPLICIT_ARM_MOTOR_OFF_OK
ARMED_IDLE_FAILSAFE_MOTOR_OFF_OK
THROTTLE_STARTS_MOTORS_OK
LOW_THROTTLE_STOPS_MOTORS_WITHOUT_DISARM_OK
EXPLICIT_DISARM_PWM_ZERO_OK
```

Additional source-level checks covered C++ syntax structure, motor-output ownership, command routing, and protocol consistency.

These checks document the release baseline; they do not certify arbitrary modified firmware, custom hardware, or third-party builds.

## Rebuilding and validating today

The published `v1.0.0` tag remains the immutable release reference. The current `main` branch may contain documentation and repository-maintenance improvements made after release.

When rebuilding the v1.0.0 firmware:

1. Use the pinned toolchain documented in `BUILDING.md`.
2. Confirm the firmware and protocol identity remain `1.0.0` and `2`.
3. Compile successfully for the documented ESP8266 target.
4. Perform propeller-off bench validation on the actual hardware.
5. Verify motor order and direction, IMU orientation, ARM/DISARM behavior, control directions, telemetry, and failsafe behavior.
6. Use ESPFlight Application v1.0.0 for the validated v1.0 platform combination.

Any change to flight-control logic, failsafe thresholds, motor mapping, sensor assumptions, PWM timing, protocol behavior, or hardware interfaces must be treated as a new tested variant rather than silently represented as the original v1.0.0 release.

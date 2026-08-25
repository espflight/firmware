# ESPFlight Assisted Takeoff / Altitude Hold / Landing

This document describes the validated assisted-flight control relationship used by
ESPFlight Firmware v1.0.0, including VL53L0X altitude sensing, safety, telemetry,
WebSocket ownership, and ACK behavior.

## Takeoff

- User ARM state must already be confirmed by firmware telemetry.
- User throttle must be <= 1050.
- VL53L0X must be detected and have at least three fresh valid physical samples.
- Application sends `flight_command: takeoff` with a positive `request_id`.
- Firmware ACKs only after rechecking ownership, ARM, failsafe, sensor, and throttle.
- After the ACK, the application moves the visible throttle from its current
  minimum value to 1500 over a short controlled ramp.
- During this initial stage firmware uses raw `channel_3`; altitude PID output is
  forced to zero and is not added to throttle.
- When filtered VL53L0X height reaches 200 mm and channel_3 has reached about
  1500, firmware switches to Altitude Hold.

## Altitude Hold

- Application keeps the visible throttle locked at 1500.
- Firmware target is 500 mm (50 cm).
- Preserved tuning: P=0.35, I=0.006, D=85, output limit=800.
- Preserved Kalman tuning: Q=0.3, R=9.0. The scalar Kalman implementation is contained directly in `altitude.cpp` because it is private to altitude sensing.
- Physical ranging remains 50 ms; the retained valid range is fed through the
  Kalman update at the flight-loop cadence, matching the previous behavior.
- Effective mixer throttle is `1500 + altitude PID correction` and is constrained
  to the existing 1100..1900 assisted range.
- Derivative history is primed when Hold engages, preventing a one-cycle maximum
  D kick without changing the tested PID gains.

## Landing

- Application sends `flight_command: landing` and waits for the matching ACK.
- After ACK, application visibly ramps throttle from 1500 to 1050 over about 6 seconds.
- Firmware preserves the proven relationship `target_mm = channel_3 - 1000`, bounded to 50..500 mm.
- Altitude PID now stays active below the old 200-mm takeoff activation threshold; it is disconnected only after physical touchdown is confirmed.
- Moving the landing setpoint no longer produces a derivative kick: derivative history is compensated by the exact setpoint delta, so the D term continues to react to real altitude motion instead of the commanded ramp itself.
- The release safety logic hard-cuts Landing when two fresh raw VL53L0X measurements are <= 50 mm.
- This five-centimeter cutoff does not wait for the application throttle ramp to finish: firmware resets assist/PID, DISARMs, and writes zero PWM immediately.
- As a secondary endpoint, if Landing reaches `channel_3 <= 1050` before the range cutoff is seen, firmware also performs the same hard DISARM so motors cannot remain idling at 1050.
- ARM/DISARM is explicit and never toggled by Throttle. During assisted Landing, firmware owns the descent and performs the final hard DISARM itself when the configured landing cutoff/fallback condition is reached.
- If the normal assisted Landing cannot reach the floor within 12 seconds, the existing deterministic failsafe descent takes over instead of leaving the landing state active indefinitely.

## Safety boundaries

- Existing link-loss / battery failsafe always has priority.
- One isolated invalid range does not abort a flight; active sensor health uses
  a 300 ms last-valid-sample timeout.
- Takeoff cannot start until three independent fresh valid samples are available.
- If Takeoff remains below the real-lift threshold for more than 5 seconds,
  firmware transfers to the existing failsafe landing/disarm path.
- If assisted filtered height reaches 850 mm, firmware transfers to failsafe
  landing before the 1000 mm accepted range ceiling.
- If fresh altitude data is lost for more than 300 ms during assistance,
  altitude PID is disconnected and existing failsafe landing takes control.
- Manual flight remains available when VL53L0X is absent.

## Wire protocol

Application -> firmware:

```json
{"type":"flight_command","cmd":"arm","request_id":1}
```

```json
{"type":"flight_command","cmd":"disarm","request_id":2}
```

```json
{"type":"flight_command","cmd":"takeoff","request_id":3}
```

```json
{"type":"flight_command","cmd":"landing","request_id":4}
```

Firmware ACK example:

```json
{"type":"flight_command_ack","cmd":"arm","request_id":1,"ok":true,"message":"arm_accepted"}
```

## Final landing cutoff

During an assisted Landing, motor power is removed as soon as two fresh raw VL53L0X measurements are at or below 50 mm. Altitude assistance is cleared, all PID runtime state is reset, the aircraft is disarmed, and PWM on all four motor pins is written to zero immediately. As a secondary safety endpoint, reaching channel_3 <= 1050 during Landing also hard-stops the motors so they can never remain idling at 1050 indefinitely.


## ARM-ready motor behavior

- ARM only changes the flight state to `ARMED`; all four motor PWM outputs remain zero while `channel_3 <= 1050`.
- Normal attitude PID remains reset in the same low-throttle ARMED-ready state.
- Motor PWM and normal attitude PID become active only after real Throttle rises above `1050`.
- Flight time is counted only while the aircraft remains ARMED and the effective motor-owning Throttle is strictly above `1100`; low-throttle ARMED time is excluded.
- DISARM is an explicit high-level command and immediately resets PID and forces all four PWM outputs to zero.

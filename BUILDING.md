# ESPFlight Firmware v1.0.0 — Reproducible Build Baseline

Use the following pinned toolchain for the v1.0.0 release build. Keeping these
versions fixed avoids unexpected behavior changes caused by later library or
ESP8266 core updates.

## Board / core

- Target board: LOLIN(WEMOS) D1 R2 & mini / ESP8266
- ESP8266 Arduino Core: **3.1.2**
- CPU frequency: **160 MHz**

## Required libraries

- ArduinoJson: **6.21.6**
- ESPAsyncTCP (ESP8266): **2.0.0**
- ESPAsyncWebServer: **3.6.0**

`ESPAsyncTCP` and `ESPAsyncWebServer` are maintained under the ESP32Async
organization. Do not silently substitute a different major version for a
release build.

## Release configuration

Before compiling, edit `config.h` and replace:

```cpp
#define ESPFLIGHT_WIFI_SSID "YOUR_WIFI_SSID"
#define ESPFLIGHT_WIFI_PASSWORD "YOUR_WIFI_PASSWORD"
```

Keep these release identity values unchanged for the v1.0.0 build:

```cpp
#define ESPFLIGHT_FIRMWARE_VERSION "1.0.0"
#define ESPFLIGHT_PROTOCOL_VERSION 2U
```

## Explicit ARM / motor-off semantics

Protocol v2 separates ARM state from motor power:

- ARM/DISARM is requested explicitly by the ESPFlight Application and acknowledged by firmware.
- ARM requires throttle <= 1050 plus healthy link, failsafe, battery and IMU checks.
- ARMED with throttle <= 1050 keeps attitude PID reset and all motor PWM outputs at zero.
- Normal motor output and attitude PID begin only after throttle rises above 1050.
- Flight time is accumulated only while the aircraft is ARMED and the effective throttle is strictly above 1100; it pauses at 1100 or below and resumes on the next qualifying segment.
- Returning throttle to <= 1050 stops normal motor PWM but keeps the aircraft ARMED.
- Explicit DISARM at low throttle immediately resets PID/assist and writes zero PWM to all four motors.
- A failsafe cannot power motors from an ARMED-ready state that never entered powered flight.

## Flight timing / PWM values

The release baseline keeps the flight-tested values in `config.h`:

- Control loop: **250 Hz** (`4000 us`)
- Motor PWM range: **0..1023**
- Motor PWM frequency: **4000 Hz**

Do not change PID gains, PWM frequency, control-loop timing, MPU configuration,
or failsafe thresholds when producing the v1.0.0 reference binary unless the
result is treated as a separately tested firmware variant.

## Optional VL53L0X assisted flight

The v1.0.0 altitude-assist variant additionally requires:

- Adafruit VL53L0X: **1.2.5**

The sensor is optional for ordinary manual flight. Takeoff/Landing assistance is
available only when the VL53L0X is detected and fresh valid range data is ready.
The sensor shares the existing I2C bus with the MPU6050.

The restored assisted Takeoff control relationship is intentionally split
between the ESPFlight Application and firmware:

1. Firmware accepts Takeoff only while ARMED, failsafe-clear, VL53L0X-ready,
   and real incoming throttle is <= 1050.
2. After the matching ACK, the application visibly ramps channel_3 to 1500.
3. Firmware uses that raw throttle during the initial lift stage; altitude PID
   is disconnected until filtered distance reaches 200 mm.
4. Once airborne and channel_3 is 1500, firmware activates the
   preserved altitude PID with a 500 mm target. Effective throttle is then
   `channel_3 + pid_output_alt`, bounded to the existing 1100..1900 range.
5. During assisted Landing, the application ramps channel_3 from 1500 toward
   1050 and firmware uses the old relationship `target_mm = channel_3 - 1000`
   while meaningful height remains.

Do not replace this relationship with a learned or fixed "hover throttle";
1500 is the application base/setpoint command from the previously working
controller, not an assumption about physical hover power.

/*
  =============================================================================
  ESPFlight Firmware Lifecycle Module
  =============================================================================

  This module owns the complete firmware setup and main loop lifecycle.
  The initialization order follows the flight-tested modular firmware while
  retaining the current v1.0.0 safety, telemetry, and failsafe behavior.

  Author: Ali KarimiZadeh
  Author Website: https://karimizadeh.site
  ESPFlight Website: https://espflight.com
  License: MIT
*/

#include <Arduino.h>            // Provide Arduino timing, GPIO, PWM, watchdog, and ESP helper functions.
#include <Wire.h>               // Provide the I2C bus used by the MPU6050.
#include <ESP8266WiFi.h>        // Provide the tested ESP8266 station-mode Wi-Fi implementation.
#include <ESPAsyncTCP.h>        // Provide asynchronous TCP support required by the WebSocket server.
#include <ESPAsyncWebServer.h>  // Provide the tested asynchronous HTTP server.
#include <AsyncWebSocket.h>     // Provide the tested real-time WebSocket transport.
#include <ArduinoJson.h>        // Provide fixed-size JSON telemetry serialization.
#include "firmware.h"           // Provide the public firmware setup and loop entry points.
#include "config.h"             // Provide the preserved pins, ports, credentials, PWM, and timing values.
#include "indicators.h"         // Provide the original startup melody and error LED behavior.
#include "flight_control.h"     // Provide shared flight state and the original 250-Hz flight-control routine.
#include "pid.h"                // Provide the Roll, Pitch, and Yaw PID controller state.
#include "imu.h"                // Provide the tested MPU6050 setup, calibration, and reading functions.
#include "failsafe.h"           // Provide link-loss and low-voltage gradual landing management.
#include "altitude.h"           // Provide optional VL53L0X ranging and the restored flight-tested altitude-hold path.
#include "network.h"            // Provide the tested Wi-Fi, WebSocket, UDP, PID storage, and receiver functions.

extern "C" {                 // Use C linkage for the ESP8266 SDK CPU-frequency function.
#include "user_interface.h"  // Provide system_update_cpu_freq for the tested 160-MHz configuration.
}  // Finish the ESP8266 SDK declaration block.

ADC_MODE(ADC_VCC);  // Configure ESP.getVcc() to read the ESP8266 supply-voltage proxy used by failsafe.

void firmwareSetup() {   // Initialize the firmware using the same tested order as the original project.
  Serial.begin(115200);  // Start serial communication at the original diagnostic baud rate.

  delay(500);  // Preserve the original delay that allows the serial monitor to become ready.

  Serial.println();  // Print an empty line before the boot messages.

  Serial.println("System booting...");  // Print the original system boot message.
  Serial.printf("ESPFlight firmware %s (protocol %u)\n", ESPFLIGHT_FIRMWARE_VERSION, ESPFLIGHT_PROTOCOL_VERSION);  // Publish the exact firmware and protocol release at boot.

  system_update_cpu_freq(ESPFLIGHT_CPU_FREQUENCY_MHZ);  // Preserve the tested 160-MHz ESP8266 CPU frequency.

  pinConfig();  // Configure the original LED and motor pins before any subsystem can use them.

  analogWriteRange(ESPFLIGHT_MOTOR_PWM_RANGE);  // Configure the ten-bit motor PWM range before writing motor outputs.

  analogWriteFreq(ESPFLIGHT_MOTOR_PWM_FREQUENCY);  // Preserve the tested four-kilohertz motor PWM frequency.

  failsafeInit();  // Establish the original deterministic disarmed and zero-PWM startup state.

  Wire.begin(ESPFLIGHT_I2C_SDA_PIN, ESPFLIGHT_I2C_SCL_PIN);  // Start I2C using the original D6 SDA and D5 SCL pins.

  Wire.setClock(400000);  // Preserve the original 400-kilohertz I2C clock.

  delay(100);  // Preserve the original delay that allows the MPU6050 to become ready.

  Serial.println("Checking MPU6050...");  // Print the original MPU6050 detection message.

  Wire.beginTransmission(0x68);  // Start communication with the MPU6050 at its original I2C address.

  i2cError = Wire.endTransmission();  // Store the original I2C detection result.

  while (i2cError != 0) {  // Continue retrying until the mandatory MPU6050 responds.
    i2cError = 2;          // Preserve the original custom MPU6050 connection error code.

    errorSignal();  // Display the original LED error signal while the sensor is unavailable.

    delay(4);  // Preserve the original short retry delay.

    yield();  // Allow ESP8266 background processing during the sensor retry loop.

    Wire.beginTransmission(0x68);  // Retry communication with the MPU6050.

    i2cError = Wire.endTransmission();  // Store the newest I2C detection result.
  }                                     // Finish waiting for the mandatory MPU6050.

  Serial.println("MPU6050 OK");  // Print the original successful sensor detection message.

  altitudeInit();  // Detect the optional VL53L0X exactly on the shared I2C bus; manual flight remains available when it is absent.

  playStartupMelody();  // Play the exact original startup melody before beginning Wi-Fi connection.

  WiFi.setOutputPower(18.5);  // Preserve the original tested Wi-Fi output power.

  WiFi.persistent(false);  // Avoid unnecessary flash writes when reconnecting to the configured network.

  WiFi.setAutoReconnect(false);  // Keep reconnect attempts under the firmware safe-state retry manager.

  WiFi.mode(WIFI_STA);  // Preserve the original station-mode configuration.

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);  // Start the first non-permanent connection attempt.

  Serial.print("Connecting to WiFi");  // Print the Wi-Fi connection progress message.

  const uint32_t wifiConnectStartedMs = millis();  // Record the start of the bounded initial Wi-Fi wait.

  while (WiFi.status() != WL_CONNECTED && static_cast<uint32_t>(millis() - wifiConnectStartedMs) < ESPFLIGHT_WIFI_CONNECT_TIMEOUT_MS) {  // Wait only until connected or the configured timeout expires.
    delay(500);                                                                                                                          // Preserve the original half-second connection check interval.

    Serial.print(".");  // Print one connection progress dot.

    yield();  // Allow ESP8266 Wi-Fi background tasks while waiting for connection.
  }           // Finish the bounded initial Wi-Fi wait.

  Serial.println();  // Move to a new line after the connection progress dots.

  wifiWasConnected = WiFi.status() == WL_CONNECTED;          // Publish the result to the non-blocking reconnect manager.
  wifiDisconnectedSince = wifiWasConnected ? 0U : millis();  // Start disconnected timing only when the initial attempt timed out.

  if (wifiWasConnected) {                                                    // Report a successful initial Wi-Fi connection.
    Serial.print("Connected! IP: ");                                         // Print the successful Wi-Fi connection message.
    Serial.println(WiFi.localIP());                                          // Print the assigned local IP address.
  } else {                                                                   // Continue safe firmware initialization without waiting forever for Wi-Fi.
    Serial.println("WiFi connection timed out; background retry enabled.");  // Explain that non-blocking retries will continue in the main loop.
  }                                                                          // Finish reporting the initial Wi-Fi result.

  bootSessionId = os_random() ^ ESP.getChipId() ^ ESP.getCycleCount() ^ micros();  // Mix hardware randomness and boot-time values into one boot identifier.
  if (bootSessionId == 0U) {                                                       // Keep zero reserved for invalid or unavailable boot identity.
    bootSessionId = 1U;                                                            // Replace the reserved value with the first valid identifier.
  }                                                                                // Finish validating the boot identifier.

  udp.begin(udpPort);  // Start UDP discovery on the original port.

  initPidStorage();  // Load the saved attitude PID gains from EEPROM before flight starts.

  ws.onEvent(onWsEvent);  // Register the original joystick, PID, ownership, and failsafe WebSocket handler.

  startWebServer();  // Attach the WebSocket and start the original asynchronous server.

  Serial.println("Configuring MPU6050...");  // Announce the verified MPU6050 register setup stage.

  while (!gyroSetup()) {                                               // Refuse to continue toward calibration until every required MPU register is written and read-back verified.
    Serial.println("MPU6050 setup verification failed; retrying...");  // Report the boot-time configuration failure for diagnostics.
    errorSignal();                                                     // Preserve the existing visible I2C error indication while setup is not trustworthy.
    delay(20);                                                         // Leave a short recovery interval before retrying the complete verified setup sequence.
    yield();                                                           // Allow ESP8266 background processing while the aircraft remains safely disarmed.
  }                                                                    // Finish waiting for a completely verified MPU6050 configuration.

  Serial.println("MPU6050 configuration verified");  // Confirm that all mandatory registers matched their requested values.

  calibrateGyro();  // Run the original gyro calibration routine only after verified sensor configuration.

  if (WiFi.status() == WL_CONNECTED) {  // Broadcast discovery only when the station has a valid address.
    sendUdpInfo();                      // Broadcast the ESPFlight IP and MAC discovery packet.
  }                                     // Finish the guarded startup discovery broadcast.

  ESP.wdtEnable(WDTO_250MS);  // Enable the original 250-millisecond ESP8266 watchdog.

  Serial.println("Setup completed.");  // Print the setup completion message after all mandatory subsystems are ready.
}  // Finish the complete firmware setup sequence.

void firmwareLoop() {        // Run the preserved cooperative flight-control and network loop.
  loopStartTime = micros();  // Record the beginning of the current loop iteration.

  nowMilis = millis();  // Record the current millisecond timestamp used by all modules.

  if (loopStartTime - lastLoopStartTime >= loopTime) {  // Run the high-priority flight controller at the original 250-Hz rate.
    lastLoopStartTime += loopTime;                      // Advance the fixed-rate schedule without accumulating drift.

    ESP.wdtFeed();  // Feed the watchdog before the flight-control calculations.

    flightControl250Hz();  // Execute the original attitude, arming, failsafe, mixer, and motor routine.

    ESP.wdtFeed();  // Feed the watchdog after the flight-control calculations.

    yield();  // Allow ESP8266 networking background tasks after the high-priority update.
  }           // Finish the scheduled 250-Hz control update.

  readAltitude(nowMilis);  // Preserve the previous implementation's physical 50-ms ranging schedule and repeated Kalman filtering between new samples.

  errorSignal();  // Preserve the original blue LED I2C error indication.

  ESP.wdtFeed();  // Feed the watchdog after the indicator update.

  static uint32_t lastMainSend = 0;                                  // Store the newest main telemetry send timestamp.
  static bool lastReportedArmed = false;                             // Remember the ARM state included in the newest telemetry packet.
  static uint32_t lastReportedFlightSessionId = 0U;                  // Remember the flight session included in the newest telemetry packet.
  static bool lastReportedFailsafeActive = false;                    // Remember whether failsafe was active in the newest telemetry packet.
  static uint8_t lastReportedFailsafeReason = FAILSAFE_REASON_NONE;  // Remember the newest failsafe reason mask.
  static bool lastReportedFailsafeLanding = false;                   // Remember whether a gradual failsafe landing was running.
  static bool lastReportedBatteryLockout = false;                    // Remember whether low voltage blocked re-arming.
  static StaticJsonDocument<832> doc;                                // Keep fixed telemetry JSON storage off the ESP8266 call stack with release identity fields included.
  static char out[832];                                              // Reserve fixed room for identity, failsafe, release, and system telemetry.

  const bool armedNow = start == 2U;                                                                                                                                                                                                         // Convert the internal flight-state variable to the public Boolean ARM state.
  const bool failsafeLandingNow = failsafeLandingActive();                                                                                                                                                                                   // Read the current gradual failsafe landing state once.
  const bool batteryLockoutNow = failsafeBatteryLockoutActive();                                                                                                                                                                             // Read the current battery lockout state once.
  const bool flightStateChanged = armedNow != lastReportedArmed || flightSessionId != lastReportedFlightSessionId;                                                                                                                           // Detect every ARM, DISARM, or new flight session immediately.
  const bool failsafeStateChanged = failsafe_active != lastReportedFailsafeActive || failsafe_reason != lastReportedFailsafeReason || failsafeLandingNow != lastReportedFailsafeLanding || batteryLockoutNow != lastReportedBatteryLockout;  // Detect every failsafe transition immediately.
  constexpr uint32_t kDisarmedTelemetryIntervalMs = 200U;                                                                                                                                                                                  // Refresh altitude and status at 5 Hz while safely disarmed for a responsive live sensor readout.
  constexpr uint32_t kArmedTelemetryIntervalMs = 1000U;                                                                                                                                                                                      // Limit telemetry to 1 Hz while armed so flight-control and Wi-Fi timing keep priority.
  const uint32_t telemetryIntervalMs = armedNow ? kArmedTelemetryIntervalMs : kDisarmedTelemetryIntervalMs;                                                                                                                                // Select the telemetry rate directly from the real firmware ARM state.
  const bool periodicTelemetryDue = (uint32_t)(nowMilis - lastMainSend) >= telemetryIntervalMs;                                                                                                                                             // Refresh faster on the ground and exactly once per second in flight.

  if (flightStateChanged || failsafeStateChanged || periodicTelemetryDue) {  // Send immediately on flight or failsafe state changes and otherwise at the ARM-aware telemetry interval.
    lastMainSend = nowMilis;                                                 // Record the newest main telemetry send time.
    lastReportedArmed = armedNow;                                            // Remember the ARM state represented by this packet.
    lastReportedFlightSessionId = flightSessionId;                           // Remember the flight session represented by this packet.
    lastReportedFailsafeActive = failsafe_active;                            // Remember the failsafe active state represented by this packet.
    lastReportedFailsafeReason = failsafe_reason;                            // Remember the failsafe reason represented by this packet.
    lastReportedFailsafeLanding = failsafeLandingNow;                        // Remember the landing state represented by this packet.
    lastReportedBatteryLockout = batteryLockoutNow;                          // Remember the battery lockout state represented by this packet.

    doc.clear();  // Remove every field from the previous telemetry packet before rebuilding it.

    doc["type"] = "main";                        // Identify the combined application telemetry packet.
    doc["armed"] = armedNow;                     // Report the real firmware ARM state without adding a separate acknowledgement protocol.
    doc["mac"] = getMacAddress();                // Report the verified board identity used for duplicate-safe application logging.
    doc["boot_session_id"] = bootSessionId;      // Distinguish this firmware boot from every earlier power cycle or restart.
    doc["flight_session_id"] = flightSessionId;  // Distinguish the powered-board timing session within this boot.

    if (count > 0) {                                     // Include loop timing only when valid samples have accumulated.
      unsigned long avg = total / count;                 // Calculate the average measured loop duration in microseconds.
      float hz = (avg > 0) ? (1000000.0f / avg) : 0.0f;  // Convert the average duration into loop frequency.

      JsonObject loopObj = doc.createNestedObject("loopTime");  // Create the original loop timing telemetry section.
      loopObj["type"] = "loopTime";                             // Preserve the original loop timing message identifier.
      loopObj["curr"] = Time;                                   // Publish the newest measured loop duration.
      loopObj["avg"] = avg;                                     // Publish the average measured loop duration.
      loopObj["hz"] = hz;                                       // Publish the calculated loop frequency.

      total = 0;  // Reset the loop duration accumulator after reporting.
      count = 0;  // Reset the loop sample counter after reporting.
    }             // Finish adding loop timing telemetry.

    JsonObject flightObj = doc.createNestedObject("flightTime");  // Create the real-flight-time telemetry section counted only during qualifying ARMED + throttle-above-1100 segments.
    flightObj["type"] = "flightTime";                             // Preserve the original flight-time message identifier.
    flightObj["seconds"] = getFlightTimeSeconds();                // Publish only the current or most recently completed flight-session duration.

    JsonObject altitudeObj = doc.createNestedObject("altitude");  // Publish optional altitude-assist state without changing the existing main telemetry envelope.
    altitudeObj["available"] = vl53_available;                       // Report whether VL53L0X was detected at boot.
    altitudeObj["ready"] = altitudeSensorReady(nowMilis);            // Report whether several fresh valid physical range samples are currently available.
    altitudeObj["healthy"] = altitudeSensorHealthy(nowMilis);            // Report true active-flight range freshness while tolerating isolated invalid physical samples.
    altitudeObj["mm"] = filtered_distance;                           // Publish the filtered height estimate in millimeters.
    altitudeObj["target_mm"] = pid_alt_setpoint;                     // Publish the active altitude PID target, normally 500 mm in Hold.
    altitudeObj["pid"] = pid_output_alt;                             // Publish the newest additive altitude throttle correction for diagnostics.
    altitudeObj["assist_active"] = altitudeAssistMode() != ALTITUDE_ASSIST_OFF;  // Report whether Takeoff/Hold/Landing assistance is currently active.
    altitudeObj["mode"] = altitudeAssistModeName();                  // Publish off/takeoff/hold/landing as a stable short label.

    JsonObject failsafeObj = doc.createNestedObject("failsafe");                                               // Create the public failsafe telemetry section.
    failsafeObj["active"] = failsafe_active;                                                                   // Report whether any failsafe reason is currently active.
    failsafeObj["reason"] = failsafe_reason;                                                                   // Report the complete reason bit mask for forward-compatible application handling.
    failsafeObj["link_loss"] = (failsafe_reason & static_cast<uint8_t>(FAILSAFE_REASON_LINK_LOSS)) != 0U;      // Report link-loss involvement directly.
    failsafeObj["low_battery"] = (failsafe_reason & static_cast<uint8_t>(FAILSAFE_REASON_LOW_BATTERY)) != 0U;  // Report low-battery involvement directly.
    failsafeObj["landing"] = failsafeLandingNow;                                                               // Report whether the gradual failsafe landing ramp is still running.
    failsafeObj["battery_lockout"] = batteryLockoutNow;                                                        // Report whether re-arming remains blocked until power cycling.

    JsonObject sysObj = doc.createNestedObject("system");  // Create the original system telemetry section.
    sysObj["type"] = "system";                             // Preserve the original system message identifier.
    sysObj["rssi"] = getRssi();                            // Publish the current Wi-Fi signal strength.
    sysObj["bat"] = battery_voltage;                       // Publish the calibrated ESP supply-voltage proxy.
    sysObj["firmware_version"] = ESPFLIGHT_FIRMWARE_VERSION;  // Publish the exact firmware release used by this board.
    sysObj["protocol_version"] = ESPFLIGHT_PROTOCOL_VERSION;  // Publish the protocol revision so the application can detect incompatible releases.

    size_t n = serializeJson(doc, out, sizeof(out));  // Serialize the complete telemetry document into the fixed buffer.
    if (n > 0) {                                      // Confirm that serialization produced an output packet.
      ws.textAll(out);                                // Broadcast the telemetry packet to every connected application client.
    }                                                 // Finish sending the main telemetry packet.
  }                                                   // Finish the ARM-aware telemetry update.

  ws.cleanupClients();  // Preserve the original asynchronous WebSocket queue cleanup.

  handleUdpDiscovery(nowMilis);  // Repeat UDP discovery for app startup and reconnection when flight control is not actively owned.

  ESP.wdtFeed();  // Feed the watchdog after telemetry and WebSocket maintenance.

  if (i2cError > 20) {             // Stop only after more than twenty consecutive MPU6050 communication failures.
    flightControlEmergencyStop();  // Use the single flight-control-owned motor shutdown path.
  }                                // Finish the consecutive I2C emergency shutdown check.

  handleWifiReconnect(nowMilis);  // Preserve the original Wi-Fi disconnection recovery behavior.

  if (Time > 50000) {                      // Preserve the original safety stop after an abnormally long control-loop duration.
    flightControlEmergencyStop();          // Use the single flight-control-owned motor shutdown path.
    digitalWrite(LED, !digitalRead(LED));  // Toggle the status LED to indicate the timing fault.
  }                                        // Finish the original timing safety check.

  unsigned long elapsed = micros() - loopStartTime;  // Measure work completed during the current loop iteration.

  if (elapsed < loopTime) {                 // Determine whether time remains before the fixed loop period ends.
    delayMicroseconds(loopTime - elapsed);  // Wait for the remaining portion of the fixed loop period.
  }                                         // Finish maintaining the original fixed loop timing.

  ESP.wdtFeed();  // Feed the watchdog before completing the current loop iteration.

  Time = micros() - loopStartTime;  // Measure the final duration of the complete loop iteration.

  total += Time;  // Accumulate loop duration for the next telemetry average.
  count++;        // Count the completed loop sample.
}  // Finish the main firmware loop iteration.

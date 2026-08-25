#ifndef ESPFLIGHT_CONFIG_H  // Prevent this configuration header from being included more than once.
#define ESPFLIGHT_CONFIG_H  // Define the include guard used by this configuration header.

#include <Arduino.h>  // Provide Arduino integer types and board pin aliases.

// -----------------------------------------------------------------------------
// Network configuration
// -----------------------------------------------------------------------------
#define ESPFLIGHT_WIFI_SSID "YOUR_WIFI_SSID"          // Replace with the Wi-Fi network used by the ESPFlight Application before compiling.
#define ESPFLIGHT_WIFI_PASSWORD "YOUR_WIFI_PASSWORD"  // Replace with the Wi-Fi password used by the ESPFlight Application before compiling.
#define ESPFLIGHT_UDP_PORT 4210U                   // Preserve the UDP discovery port used by the application.
#define ESPFLIGHT_WEBSOCKET_PORT 81U               // Preserve the asynchronous WebSocket server port.
#define ESPFLIGHT_WIFI_CONNECT_TIMEOUT_MS 15000UL  // Stop the initial blocking Wi-Fi wait after fifteen seconds.
#define ESPFLIGHT_WIFI_RETRY_INTERVAL_MS 5000UL    // Retry Wi-Fi connection every five seconds while safely disarmed.

// -----------------------------------------------------------------------------
// Release identity
// -----------------------------------------------------------------------------
#define ESPFLIGHT_FIRMWARE_VERSION "1.0.0"  // Publish the firmware release version to diagnostics and telemetry.
#define ESPFLIGHT_PROTOCOL_VERSION 2U        // Protocol v2 adds explicit ARM/DISARM commands and motor-off ARMED-ready semantics.

// -----------------------------------------------------------------------------
// Hardware configuration
// -----------------------------------------------------------------------------
#define ESPFLIGHT_LED_PIN 2       // Preserve the status LED GPIO assignment.
#define ESPFLIGHT_MOTOR_1_PIN 5   // Preserve the front-right motor GPIO assignment.
#define ESPFLIGHT_MOTOR_2_PIN 4   // Preserve the rear-right motor GPIO assignment.
#define ESPFLIGHT_MOTOR_3_PIN 13  // Preserve the rear-left motor GPIO assignment.
#define ESPFLIGHT_MOTOR_4_PIN 15  // Preserve the front-left motor GPIO assignment.
#define ESPFLIGHT_BUZZER_PIN 3    // Preserve the startup and failsafe buzzer GPIO assignment.
#define ESPFLIGHT_I2C_SDA_PIN D6  // Preserve the MPU6050 SDA board pin assignment.
#define ESPFLIGHT_I2C_SCL_PIN D5  // Preserve the MPU6050 SCL board pin assignment.

// -----------------------------------------------------------------------------
// Tested timing and PWM configuration
// -----------------------------------------------------------------------------
#define ESPFLIGHT_CPU_FREQUENCY_MHZ 160U     // Preserve the tested ESP8266 CPU frequency.
#define ESPFLIGHT_MOTOR_PWM_RANGE 1023U      // Configure the ten-bit motor PWM range.
#define ESPFLIGHT_MOTOR_PWM_FREQUENCY 4000U  // Preserve the tested 4-kilohertz motor PWM frequency.
#define ESPFLIGHT_CONTROL_LOOP_US 4000UL     // Preserve the tested 250-Hz control-loop period.

#endif  // Close the ESPFLIGHT_CONFIG_H include guard.

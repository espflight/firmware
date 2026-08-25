#ifndef NETWORK_H  // Prevent this network header from being included more than once.
#define NETWORK_H  // Define the include guard used by this network header.

#include <ESPAsyncWebServer.h>  // Import declarations supplied by <ESPAsyncWebServer.h>.
#include <ESP8266WiFi.h>        // Import declarations supplied by <ESP8266WiFi.h>.
#include <WiFiUdp.h>            // Import declarations supplied by <WiFiUdp.h>.
#include <Arduino.h>            // Import declarations supplied by <Arduino.h>.

extern WiFiUDP udp;       // Declare the shared udp symbol defined by another module.
extern uint16_t udpPort;  // Declare the shared udpPort symbol defined by another module.

// WiFi credentials (SSID & Password)
extern const char *WIFI_SSID;      // Declare the shared WIFI_SSID symbol defined by another module.
extern const char *WIFI_PASSWORD;  // Declare the shared WIFI_PASSWORD symbol defined by another module.

// Server & WebSocket

extern AsyncWebServer server;  // Declare the shared server symbol defined by another module.
extern AsyncWebSocket ws;      // Declare the shared ws symbol defined by another module.

// Event handler
void onWsEvent(AsyncWebSocket *socket, AsyncWebSocketClient *client,      // Declare the WebSocket event callback interface.
               AwsEventType type, void *arg, uint8_t *data, size_t len);  // Execute this statement as part of the current operation.

// Helper to start the server
void startWebServer();  // Invoke or continue startWebServer for the current operation.

void handleWifiReconnect(unsigned long now);  // Invoke or continue handleWifiReconnect for the current operation.

String getMacAddress();  // Declare value for use in the current scope.

void sendUdpInfo();                          // Broadcast the ESPFlight IP and MAC immediately.
void handleUdpDiscovery(unsigned long now);  // Repeat discovery while disarmed or without an active controller.

void initPidStorage();  // Initialize EEPROM-backed PID storage and load a validated saved gain set when available.

int getRssi();  // Declare value for use in the current scope.

#endif  // Close the NETWORK_H include guard.

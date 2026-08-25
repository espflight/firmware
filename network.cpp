#include <ArduinoJson.h>     // Import declarations supplied by <ArduinoJson.h>.
#include <EEPROM.h>          // Import declarations supplied by <EEPROM.h>.
#include <math.h>            // Import declarations supplied by <math.h>.
#include <stddef.h>          // Import declarations supplied by <stddef.h>.
#include "network.h"         // Provide the preserved Wi-Fi, WebSocket, UDP, and receiver API.
#include "config.h"          // Provide the preserved network credentials and ports.
#include "flight_control.h"  // Provide the preserved receiver channels and flight state.
#include "pid.h"             // Import declarations supplied by "pid.h".
#include "failsafe.h"        // Connect validated controller packets and disconnects to the failsafe manager.
#include "altitude.h"        // Validate Takeoff/Landing commands against the restored VL53L0X altitude-assist state.

// WiFi settings
const char *WIFI_SSID = ESPFLIGHT_WIFI_SSID;          // Initialize WIFI_SSID with its required starting value.
const char *WIFI_PASSWORD = ESPFLIGHT_WIFI_PASSWORD;  // Initialize WIFI_PASSWORD with its required starting value.

WiFiUDP udp;                            // Create the UDP transport used for device discovery.
uint16_t udpPort = ESPFLIGHT_UDP_PORT;  // Use the configurable UDP discovery port.

AsyncWebServer server(ESPFLIGHT_WEBSOCKET_PORT);  // Declare value for use in the current scope.
AsyncWebSocket ws("/");                           // Declare value for use in the current scope.
static uint32_t activeControlClientId = 0U;       // Store the only WebSocket client currently allowed to command flight channels.
static bool activeControlClientAssigned = false;  // Track whether controller ownership has been assigned.
static uint32_t lastUdpDiscoveryMs = 0U;          // Store the most recent successful UDP discovery broadcast time.
static uint32_t lastWifiReconnectAttemptMs = 0U;  // Store the newest non-blocking Wi-Fi retry timestamp.

// ============================================================
// PID persistent storage
// ============================================================
static const uint32_t PID_STORE_MAGIC = 0x45535050UL;  // "ESPP"
static const uint16_t PID_STORE_VERSION = 3;           // Store Altitude PID together with the retained attitude PID values.
static const uint16_t PID_STORE_LEGACY_VERSION = 2;    // Recognize the previous attitude-only EEPROM layout for one-time migration.
static const int PID_EEPROM_ADDR = 0;                  // Keep the PID data at the existing EEPROM start address.
static constexpr float PID_P_MIN = 0.0f;               // Define the minimum accepted proportional PID gain.
static constexpr float PID_P_MAX = 50.0f;              // Define the maximum accepted proportional PID gain.
static constexpr float PID_I_MIN = 0.0f;               // Define the minimum accepted integral PID gain.
static constexpr float PID_I_MAX = 10.0f;              // Define the maximum accepted integral PID gain.
static constexpr float PID_D_MIN = 0.0f;               // Define the minimum accepted derivative PID gain.
static constexpr float PID_D_MAX = 500.0f;             // Define the maximum accepted derivative PID gain.

struct PidStoreDataV2 {  // Preserve the exact previous EEPROM layout so existing tuned attitude PID values can be migrated safely.
  uint32_t magic;        // Store the fixed PID block signature.
  uint16_t version;      // Store the legacy layout version.
  float kpRoll;          // Store legacy Roll proportional gain.
  float kiRoll;          // Store legacy Roll integral gain.
  float kdRoll;          // Store legacy Roll derivative gain.
  float kpPitch;         // Store legacy Pitch proportional gain.
  float kiPitch;         // Store legacy Pitch integral gain.
  float kdPitch;         // Store legacy Pitch derivative gain.
  float kpYaw;           // Store legacy Yaw proportional gain.
  float kiYaw;           // Store legacy Yaw integral gain.
  float kdYaw;           // Store legacy Yaw derivative gain.
  uint16_t checksum;     // Store the legacy checksum at the same offset used by version 2 firmware.
};

struct PidStoreData {  // Define the current EEPROM layout containing attitude and Altitude PID gains.
  uint32_t magic;      // Store the fixed PID block signature.
  uint16_t version;    // Store the current layout version.
  float kpRoll;        // Store Roll proportional gain.
  float kiRoll;        // Store Roll integral gain.
  float kdRoll;        // Store Roll derivative gain.
  float kpPitch;       // Store Pitch proportional gain.
  float kiPitch;       // Store Pitch integral gain.
  float kdPitch;       // Store Pitch derivative gain.
  float kpYaw;         // Store Yaw proportional gain.
  float kiYaw;         // Store Yaw integral gain.
  float kdYaw;         // Store Yaw derivative gain.
  float kpAlt;         // Store Altitude proportional gain.
  float kiAlt;         // Store Altitude integral gain.
  float kdAlt;         // Store Altitude derivative gain.
  uint16_t checksum;   // Protect the complete current PID block against corrupted EEPROM data.
};

static uint16_t pidChecksumBytes(const uint8_t *raw, size_t length) {  // Calculate the preserved rotating checksum over a bounded byte range.
  uint16_t sum = 0xA5A5;                                               // Start from the same checksum seed used by the previous firmware.
  for (size_t i = 0; i < length; i++) {                                // Process every byte before the checksum field.
    sum = static_cast<uint16_t>((sum << 5) | (sum >> 11));             // Preserve the previous checksum rotation.
    sum ^= raw[i];                                                     // Mix the current EEPROM byte into the checksum.
  }
  return sum;  // Return the final checksum value.
}

static uint16_t pidChecksum(const PidStoreData &data) {                           // Calculate a checksum for the current version-3 PID block.
  return pidChecksumBytes(reinterpret_cast<const uint8_t *>(&data),              // Read the current PID structure as raw bytes.
                          offsetof(PidStoreData, checksum));                      // Stop immediately before the checksum field.
}

static uint16_t pidChecksumLegacy(const PidStoreDataV2 &data) {                   // Calculate the exact checksum expected by the old version-2 layout.
  return pidChecksumBytes(reinterpret_cast<const uint8_t *>(&data),              // Read the legacy structure using the same byte order as before.
                          offsetof(PidStoreDataV2, checksum));                    // Stop at the original version-2 checksum offset.
}

static bool sanePid(float value, float minValue, float maxValue) {   // Validate one PID gain against finite numeric bounds.
  return isfinite(value) && value >= minValue && value <= maxValue;  // Accept only finite gains inside the configured safe range.
}

static bool isStoredPidValid(const PidStoreData &data) {                                 // Validate the current attitude-plus-altitude EEPROM block.
  if (data.magic != PID_STORE_MAGIC || data.version != PID_STORE_VERSION) return false;  // Reject a different signature or layout version.
  if (pidChecksum(data) != data.checksum) return false;                                  // Reject corrupted current-format data.
  if (!sanePid(data.kpRoll, PID_P_MIN, PID_P_MAX)) return false;                         // Validate Roll proportional gain.
  if (!sanePid(data.kiRoll, PID_I_MIN, PID_I_MAX)) return false;                         // Validate Roll integral gain.
  if (!sanePid(data.kdRoll, PID_D_MIN, PID_D_MAX)) return false;                         // Validate Roll derivative gain.
  if (!sanePid(data.kpPitch, PID_P_MIN, PID_P_MAX)) return false;                        // Validate Pitch proportional gain.
  if (!sanePid(data.kiPitch, PID_I_MIN, PID_I_MAX)) return false;                        // Validate Pitch integral gain.
  if (!sanePid(data.kdPitch, PID_D_MIN, PID_D_MAX)) return false;                        // Validate Pitch derivative gain.
  if (!sanePid(data.kpYaw, PID_P_MIN, PID_P_MAX)) return false;                          // Validate Yaw proportional gain.
  if (!sanePid(data.kiYaw, PID_I_MIN, PID_I_MAX)) return false;                          // Validate Yaw integral gain.
  if (!sanePid(data.kdYaw, PID_D_MIN, PID_D_MAX)) return false;                          // Validate Yaw derivative gain.
  if (!sanePid(data.kpAlt, PID_P_MIN, PID_P_MAX)) return false;                          // Validate Altitude proportional gain.
  if (!sanePid(data.kiAlt, PID_I_MIN, PID_I_MAX)) return false;                          // Validate Altitude integral gain.
  if (!sanePid(data.kdAlt, PID_D_MIN, PID_D_MAX)) return false;                          // Validate Altitude derivative gain.
  return true;                                                                           // Accept the complete current PID block.
}

static bool isStoredPidLegacyValid(const PidStoreDataV2 &data) {                               // Validate one existing version-2 attitude-only EEPROM block before migration.
  if (data.magic != PID_STORE_MAGIC || data.version != PID_STORE_LEGACY_VERSION) return false;  // Require the previous exact signature and version.
  if (pidChecksumLegacy(data) != data.checksum) return false;                                   // Reject legacy data with a checksum mismatch.
  if (!sanePid(data.kpRoll, PID_P_MIN, PID_P_MAX)) return false;                                // Validate legacy Roll proportional gain.
  if (!sanePid(data.kiRoll, PID_I_MIN, PID_I_MAX)) return false;                                // Validate legacy Roll integral gain.
  if (!sanePid(data.kdRoll, PID_D_MIN, PID_D_MAX)) return false;                                // Validate legacy Roll derivative gain.
  if (!sanePid(data.kpPitch, PID_P_MIN, PID_P_MAX)) return false;                               // Validate legacy Pitch proportional gain.
  if (!sanePid(data.kiPitch, PID_I_MIN, PID_I_MAX)) return false;                               // Validate legacy Pitch integral gain.
  if (!sanePid(data.kdPitch, PID_D_MIN, PID_D_MAX)) return false;                               // Validate legacy Pitch derivative gain.
  if (!sanePid(data.kpYaw, PID_P_MIN, PID_P_MAX)) return false;                                 // Validate legacy Yaw proportional gain.
  if (!sanePid(data.kiYaw, PID_I_MIN, PID_I_MAX)) return false;                                 // Validate legacy Yaw integral gain.
  if (!sanePid(data.kdYaw, PID_D_MIN, PID_D_MAX)) return false;                                 // Validate legacy Yaw derivative gain.
  return true;                                                                                  // Accept the legacy attitude PID block for migration.
}

static void resetPidRuntimeTerms() {  // Clear every PID runtime accumulator after a persisted tuning change.
  pid_i_mem_roll = 0;                 // Clear Roll integral memory.
  pid_last_roll_d_error = 0;          // Clear Roll derivative history.
  pid_i_mem_pitch = 0;                // Clear Pitch integral memory.
  pid_last_pitch_d_error = 0;         // Clear Pitch derivative history.
  pid_i_mem_yaw = 0;                  // Clear Yaw integral memory.
  pid_last_yaw_d_error = 0;           // Clear Yaw derivative history.
  resetAltitudePidRuntime();          // Clear Altitude output, integral memory, target, and derivative history without changing its gains.
}

static void fillPidStore(PidStoreData &data) {  // Build one complete current-format EEPROM image from the live gains.
  data.magic = PID_STORE_MAGIC;                 // Write the PID block signature.
  data.version = PID_STORE_VERSION;             // Write the current storage version.
  data.kpRoll = pid_p_gain_roll;                // Save Roll proportional gain.
  data.kiRoll = pid_i_gain_roll;                // Save Roll integral gain.
  data.kdRoll = pid_d_gain_roll;                // Save Roll derivative gain.
  data.kpPitch = pid_p_gain_pitch;              // Save Pitch proportional gain.
  data.kiPitch = pid_i_gain_pitch;              // Save Pitch integral gain.
  data.kdPitch = pid_d_gain_pitch;              // Save Pitch derivative gain.
  data.kpYaw = pid_p_gain_yaw;                  // Save Yaw proportional gain.
  data.kiYaw = pid_i_gain_yaw;                  // Save Yaw integral gain.
  data.kdYaw = pid_d_gain_yaw;                  // Save Yaw derivative gain.
  data.kpAlt = pid_p_gain_alt;                  // Save Altitude proportional gain.
  data.kiAlt = pid_i_gain_alt;                  // Save Altitude integral gain.
  data.kdAlt = pid_d_gain_alt;                  // Save Altitude derivative gain.
  data.checksum = pidChecksum(data);            // Protect all saved gains with the current checksum.
}

static bool savePidToEeprom() {       // Persist the complete current attitude-plus-altitude PID set atomically.
  PidStoreData data;                  // Reserve one current-format PID block.
  fillPidStore(data);                 // Fill the block from the live controller gains.
  EEPROM.put(PID_EEPROM_ADDR, data);  // Stage the complete PID block at the existing EEPROM address.
  return EEPROM.commit();             // Report success only when the ESP8266 EEPROM emulation commits successfully.
}

static void loadPidStore(const PidStoreData &data) {  // Restore every current PID gain from validated EEPROM data.
  pid_p_gain_roll = data.kpRoll;                      // Restore Roll proportional gain.
  pid_i_gain_roll = data.kiRoll;                      // Restore Roll integral gain.
  pid_d_gain_roll = data.kdRoll;                      // Restore Roll derivative gain.
  pid_p_gain_pitch = data.kpPitch;                    // Restore Pitch proportional gain.
  pid_i_gain_pitch = data.kiPitch;                    // Restore Pitch integral gain.
  pid_d_gain_pitch = data.kdPitch;                    // Restore Pitch derivative gain.
  pid_p_gain_yaw = data.kpYaw;                        // Restore Yaw proportional gain.
  pid_i_gain_yaw = data.kiYaw;                        // Restore Yaw integral gain.
  pid_d_gain_yaw = data.kdYaw;                        // Restore Yaw derivative gain.
  pid_p_gain_alt = data.kpAlt;                        // Restore Altitude proportional gain.
  pid_i_gain_alt = data.kiAlt;                        // Restore Altitude integral gain.
  pid_d_gain_alt = data.kdAlt;                        // Restore Altitude derivative gain.
}

static void loadLegacyPidStore(const PidStoreDataV2 &data) {  // Restore only attitude gains from version 2 while retaining the tested compiled Altitude defaults.
  pid_p_gain_roll = data.kpRoll;                              // Restore legacy Roll proportional gain.
  pid_i_gain_roll = data.kiRoll;                              // Restore legacy Roll integral gain.
  pid_d_gain_roll = data.kdRoll;                              // Restore legacy Roll derivative gain.
  pid_p_gain_pitch = data.kpPitch;                            // Restore legacy Pitch proportional gain.
  pid_i_gain_pitch = data.kiPitch;                            // Restore legacy Pitch integral gain.
  pid_d_gain_pitch = data.kdPitch;                            // Restore legacy Pitch derivative gain.
  pid_p_gain_yaw = data.kpYaw;                                // Restore legacy Yaw proportional gain.
  pid_i_gain_yaw = data.kiYaw;                                // Restore legacy Yaw integral gain.
  pid_d_gain_yaw = data.kdYaw;                                // Restore legacy Yaw derivative gain.
}

void initPidStorage() {  // Initialize EEPROM PID storage while preserving existing version-2 attitude tuning.
  EEPROM.begin(256);     // Keep the existing EEPROM emulation allocation, which is larger than either PID structure.

  PidStoreData data;                  // Read the current version-3 layout first.
  EEPROM.get(PID_EEPROM_ADDR, data);  // Copy the stored bytes into the current PID structure.
  if (isStoredPidValid(data)) {       // Use the saved attitude and Altitude gains only after full validation.
    loadPidStore(data);               // Restore all validated gains.
    resetPidRuntimeTerms();           // Start every controller with clean runtime memory.
    Serial.println("✅ PID including Altitude loaded from EEPROM");  // Report the successful current-format restore.
    return;                           // Finish initialization after a valid version-3 load.
  }

  PidStoreDataV2 legacyData;                        // Read the old attitude-only layout for backward-compatible migration.
  EEPROM.get(PID_EEPROM_ADDR, legacyData);          // Re-read the same EEPROM bytes using the exact previous structure.
  if (isStoredPidLegacyValid(legacyData)) {         // Migrate only when the old checksum and all gains are valid.
    loadLegacyPidStore(legacyData);                 // Preserve the user's existing Roll/Pitch/Yaw tuning.
    resetPidRuntimeTerms();                         // Clear runtime controller memory before persisting the migrated set.
    if (savePidToEeprom()) {                        // Save a new version-3 block with the tested compiled Altitude defaults.
      Serial.println("✅ PID EEPROM migrated v2 -> v3; Altitude defaults added");  // Confirm one-time migration.
    } else {                                        // Handle a rare EEPROM commit failure without discarding the valid live gains.
      Serial.println("⚠️ PID v2 loaded but v3 migration commit failed");           // Keep the live migrated values and report persistence failure.
    }
    return;  // Finish initialization after loading the valid legacy values.
  }

  resetPidRuntimeTerms();                                                    // Ensure defaults begin with clean runtime controller memory.
  Serial.println("ℹ️ PID EEPROM empty/invalid, using firmware defaults");  // Report that compiled defaults will be used.
  if (!savePidToEeprom()) {                                                  // Persist the complete default attitude and Altitude set when possible.
    Serial.println("⚠️ Failed to save default PID values to EEPROM");     // Report a first-time persistence failure without changing valid RAM defaults.
  }
}

static void sendPidAck(AsyncWebSocketClient *client, uint32_t requestId, bool ok, const char *message) {  // Send one small PID result only to the requesting application.
  if (client == nullptr) return;                                                                          // Ignore a missing requester without broadcasting control information.
  StaticJsonDocument<160> txDoc;                                                                          // Reserve only the memory required by the compact acknowledgement.
  txDoc["type"] = "pid_ack";                                                                              // Identify this frame as the PID command result.
  txDoc["request_id"] = requestId;                                                                        // Correlate the result with the application's single PID request.
  txDoc["ok"] = ok;                                                                                       // Report whether the values were applied and stored successfully.
  txDoc["message"] = message == nullptr ? "" : message;                                                   // Publish one short stable result code for the user interface.

  char out[192];                                      // Reserve a bounded output buffer without allocating a dynamic String.
  size_t n = serializeJson(txDoc, out, sizeof(out));  // Serialize the compact acknowledgement into the fixed buffer.
  if (n > 0U) client->text(out, n);                   // Send the result only when serialization completed successfully.
}  // Close the current declaration or execution block.

static bool hasAnyPidKey(JsonDocument &doc) {                                                          // Detect any supported attitude or Altitude PID field.
  return doc.containsKey("kp_roll") || doc.containsKey("ki_roll") || doc.containsKey("kd_roll") ||     // Detect the Roll PID fields.
         doc.containsKey("kp_pitch") || doc.containsKey("ki_pitch") || doc.containsKey("kd_pitch") ||  // Detect the Pitch PID fields.
         doc.containsKey("kp_yaw") || doc.containsKey("ki_yaw") || doc.containsKey("kd_yaw") ||       // Detect the Yaw PID fields.
         doc.containsKey("kp_alt") || doc.containsKey("ki_alt") || doc.containsKey("kd_alt");          // Detect the Altitude PID fields.
}  // Finish detecting supported PID fields.

static bool readFloatKey(JsonDocument &doc, const char *key, float &target) {  // Declare value for use in the current scope.
  if (!doc.containsKey(key)) return false;                                     // Evaluate this condition before executing its protected branch.
  target = doc[key].as<float>();                                               // Update target with the value calculated on this line.
  return true;                                                                 // Return the resulting value to the caller.
}  // Close the current declaration or execution block.

static bool isPidJsonKeyValid(JsonDocument &doc, const char *key, float minValue, float maxValue) {  // Validate one optional PID JSON field before any live gain is changed.
  if (!doc.containsKey(key)) return true;                                                            // Treat an omitted field as valid because partial PID updates are supported.
  JsonVariantConst value = doc[key];                                                                 // Read the field without mutating the source document.
  if (!value.is<float>() && !value.is<int>() && !value.is<long>()) return false;                     // Reject strings, Booleans, null values, arrays, and objects.
  return sanePid(value.as<float>(), minValue, maxValue);                                             // Accept only finite values inside the configured safe range.
}  // Finish validating one optional PID JSON field.

static bool arePidJsonValuesValid(JsonDocument &doc) {                // Validate every supplied attitude and Altitude PID field before applying any of them.
  return isPidJsonKeyValid(doc, "kp_roll", PID_P_MIN, PID_P_MAX) &&   // Validate the Roll proportional gain when present.
         isPidJsonKeyValid(doc, "ki_roll", PID_I_MIN, PID_I_MAX) &&   // Validate the Roll integral gain when present.
         isPidJsonKeyValid(doc, "kd_roll", PID_D_MIN, PID_D_MAX) &&   // Validate the Roll derivative gain when present.
         isPidJsonKeyValid(doc, "kp_pitch", PID_P_MIN, PID_P_MAX) &&  // Validate the Pitch proportional gain when present.
         isPidJsonKeyValid(doc, "ki_pitch", PID_I_MIN, PID_I_MAX) &&  // Validate the Pitch integral gain when present.
         isPidJsonKeyValid(doc, "kd_pitch", PID_D_MIN, PID_D_MAX) &&  // Validate the Pitch derivative gain when present.
         isPidJsonKeyValid(doc, "kp_yaw", PID_P_MIN, PID_P_MAX) &&    // Validate the Yaw proportional gain when present.
         isPidJsonKeyValid(doc, "ki_yaw", PID_I_MIN, PID_I_MAX) &&    // Validate the Yaw integral gain when present.
         isPidJsonKeyValid(doc, "kd_yaw", PID_D_MIN, PID_D_MAX) &&    // Validate the Yaw derivative gain when present.
         isPidJsonKeyValid(doc, "kp_alt", PID_P_MIN, PID_P_MAX) &&    // Validate the Altitude proportional gain when present.
         isPidJsonKeyValid(doc, "ki_alt", PID_I_MIN, PID_I_MAX) &&    // Validate the Altitude integral gain when present.
         isPidJsonKeyValid(doc, "kd_alt", PID_D_MIN, PID_D_MAX);      // Validate the Altitude derivative gain when present.
}  // Finish validating all supplied PID fields.

static bool applyPidFromJson(JsonDocument &doc, AsyncWebSocketClient *client) {                 // Declare value for use in the current scope.
  const char *packetType = doc["type"] | "";                                                    // Initialize packetType with its required starting value.
  const char *cmd = doc["cmd"] | "";                                                            // Initialize cmd with its required starting value.
  const long requestIdValue = doc["request_id"] | 0L;                                           // Read the optional positive application request identifier.
  const uint32_t requestId = requestIdValue > 0L ? static_cast<uint32_t>(requestIdValue) : 0U;  // Use zero only for backward-compatible legacy requests.

  const bool pidPacket = strcmp(packetType, "pid") == 0 || strcmp(cmd, "pid") == 0 || hasAnyPidKey(doc);  // Detect an explicit PID packet or any packet carrying supported PID keys.
  if (!pidPacket) return false;                                                                           // Leave non-PID packets for the remaining WebSocket handlers.

  if (activeControlClientAssigned && client->id() != activeControlClientId) {         // Reject PID changes from a different client whenever flight-control ownership is already assigned.
    sendPidAck(client, requestId, false, "pid_rejected_non_control_client");         // Tell the application that only the current control owner may change PID gains.
    return true;                                                                       // Mark the PID packet as handled without changing RAM or EEPROM values.
  }                                                                                    // Finish enforcing PID ownership without blocking setup when no controller owns the session yet.

  if (start != 0U) {                                                             // Permit PID changes only from the fully disarmed state, never while armed.
    sendPidAck(client, requestId, false, "pid_rejected_unless_fully_disarmed");  // Tell the application to return the controller to the fully disarmed state before retrying.
    return true;                                                                 // Mark the PID packet as handled without changing any controller state.
  }                                                                              // Finish enforcing the fully-disarmed-only PID update rule.

  if (!arePidJsonValuesValid(doc)) {                                            // Reject the entire PID packet before any live controller value is changed.
    sendPidAck(client, requestId, false, "pid_value_invalid_or_out_of_range");  // Report the validation failure to the application.
    return true;                                                                // Mark the PID packet as handled without applying or saving it.
  }                                                                             // Finish validating the complete PID packet.

  const float previousKpRoll = pid_p_gain_roll;    // Preserve the complete currently active PID set for rollback if EEPROM persistence fails.
  const float previousKiRoll = pid_i_gain_roll;    // Preserve the previous Roll integral gain.
  const float previousKdRoll = pid_d_gain_roll;    // Preserve the previous Roll derivative gain.
  const float previousKpPitch = pid_p_gain_pitch;  // Preserve the previous Pitch proportional gain.
  const float previousKiPitch = pid_i_gain_pitch;  // Preserve the previous Pitch integral gain.
  const float previousKdPitch = pid_d_gain_pitch;  // Preserve the previous Pitch derivative gain.
  const float previousKpYaw = pid_p_gain_yaw;      // Preserve the previous Yaw proportional gain.
  const float previousKiYaw = pid_i_gain_yaw;      // Preserve the previous Yaw integral gain.
  const float previousKdYaw = pid_d_gain_yaw;      // Preserve the previous Yaw derivative gain.
  const float previousKpAlt = pid_p_gain_alt;      // Preserve the previous Altitude proportional gain.
  const float previousKiAlt = pid_i_gain_alt;      // Preserve the previous Altitude integral gain.
  const float previousKdAlt = pid_d_gain_alt;      // Preserve the previous Altitude derivative gain.

  bool changed = false;  // Initialize changed with its required starting value.

  bool rollPChanged = readFloatKey(doc, "kp_roll", pid_p_gain_roll);  // Initialize rollPChanged with its required starting value.
  bool rollIChanged = readFloatKey(doc, "ki_roll", pid_i_gain_roll);  // Initialize rollIChanged with its required starting value.
  bool rollDChanged = readFloatKey(doc, "kd_roll", pid_d_gain_roll);  // Initialize rollDChanged with its required starting value.
  changed |= rollPChanged || rollIChanged || rollDChanged;            // Execute this statement as part of the current operation.

  // The current app uses one Pitch/Roll group, so when pitch keys are absent,
  // mirror Roll values to Pitch. If future apps send explicit pitch keys, use them.
  if (!readFloatKey(doc, "kp_pitch", pid_p_gain_pitch) && rollPChanged) pid_p_gain_pitch = pid_p_gain_roll;  // Evaluate this condition before executing its protected branch.
  else if (doc.containsKey("kp_pitch")) changed = true;                                                      // Test the next condition when the previous branch did not run.

  if (!readFloatKey(doc, "ki_pitch", pid_i_gain_pitch) && rollIChanged) pid_i_gain_pitch = pid_i_gain_roll;  // Evaluate this condition before executing its protected branch.
  else if (doc.containsKey("ki_pitch")) changed = true;                                                      // Test the next condition when the previous branch did not run.

  if (!readFloatKey(doc, "kd_pitch", pid_d_gain_pitch) && rollDChanged) pid_d_gain_pitch = pid_d_gain_roll;  // Evaluate this condition before executing its protected branch.
  else if (doc.containsKey("kd_pitch")) changed = true;                                                      // Test the next condition when the previous branch did not run.

  changed |= readFloatKey(doc, "kp_yaw", pid_p_gain_yaw);  // Invoke or continue readFloatKey for the current operation.
  changed |= readFloatKey(doc, "ki_yaw", pid_i_gain_yaw);  // Invoke or continue readFloatKey for the current operation.
  changed |= readFloatKey(doc, "kd_yaw", pid_d_gain_yaw);  // Apply Yaw derivative gain when present.
  changed |= readFloatKey(doc, "kp_alt", pid_p_gain_alt);  // Apply Altitude proportional gain when present.
  changed |= readFloatKey(doc, "ki_alt", pid_i_gain_alt);  // Apply Altitude integral gain when present.
  changed |= readFloatKey(doc, "kd_alt", pid_d_gain_alt);  // Apply Altitude derivative gain when present.

  if (!changed) {                                                       // Evaluate this condition before executing its protected branch.
    sendPidAck(client, requestId, false, "pid_packet_without_values");  // Invoke or continue sendPidAck for the current operation.
    return true;                                                        // Return the resulting value to the caller.
  }                                                                     // Close the current declaration or execution block.

  const bool saved = savePidToEeprom();  // Try to persist the complete new PID set before declaring it active.

  if (!saved) {                                                                                                             // Roll back every live gain when persistent storage does not confirm the update.
    pid_p_gain_roll = previousKpRoll;                                                                                       // Restore the previous Roll proportional gain.
    pid_i_gain_roll = previousKiRoll;                                                                                       // Restore the previous Roll integral gain.
    pid_d_gain_roll = previousKdRoll;                                                                                       // Restore the previous Roll derivative gain.
    pid_p_gain_pitch = previousKpPitch;                                                                                     // Restore the previous Pitch proportional gain.
    pid_i_gain_pitch = previousKiPitch;                                                                                     // Restore the previous Pitch integral gain.
    pid_d_gain_pitch = previousKdPitch;                                                                                     // Restore the previous Pitch derivative gain.
    pid_p_gain_yaw = previousKpYaw;                                                                                         // Restore the previous Yaw proportional gain.
    pid_i_gain_yaw = previousKiYaw;                                                                                         // Restore the previous Yaw integral gain.
    pid_d_gain_yaw = previousKdYaw;                                                                                         // Restore the previous Yaw derivative gain.
    pid_p_gain_alt = previousKpAlt;                                                                                         // Restore the previous Altitude proportional gain.
    pid_i_gain_alt = previousKiAlt;                                                                                         // Restore the previous Altitude integral gain.
    pid_d_gain_alt = previousKdAlt;                                                                                         // Restore the previous Altitude derivative gain.
    resetPidRuntimeTerms();                                                                                                 // Clear controller memory after restoring the previously active gains.
    PidStoreData rollbackData;                                                                                              // Build a complete EEPROM image from the restored PID set.
    fillPidStore(rollbackData);                                                                                             // Fill the rollback image using the restored gains and a fresh checksum.
    EEPROM.put(PID_EEPROM_ADDR, rollbackData);                                                                              // Replace the RAM-backed EEPROM staging buffer with the previous valid PID set.
    const bool rollbackStored = EEPROM.commit();                                                                            // Make one best-effort attempt to restore persistent storage after the failed commit.
    Serial.printf("❌ PID EEPROM commit failed; runtime rolled back, EEPROM rollback=%s\n", rollbackStored ? "yes" : "no");  // Report both the original persistence failure and rollback result.
    sendPidAck(client, requestId, false, "pid_eeprom_commit_failed_rolled_back");                                           // Tell the application that the requested gains were not kept active.
    return true;                                                                                                            // Finish handling the failed PID update without leaving mixed controller state.
  }                                                                                                                         // Finish the rollback path for persistent-storage failure.

  resetPidRuntimeTerms();  // Clear PID memory only after the new gains are both validated and successfully persisted.

  Serial.printf(                                                                                                                      // Report the applied attitude and Altitude PID values.
    "✅ PID updated: ROLL(%.3f, %.3f, %.3f) PITCH(%.3f, %.3f, %.3f) YAW(%.3f, %.3f, %.3f) ALT(%.3f, %.3f, %.3f) saved=yes\n",  // Print one complete persisted PID summary.
    pid_p_gain_roll, pid_i_gain_roll, pid_d_gain_roll,                                                                                // Report the Roll PID gains.
    pid_p_gain_pitch, pid_i_gain_pitch, pid_d_gain_pitch,                                                                             // Report the Pitch PID gains.
    pid_p_gain_yaw, pid_i_gain_yaw, pid_d_gain_yaw,                                                                                   // Report the Yaw PID gains.
    pid_p_gain_alt, pid_i_gain_alt, pid_d_gain_alt);                                                                                   // Report the Altitude PID gains.

  sendPidAck(client, requestId, true, "pid_saved_and_applied");  // Confirm success only after the values are active and EEPROM commit succeeded.
  return true;                                                   // Return the resulting value to the caller.
}  // Close the current declaration or execution block.

static bool hasAnyReceiverKey(JsonDocument &doc) {                  // Detect packets that attempt to update any flight-control channel.
  return doc.containsKey("throttle") || doc.containsKey("roll") ||  // Detect throttle or roll command fields.
         doc.containsKey("pitch") || doc.containsKey("yaw");        // Detect pitch or yaw command fields.
}  // Finish checking for receiver command fields.

static bool readReceiverChannel(JsonDocument &doc, const char *key, int &value) {  // Read one strictly typed and bounded receiver channel.
  if (!doc.containsKey(key) || !doc[key].is<int>()) {                              // Require the named field and an integer JSON value.
    return false;                                                                  // Reject missing, floating-point, string, Boolean, object, array, or null values.
  }                                                                                // Finish validating the field type.
  const int candidate = doc[key].as<int>();                                        // Convert the already validated integer field.
  if (candidate < 1000 || candidate > 2000) {                                      // Enforce the supported receiver command range.
    return false;                                                                  // Reject commands that could create unsafe mixer or arming values.
  }                                                                                // Finish validating the receiver range.
  value = candidate;                                                               // Publish the validated receiver value to the caller.
  return true;                                                                     // Report that this channel is complete and safe to use.
}  // Finish reading one receiver channel.

static bool readCompleteReceiverPacket(JsonDocument &doc, int &newThrottle, int &newRoll, int &newPitch, int &newYaw) {  // Validate all four channels before changing any live command.
  return readReceiverChannel(doc, "throttle", newThrottle) &&                                                            // Require a valid throttle field.
         readReceiverChannel(doc, "roll", newRoll) &&                                                                    // Require a valid roll field.
         readReceiverChannel(doc, "pitch", newPitch) &&                                                                  // Require a valid pitch field.
         readReceiverChannel(doc, "yaw", newYaw);                                                                        // Require a valid yaw field.
}  // Finish validating one atomic receiver packet.

static bool readPositiveRequestId(JsonDocument &doc, uint32_t &requestId) {  // Read the mandatory positive request identifier used to correlate assisted-flight acknowledgements.
  if (!doc.containsKey("request_id")) return false;                         // Reject commands that cannot be correlated with the requesting application action.
  JsonVariantConst raw = doc["request_id"];                               // Inspect the request id without mutating the received JSON document.
  if (!raw.is<int>() && !raw.is<long>() && !raw.is<unsigned int>() && !raw.is<unsigned long>()) return false;  // Reject strings, floats, Booleans, and container types.
  const long value = raw.as<long>();                                       // Convert only after strict numeric type validation.
  if (value <= 0L) return false;                                           // Reserve zero and negative values as invalid request identifiers.
  requestId = static_cast<uint32_t>(value);                                // Publish the validated positive request identifier.
  return true;                                                             // Report successful correlation validation.
}

static void sendFlightCommandAck(AsyncWebSocketClient *client, uint32_t requestId, const char *command, bool ok, const char *message) {  // Send one bounded high-level flight-command result only to the requester.
  if (client == nullptr) return;                                                                                                          // Ignore a missing requester without broadcasting flight-control state.
  StaticJsonDocument<192> txDoc;                                                                                                          // Reserve fixed storage for one compact acknowledgement.
  txDoc["type"] = "flight_command_ack";                                                                                                  // Match the exact frame type expected by ESPFlight Application.
  txDoc["cmd"] = command == nullptr ? "" : command;                                                                                      // Echo the normalized command so the app can correlate both id and command name.
  txDoc["request_id"] = requestId;                                                                                                       // Echo the positive request identifier from the application.
  txDoc["ok"] = ok;                                                                                                                      // Report whether firmware actually accepted the requested state transition.
  txDoc["message"] = message == nullptr ? "" : message;                                                                                  // Return one stable short result code for UI feedback.
  char out[224];                                                                                                                          // Serialize into bounded stack storage without dynamic String allocation.
  const size_t n = serializeJson(txDoc, out, sizeof(out));                                                                                // Build the compact acknowledgement frame.
  if (n > 0U) client->text(out, n);                                                                                                       // Send only after successful serialization.
}

static bool handleFlightCommand(JsonDocument &doc, AsyncWebSocketClient *client) {  // Validate and dispatch only the high-level ARM/DISARM/Takeoff/Landing requests prepared by the application.
  const char *packetType = doc["type"] | "";                                      // Read the packet type without treating ordinary joystick/PID frames as assisted commands.
  if (strcmp(packetType, "flight_command") != 0) return false;                     // Leave every non-flight-command packet for the existing handlers.

  const char *command = doc["cmd"] | "";                                          // Read the requested high-level flight action.
  uint32_t requestId = 0U;                                                         // Reserve the acknowledgement correlation identifier.
  if (!readPositiveRequestId(doc, requestId)) {                                    // Require strict positive request correlation.
    sendFlightCommandAck(client, 0U, command, false, "flight_command_invalid_request_id");  // Reject malformed requests without changing any flight state.
    return true;                                                                   // Mark the malformed flight-command packet handled.
  }

  const bool isArm = strcmp(command, "arm") == 0;                                  // Detect the explicit ARM-ready command.
  const bool isDisarm = strcmp(command, "disarm") == 0;                            // Detect the explicit zero-PWM DISARM command.
  const bool isTakeoff = strcmp(command, "takeoff") == 0;                          // Detect the supported Takeoff command.
  const bool isLanding = strcmp(command, "landing") == 0;                          // Detect the supported Landing command.
  if (!isArm && !isDisarm && !isTakeoff && !isLanding) {                          // Reject unknown high-level flight commands explicitly.
    sendFlightCommandAck(client, requestId, command, false, "flight_command_unsupported");  // Report unsupported command name.
    return true;                                                                   // Prevent fall-through into receiver parsing.
  }
  if (!activeControlClientAssigned) {                                              // Require the same validated controller ownership used by joystick flight commands.
    sendFlightCommandAck(client, requestId, command, false, "flight_command_rejected_no_control_owner");  // Reject commands before a controller owns the receiver stream.
    return true;                                                                   // Preserve exclusive flight-control ownership.
  }
  if (client == nullptr || client->id() != activeControlClientId) {                // Reject spectators or secondary WebSocket clients.
    sendFlightCommandAck(client, requestId, command, false, "flight_command_rejected_non_control_client");  // Preserve exclusive control ownership.
    return true;                                                                   // Finish handling the unauthorized command.
  }

  const uint32_t nowMs = millis();                                                 // Capture one timestamp for all readiness/state validation in this command.
  const AltitudeAssistMode mode = altitudeAssistMode();                            // Read the current assisted phase once for deterministic command validation.

  if (isArm) {                                                                     // ARM is an explicit state command and never starts motor PWM by itself.
    if (start == 2U) {                                                             // Treat retransmission after a lost ACK as already complete.
      sendFlightCommandAck(client, requestId, command, true, "arm_already_active");  // Preserve idempotent ARM behavior.
      return true;                                                                 // Do not reset active flight state on duplicate requests.
    }
    if (failsafe_active || failsafeLandingActive()) {                              // Never arm while a safety descent or latched failsafe owns the aircraft.
      sendFlightCommandAck(client, requestId, command, false, "arm_rejected_failsafe_active");  // Keep safety authority exclusive.
      return true;
    }
    if (channel_3 > 1050) {                                                        // Require real received throttle at minimum before ARM-ready state is entered.
      sendFlightCommandAck(client, requestId, command, false, "arm_rejected_throttle_not_minimum");  // Explain the exact pre-arm failure.
      return true;
    }
    if (!failsafeCanArm(nowMs)) {                                                  // Preserve fresh control link, battery, and failsafe-latch checks.
      sendFlightCommandAck(client, requestId, command, false, "arm_rejected_prearm_checks");  // Report a safety pre-arm rejection.
      return true;
    }
    if (!flightControlImuReady(nowMs)) {                                           // Require fresh MPU6050 data and a safe finite attitude.
      sendFlightCommandAck(client, requestId, command, false, "arm_rejected_imu_not_ready");  // Keep the aircraft disarmed until sensor state is trustworthy.
      return true;
    }
    const bool accepted = flightControlRequestArm(nowMs);                          // Enter ARMED-ready state with all motor PWM outputs still zero.
    sendFlightCommandAck(client, requestId, command, accepted, accepted ? "arm_accepted" : "arm_rejected_state_changed");  // ACK only the actual firmware transition.
    return true;
  }

  if (isDisarm) {                                                                  // DISARM is explicit and must always end with zero motor PWM.
    if (start != 2U) {                                                             // Repeated DISARM requests are already satisfied.
      sendFlightCommandAck(client, requestId, command, true, "disarm_already_inactive");  // Preserve idempotent command behavior.
      return true;
    }
    if (channel_3 > 1050) {                                                        // Normal manual DISARM requires low throttle so an accidental UI action cannot cut an airborne aircraft.
      sendFlightCommandAck(client, requestId, command, false, "disarm_rejected_throttle_not_minimum");  // Ask the pilot to lower throttle first.
      return true;
    }
    flightControlRequestDisarm();                                                  // Cancel assist/PID and force all four PWM outputs to zero immediately.
    sendFlightCommandAck(client, requestId, command, true, "disarm_accepted");    // Confirm only after firmware has executed the hard stop.
    return true;
  }

  if (isTakeoff) {                                                                 // Keep Takeoff strict because it intentionally adds upward motor authority.
    if (start != 2U) {                                                             // Require real telemetry-confirmed ARM before automatic throttle motion can begin.
      sendFlightCommandAck(client, requestId, command, false, "flight_command_rejected_not_armed");  // Keep the aircraft fully disarmed until the explicit ARM-ready state is confirmed.
      return true;                                                                 // Do not let Takeoff alter any disarmed state.
    }
    if (failsafe_active || failsafeLandingActive()) {                              // Never let optional altitude assistance override an existing safety landing.
      sendFlightCommandAck(client, requestId, command, false, "flight_command_rejected_failsafe_active");  // Keep failsafe as sole throttle authority.
      return true;                                                                 // Finish handling the blocked upward request.
    }
    if (mode == ALTITUDE_ASSIST_TAKEOFF || mode == ALTITUDE_ASSIST_HOLD) {         // Treat every retransmission as idempotent before rechecking transient sensor or throttle values that may already have changed after acceptance.
      sendFlightCommandAck(client, requestId, command, true, "takeoff_already_active");  // Do not reset altitude PID or restart the app ramp.
      return true;                                                                 // Make a lost/delayed ACK incapable of becoming a later command rejection.
    }
    if (!vl53_available) {                                                         // Assisted Takeoff requires the optional range sensor.
      sendFlightCommandAck(client, requestId, command, false, "takeoff_rejected_sensor_unavailable");  // Explain missing hardware.
      return true;                                                                 // Never start an automatic upward ramp without height sensing.
    }
    if (!altitudeSensorReady(nowMs)) {                                             // Require several independent valid readings before an automatic upward throttle ramp may begin.
      sendFlightCommandAck(client, requestId, command, false, "takeoff_rejected_sensor_not_ready");  // Explain temporary range-data readiness failure.
      return true;                                                                 // Leave throttle untouched until the app retries internally.
    }
    if (channel_3 > 1050) {                                                        // Recheck the user's real throttle position in firmware, not only the UI.
      sendFlightCommandAck(client, requestId, command, false, "takeoff_rejected_throttle_not_minimum");  // Require the same low-throttle boundary used by ARM.
      return true;                                                                 // Do not manufacture an upward transition from an unsafe stick position.
    }
    if (mode != ALTITUDE_ASSIST_OFF) {                                             // Do not restart Takeoff during Landing.
      sendFlightCommandAck(client, requestId, command, false, "takeoff_rejected_assist_state");  // Preserve the safer downward maneuver.
      return true;                                                                 // Keep the active landing state authoritative.
    }
    const bool accepted = altitudeRequestTakeoff(nowMs);                           // Enable only the old altitude-assist gate; firmware does not manufacture a new throttle ramp.
    sendFlightCommandAck(client, requestId, command, accepted, accepted ? "takeoff_accepted" : "takeoff_rejected_state_changed");  // App starts moving its visible throttle only after this ACK.
    return true;                                                                   // Finish the strict Takeoff path.
  }

  // Landing is deliberately fail-safe and idempotent: once an authorized pilot asks to descend,
  // transient sensor/state races must not force repeated taps. Healthy assisted flight uses the
  // VL53L0X landing path; every unavailable/stale/non-assisted case falls back to the existing
  // deterministic failsafe descent from the current effective mixer throttle.
  if (start != 2U) {                                                               // If the aircraft became disarmed just before this packet arrived, the requested end state is already satisfied.
    sendFlightCommandAck(client, requestId, command, true, "landing_already_disarmed");  // Acknowledge completion without asking the app to move throttle again.
    return true;                                                                   // Avoid a stale landing command changing the already-disarmed state.
  }
  if (failsafe_active || failsafeLandingActive()) {                                // If any safety landing already owns throttle, the user's requested action is already underway.
    sendFlightCommandAck(client, requestId, command, true, "landing_already_in_progress");  // Make repeated taps and retransmissions harmless.
    return true;                                                                   // Never compete with the existing failsafe controller.
  }
  if (mode == ALTITUDE_ASSIST_LANDING) {                                           // Treat duplicate Landing requests as idempotent.
    sendFlightCommandAck(client, requestId, command, true, "landing_already_active");  // Keep the existing descending app throttle ramp.
    return true;                                                                   // Do not reset touchdown counters or landing timing.
  }

  if (vl53_available && altitudeSensorHealthy(nowMs) &&                            // Prefer the normal sensor-guided path whenever fresh range data exists.
      (mode == ALTITUDE_ASSIST_HOLD || mode == ALTITUDE_ASSIST_TAKEOFF)) {         // The normal assisted landing belongs to an active Takeoff/Hold session.
    const bool accepted = altitudeRequestLanding(nowMs);                           // Tell firmware to interpret the app's upcoming 1500 -> 1050 ramp as a descending altitude target.
    if (accepted) {                                                                // Confirm the ordinary assisted transition immediately.
      sendFlightCommandAck(client, requestId, command, true, "landing_accepted"); // Let the application begin its smooth six-second throttle ramp.
      return true;                                                                 // Finish after the preferred landing path starts.
    }
  }

  altitudeResetAssist();                                                           // Remove stale altitude PID/assist state before the fallback controller takes ownership.
  failsafeStartCommandedLanding(nowMs, static_cast<int16_t>(constrain(throttle, 1050, 1900)));  // Start the existing deterministic descent from the actual current mixer throttle without latching a fault reason.
  sendFlightCommandAck(client, requestId, command, true, "landing_accepted_fallback");  // Report success because a safe downward action has started even without fresh assisted state.
  return true;                                                                     // One authorized Landing tap always resolves to a downward or already-complete state.
}

// Event callback
void onWsEvent(AsyncWebSocket *socket, AsyncWebSocketClient *client,       // Begin the WebSocket event callback.
               AwsEventType type, void *arg, uint8_t *data, size_t len) {  // Open the block associated with this declaration or condition.
  (void)socket;                                                            // Mark the callback's socket pointer as intentionally unused.

  if (type == WS_EVT_CONNECT) {                              // Evaluate this condition before executing its protected branch.
    Serial.printf("📶 Client %u connected\n", client->id());  // Invoke or continue id for the current operation.

    StaticJsonDocument<128> txDoc;   // Execute this statement as part of the current operation.
    txDoc["type"] = "mac";           // Write the "type" field into the txDoc JSON object.
    txDoc["mac"] = getMacAddress();  // Write the "mac" field into the txDoc JSON object.

    char json[128];                                                      // Reserve a fixed connection-response buffer to avoid heap fragmentation.
    const size_t jsonLength = serializeJson(txDoc, json, sizeof(json));  // Serialize the MAC response into bounded storage.
    if (jsonLength > 0U) client->text(json, jsonLength);                 // Send the simple connection response when serialization succeeds.
  }                                                                      // Close the current declaration or execution block.

  else if (type == WS_EVT_DISCONNECT) {                                          // Test the next condition when the previous branch did not run.
    if (activeControlClientAssigned && client->id() == activeControlClientId) {  // React only when the flight-control owner disconnects.
      activeControlClientAssigned = false;                                       // Release ownership so a new controller can establish a validated stream.
      activeControlClientId = 0U;                                                // Clear the stale controller identifier.
      flightControlEndSession();                                                 // Preserve cumulative time because only a board power cycle resets it.
      failsafeNotifyControlDisconnect();                                         // Request immediate link-loss handling in the next flight-control update.
    }                                                                            // Finish handling an active-controller disconnect.
    Serial.printf("📴 Client %u disconnected\n", client->id());                   // Invoke or continue id for the current operation.
  }                                                                              // Close the current declaration or execution block.

  else if (type == WS_EVT_DATA) {                                                                   // Test the next condition when the previous branch did not run.
    AwsFrameInfo *info = (AwsFrameInfo *)arg;                                                       // Initialize info with its required starting value.
    if (!(info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT)) return;  // Evaluate this condition before executing its protected branch.

    if (len == 0 || len > 384) {                                                         // Reject packets larger than the complete supported control or PID format.
      Serial.printf("⚠️ WS packet ignored, size=%u\n", static_cast<unsigned>(len));  // Invoke or continue printf for the current operation.
      return;                                                                            // Return immediately to the caller.
    }                                                                                    // Close the current declaration or execution block.

    char msg[385];           // Reserve a bounded input buffer sized for the supported JSON commands.
    memcpy(msg, data, len);  // Invoke or continue memcpy for the current operation.
    msg[len] = '\0';         // Update msg[len] with the value calculated on this line.

    StaticJsonDocument<384> rxDoc;                             // Reserve enough fixed JSON memory for one complete PID or control packet.
    DeserializationError error = deserializeJson(rxDoc, msg);  // Initialize error with its required starting value.

    if (error) {                                  // Evaluate this condition before executing its protected branch.
      Serial.print("⚠️ JSON parse error: ");  // Invoke or continue print for the current operation.
      Serial.println(error.c_str());              // Invoke or continue c_str for the current operation.
      return;                                     // Return immediately to the caller.
    }                                             // Close the current declaration or execution block.

    // --- Handle PID and high-level flight-state and assisted-flight requests before receiver packets so each protocol remains independent. ---
    if (applyPidFromJson(rxDoc, client)) return;  // Preserve the existing fully-disarmed PID update and acknowledgement path.
    if (handleFlightCommand(rxDoc, client)) return;  // Accept validated ARM/DISARM/Takeoff/Landing commands from the active controller and ACK them by request_id.

    // --- Atomic joystick / receiver values and control heartbeat ---
    if (hasAnyReceiverKey(rxDoc)) {                                                                  // Process a receiver packet only when it explicitly contains control fields.
      int newThrottle = 0;                                                                           // Hold the candidate throttle away from live state until every field passes validation.
      int newRoll = 0;                                                                               // Hold the candidate roll away from live state until every field passes validation.
      int newPitch = 0;                                                                              // Hold the candidate pitch away from live state until every field passes validation.
      int newYaw = 0;                                                                                // Hold the candidate yaw away from live state until every field passes validation.
      if (!readCompleteReceiverPacket(rxDoc, newThrottle, newRoll, newPitch, newYaw)) {              // Reject partial, mistyped, or out-of-range packets atomically.
        Serial.printf("⚠️ Invalid receiver packet ignored from client %u\n", client->id());      // Report the rejected controller packet.
        return;                                                                                      // Prevent invalid data from refreshing the failsafe heartbeat.
      }                                                                                              // Finish validating the complete receiver packet.
      if (activeControlClientAssigned && client->id() != activeControlClientId) {                    // Reject commands from every non-owner client.
        Serial.printf("⚠️ Receiver packet ignored from non-control client %u\n", client->id());  // Report the ownership rejection.
        return;                                                                                      // Preserve the active controller and its current command stream.
      }                                                                                              // Finish enforcing single-controller ownership.
      if (!activeControlClientAssigned) {                                                            // Assign ownership only after receiving a complete valid receiver packet.
        activeControlClientId = client->id();                                                        // Store the validated controller's WebSocket identifier.
        activeControlClientAssigned = true;                                                          // Mark the validated client as the flight-control owner.
      }                                                                                              // Finish assigning controller ownership.
      rc_throttle = newThrottle;                                                                     // Commit the validated throttle command.
      rc_roll = newRoll;                                                                             // Commit the validated roll command.
      rc_pitch = newPitch;                                                                           // Commit the validated pitch command.
      rc_yaw = newYaw;                                                                               // Commit the validated yaw command.
      inputReceiver();                                                                               // Transform and publish all four channels as one coherent receiver frame.
      failsafeNotifyValidControlPacket(millis(), static_cast<int16_t>(newThrottle));                 // Refresh link health and snapshot throttle from this exact validated packet.
    }                                                                                                // Finish processing the atomic receiver packet.

  }    // Close the current declaration or execution block.
}  // Close the current declaration or execution block.

// Start server helper
void startWebServer() {  // Begin the startWebServer function implementation.
  // Preserve the original design in which setup registers the WebSocket event handler.
  server.addHandler(&ws);  // Invoke or continue addHandler for the current operation.
  server.begin();          // Invoke or continue begin for the current operation.
}  // Close the current declaration or execution block.
// -------------------------------
// Get WiFi RSSI in dBm
// -------------------------------
int getRssi() {  // Declare value for use in the current scope.
  // Check if WiFi is connected
  if (WiFi.status() == WL_CONNECTED) {  // Evaluate this condition before executing its protected branch.
    return WiFi.RSSI();                 // Return signal strength in dBm (e.g. -45)
  }                                     // Close the current declaration or execution block.
  return 0;                             // Return 0 if not connected
}  // Close the current declaration or execution block.

void sendUdpInfo() {                          // Begin the sendUdpInfo function implementation.
  if (WiFi.status() != WL_CONNECTED) return;  // Evaluate this condition before executing its protected branch.

  IPAddress ip = WiFi.localIP();   // Initialize ip with its required starting value.
  String mac = WiFi.macAddress();  // Initialize mac with its required starting value.

  String msg = "{";                            // Initialize msg with its required starting value.
  msg += "\"type\":\"esp\",";                  // Accumulate into msg with the value calculated on this line.
  msg += "\"ip\":\"" + ip.toString() + "\",";  // Accumulate into msg with the value calculated on this line.
  msg += "\"mac\":\"" + mac + "\"";            // Accumulate into msg with the value calculated on this line.
  msg += "}";                                  // Accumulate into msg with the value calculated on this line.

  // Calculate a subnet broadcast address that also works with phone hotspots.
  IPAddress subnet = WiFi.subnetMask();  // Initialize subnet with its required starting value.
  IPAddress broadcast = IPAddress(       // Initialize broadcast with its required starting value.
    ip[0] | ~subnet[0],                  // Continue the current declaration, initializer, or argument list.
    ip[1] | ~subnet[1],                  // Continue the current declaration, initializer, or argument list.
    ip[2] | ~subnet[2],                  // Continue the current declaration, initializer, or argument list.
    ip[3] | ~subnet[3]);                 // Execute this statement as part of the current operation.

  udp.beginPacket(broadcast, udpPort);  // Invoke or continue beginPacket for the current operation.
  udp.print(msg);                       // Invoke or continue print for the current operation.
  udp.endPacket();                      // Invoke or continue endPacket for the current operation.
  lastUdpDiscoveryMs = millis();        // Start the periodic discovery interval from this successful broadcast.

  Serial.println("UDP Sent: " + msg);  // Invoke or continue println for the current operation.
}  // Close the current declaration or execution block.

void handleUdpDiscovery(unsigned long now) {                               // Periodically advertise the board for startup and reconnection.
  const uint32_t discoveryIntervalMs = 1500U;                              // Repeat discovery every one and a half seconds when needed.
  if (WiFi.status() != WL_CONNECTED) return;                               // Broadcast only while the station has a valid network address.
  const bool linkHealthy = failsafeLinkHealthy(now);                       // Evaluate whether the assigned controller still sends fresh packets.
  if (activeControlClientAssigned && !linkHealthy) {                       // Release stale ownership even if the old TCP socket has not closed yet.
    activeControlClientAssigned = false;                                   // Allow the same or a new app instance to claim control after safe recovery.
    activeControlClientId = 0U;                                            // Remove the stale WebSocket client identifier.
    flightControlEndSession();                                             // Preserve cumulative time across stale-link recovery and reconnection.
  }                                                                        // Finish stale control-owner cleanup.
  if (start == 2U && activeControlClientAssigned && linkHealthy) return;   // Avoid broadcasts only during a healthy owned flight-control stream.
  if ((uint32_t)(now - lastUdpDiscoveryMs) < discoveryIntervalMs) return;  // Preserve an overflow-safe fixed discovery interval.
  sendUdpInfo();                                                           // Advertise the current IP and MAC for app startup or reconnection.
}  // Finish periodic ESPFlight discovery handling.

void handleWifiReconnect(unsigned long now) {                            // Maintain Wi-Fi without blocking or restarting the flight controller.
  if (static_cast<uint32_t>(now - lastWiFiCheck) < wifiCheckInterval) {  // Run status checks only at the configured low-frequency interval.
    return;                                                              // Leave immediately until the next scheduled Wi-Fi check.
  }                                                                      // Finish enforcing the Wi-Fi check interval.
  lastWiFiCheck = now;                                                   // Record the newest Wi-Fi status check time.

  const wl_status_t status = WiFi.status();    // Read the current station connection state once.
  if (status == WL_CONNECTED) {                // Handle an active or newly recovered Wi-Fi connection.
    if (!wifiWasConnected) {                   // Detect the first check after a successful reconnect.
      wifiWasConnected = true;                 // Publish the recovered connection state.
      wifiDisconnectedSince = 0U;              // Clear the disconnected duration marker.
      lastWifiReconnectAttemptMs = 0U;         // Clear retry timing for a future independent outage.
      udp.stop();                              // Close the stale UDP socket bound before reconnection.
      udp.begin(udpPort);                      // Rebind UDP discovery to the recovered network interface.
      lastUdpDiscoveryMs = 0U;                 // Allow an immediate fresh discovery broadcast.
      Serial.print("WiFi reconnected. IP: ");  // Report successful background recovery.
      Serial.println(WiFi.localIP());          // Print the recovered station address.
      sendUdpInfo();                           // Advertise the recovered address immediately to the application.
    }                                          // Finish handling the newly recovered connection.
    return;                                    // No retry action is required while connected.
  }                                            // Finish handling the connected state.

  if (wifiWasConnected) {                                          // Detect the first status check after losing Wi-Fi.
    wifiWasConnected = false;                                      // Publish the disconnected state.
    wifiDisconnectedSince = now;                                   // Record when this independent outage began.
    lastWifiReconnectAttemptMs = 0U;                               // Allow a prompt first retry once flight safety permits.
    Serial.println("WiFi disconnected; waiting for safe retry.");  // Report the outage without restarting the controller.
  } else if (wifiDisconnectedSince == 0U) {                        // Initialize timing after an initial startup timeout.
    wifiDisconnectedSince = now;                                   // Record the start of the disconnected period.
  }                                                                // Finish initializing outage timing.

  if (start != 0U || failsafeLandingActive()) {  // Avoid starting a new Wi-Fi association during armed flight or gradual landing.
    return;                                      // Preserve control-loop timing until the aircraft is safely stopped.
  }                                              // Finish enforcing safe retry timing.

  if (lastWifiReconnectAttemptMs != 0U && static_cast<uint32_t>(now - lastWifiReconnectAttemptMs) < ESPFLIGHT_WIFI_RETRY_INTERVAL_MS) {  // Enforce the configured retry interval.
    return;                                                                                                                              // Wait until the next allowed retry window.
  }                                                                                                                                      // Finish enforcing retry spacing.

  lastWifiReconnectAttemptMs = now;               // Record this non-blocking retry attempt.
  Serial.println("Retrying WiFi connection...");  // Report the background retry.
  WiFi.disconnect(false);                         // Clear the stale station association without disabling Wi-Fi hardware.
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);           // Start a new asynchronous connection attempt.
}  // Finish non-blocking Wi-Fi recovery.

String getMacAddress() {  // Declare value for use in the current scope.
  byte mac[6];            // Declare mac for use in the current scope.
  WiFi.macAddress(mac);   // Invoke or continue macAddress for the current operation.
  char macStr[18];        // Declare macStr for use in the current scope.

  sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X",  // Invoke or continue sprintf for the current operation.
          mac[0], mac[1], mac[2],                   // Continue the current declaration, initializer, or argument list.
          mac[3], mac[4], mac[5]);                  // Execute this statement as part of the current operation.

  return String(macStr);  // Return the resulting value to the caller.
}  // Close the current declaration or execution block.

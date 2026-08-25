#include <Wire.h>            // Import declarations supplied by <Wire.h>.
#include "imu.h"             // Import declarations supplied by "imu.h".
#include "flight_control.h"  // Provide the preserved shared flight state used by the MPU6050 module.

// -------------------------------
// Function declarations
// -------------------------------

// -------------------------------
// Raw accelerometer values
// -------------------------------
int16_t raw_acc_x = 0;  // Initialize raw_acc_x with its required starting value.
int16_t raw_acc_y = 0;  // Initialize raw_acc_y with its required starting value.
int16_t raw_acc_z = 0;  // Initialize raw_acc_z with its required starting value.

// -------------------------------
// Processed accelerometer values
// -------------------------------
int16_t acc_x = 0;  // Initialize acc_x with its required starting value.
int16_t acc_y = 0;  // Initialize acc_y with its required starting value.
int16_t acc_z = 0;  // Initialize acc_z with its required starting value.

// -------------------------------
// Raw gyro values
// -------------------------------
int16_t gyro_roll_old = 0;   // Initialize gyro_roll_old with its required starting value.
int16_t gyro_pitch_old = 0;  // Initialize gyro_pitch_old with its required starting value.
int16_t gyro_yaw_old = 0;    // Initialize gyro_yaw_old with its required starting value.

int16_t gyro_roll = 0;   // Initialize gyro_roll with its required starting value.
int16_t gyro_pitch = 0;  // Initialize gyro_pitch with its required starting value.
int16_t gyro_yaw = 0;    // Initialize gyro_yaw with its required starting value.

// -------------------------------
// Calibration offsets
// -------------------------------
int16_t gyro_roll_cal = 0;   // Initialize gyro_roll_cal with its required starting value.
int16_t gyro_pitch_cal = 0;  // Initialize gyro_pitch_cal with its required starting value.
int16_t gyro_yaw_cal = 0;    // Initialize gyro_yaw_cal with its required starting value.

int16_t manual_acc_pitch_cal_value = 0;   // Initialize manual_acc_pitch_cal_value with its required starting value.
int16_t manual_acc_roll_cal_value = 0;    // Initialize manual_acc_roll_cal_value with its required starting value.
int16_t manual_gyro_roll_cal_value = 0;   // Initialize manual_gyro_roll_cal_value with its required starting value.
int16_t manual_gyro_pitch_cal_value = 0;  // Initialize manual_gyro_pitch_cal_value with its required starting value.
int16_t manual_gyro_yaw_cal_value = 0;    // Initialize manual_gyro_yaw_cal_value with its required starting value.
int32_t acc_total_vector = 0;             // Initialize acc_total_vector with its required starting value.

// -------------------------------
// Misc
// -------------------------------
uint8_t i2cError = 0;                 // I2C communication error counter
int16_t cal_int = 0;                  // calibration loop counter
bool use_manual_calibration = false;  // Initialize use_manual_calibration with its required starting value.

// -------------------------------
// Setup MPU-6050 registers
// -------------------------------

namespace {  // Keep MPU6050 register configuration helpers private to this module.

constexpr uint8_t kMpuAddress = 0x68U;          // Preserve the tested MPU6050 I2C address.
constexpr uint8_t kMpuSetupRetryCount = 5U;     // Retry each register write/read-back several times before reporting setup failure.
constexpr uint32_t kMpuSetupRetryDelayMs = 2U;  // Leave a short recovery interval between failed setup transactions.

bool readMpuRegister(uint8_t reg, uint8_t &value) {                                        // Read one MPU6050 register so setup writes can be verified.
  Wire.beginTransmission(kMpuAddress);                                                     // Address the MPU6050 before selecting the register.
  Wire.write(reg);                                                                         // Select the register that must be read back.
  if (Wire.endTransmission(false) != 0U) {                                                 // Require a successful repeated-start register selection.
    return false;                                                                          // Report the failed I2C transaction to the setup retry helper.
  }                                                                                        // Finish validating register selection.
  const uint8_t received = Wire.requestFrom((uint8_t)kMpuAddress, (size_t)1, (bool)true);  // Request exactly one register byte and release the bus afterward.
  if (received != 1U || Wire.available() < 1) {                                            // Require exactly one available byte before accepting the read-back.
    return false;                                                                          // Report a short or missing register read.
  }                                                                                        // Finish validating the read-back length.
  value = static_cast<uint8_t>(Wire.read());                                               // Publish the verified register byte to the caller.
  return true;                                                                             // Report a complete register read.
}  // Finish reading one MPU6050 register.

bool writeAndVerifyMpuRegister(uint8_t reg, uint8_t value) {              // Write one required MPU6050 setting and confirm it by reading the register back.
  for (uint8_t attempt = 0U; attempt < kMpuSetupRetryCount; attempt++) {  // Retry transient boot-time I2C failures without accepting an unknown configuration.
    Wire.beginTransmission(kMpuAddress);                                  // Address the MPU6050 before writing the target register.
    Wire.write(reg);                                                      // Select the register that must be configured.
    Wire.write(value);                                                    // Write the exact required configuration byte.
    if (Wire.endTransmission() == 0U) {                                   // Continue only when the write transaction was acknowledged.
      uint8_t readBack = 0U;                                              // Reserve one byte for independent read-back verification.
      if (readMpuRegister(reg, readBack) && readBack == value) {          // Accept the setting only when the sensor returns the exact requested value.
        i2cError = 0U;                                                    // Clear setup communication errors after one fully verified register.
        return true;                                                      // Report successful write-and-verify completion.
      }                                                                   // Finish validating the read-back value.
    }                                                                     // Finish validating the register write.
    if (i2cError < 255U) i2cError++;                                      // Count the failed setup attempt without allowing overflow.
    delay(kMpuSetupRetryDelayMs);                                         // Give the I2C device a short recovery interval before retrying.
    yield();                                                              // Service ESP8266 background work during boot-time setup retries.
  }                                                                       // Finish the bounded retry sequence for this register.
  return false;                                                           // Report that the required register could not be verified.
}  // Finish writing and verifying one MPU6050 register.

}  // Finish the private MPU6050 configuration helper namespace.

bool gyroSetup() {                                             // Configure every required MPU6050 register and verify each setting by read-back.
  if (!writeAndVerifyMpuRegister(0x6BU, 0x00U)) return false;  // Wake the MPU6050 and require confirmation from PWR_MGMT_1.
  if (!writeAndVerifyMpuRegister(0x1BU, 0x10U)) return false;  // Configure the gyroscope for the tested +/-1000 dps full-scale range.
  if (!writeAndVerifyMpuRegister(0x1CU, 0x10U)) return false;  // Configure the accelerometer for the tested +/-8g full-scale range.
  if (!writeAndVerifyMpuRegister(0x1AU, 0x03U)) return false;  // Configure the tested approximately 43-Hz digital low-pass filter.
  i2cError = 0U;                                               // Leave the shared error state clear after complete verified MPU setup.
  return true;                                                 // Report that every mandatory MPU6050 setting was written and confirmed.
}  // Finish the verified MPU6050 setup sequence.

// -------------------------------
// Read raw gyro & accelerometer data
// -------------------------------
bool gyroSignalen() {  // Read one complete MPU6050 sample and report whether the transaction succeeded.

  // Set start register
  Wire.beginTransmission(0x68);            // Invoke or continue beginTransmission for the current operation.
  Wire.write(0x3B);                        // Invoke or continue write for the current operation.
  if (Wire.endTransmission(false) != 0) {  // Use a repeated start to reduce I2C transaction overhead.
    if (i2cError < 255U) i2cError++;       // Count this consecutive I2C failure without allowing counter overflow.
    return false;                          // Report that this MPU6050 transaction failed.
  }                                        // Close the current declaration or execution block.

  // Read 14 bytes
  uint8_t n = Wire.requestFrom((uint8_t)0x68, (size_t)14, (bool)true);  // Initialize n with its required starting value.
  if (n < 14) {                                                         // Evaluate this condition before executing its protected branch.
    if (i2cError < 255U) i2cError++;                                    // Count this consecutive short-read failure without allowing counter overflow.
    return false;                                                       // Report that this MPU6050 transaction failed.
  }                                                                     // Close the current declaration or execution block.

  // Read raw data
  int16_t raw_acc_x_new = (Wire.read() << 8) | Wire.read();  // Initialize raw_acc_x_new with its required starting value.
  int16_t raw_acc_y_new = (Wire.read() << 8) | Wire.read();  // Initialize raw_acc_y_new with its required starting value.
  int16_t raw_acc_z_new = (Wire.read() << 8) | Wire.read();  // Initialize raw_acc_z_new with its required starting value.

  Wire.read();  // Invoke or continue read for the current operation.
  Wire.read();  // temperature - skipped

  int16_t gyro_roll_old_new = (Wire.read() << 8) | Wire.read();   // Initialize gyro_roll_old_new with its required starting value.
  int16_t gyro_pitch_old_new = (Wire.read() << 8) | Wire.read();  // Initialize gyro_pitch_old_new with its required starting value.
  int16_t gyro_yaw_old_new = (Wire.read() << 8) | Wire.read();    // Initialize gyro_yaw_old_new with its required starting value.


  i2cError = 0U;  // Clear the consecutive error counter after one complete valid MPU6050 sample.

  // Update raw values
  raw_acc_x = raw_acc_x_new;  // Update raw_acc_x with the value calculated on this line.
  raw_acc_y = raw_acc_y_new;  // Update raw_acc_y with the value calculated on this line.
  raw_acc_z = raw_acc_z_new;  // Update raw_acc_z with the value calculated on this line.

  gyro_roll_old = gyro_roll_old_new;    // Update gyro_roll_old with the value calculated on this line.
  gyro_pitch_old = gyro_pitch_old_new;  // Update gyro_pitch_old with the value calculated on this line.
  gyro_yaw_old = gyro_yaw_old_new;      // Update gyro_yaw_old with the value calculated on this line.


  // Axis transform
  gyro_roll = gyro_pitch_old;   // Update gyro_roll with the value calculated on this line.
  gyro_pitch = -gyro_roll_old;  // Update gyro_pitch with the value calculated on this line.
  gyro_yaw = gyro_yaw_old;      // Update gyro_yaw with the value calculated on this line.

  acc_x = raw_acc_y;   // Update acc_x with the value calculated on this line.
  acc_y = -raw_acc_x;  // Update acc_y with the value calculated on this line.
  acc_z = raw_acc_z;   // Update acc_z with the value calculated on this line.

  gyro_pitch = -gyro_pitch;  // Update gyro_pitch with the value calculated on this line.
  gyro_yaw = -gyro_yaw;      // Update gyro_yaw with the value calculated on this line.


  // Calibration offset
  acc_y -= manual_acc_pitch_cal_value;        // Subtract from acc_y with the value calculated on this line.
  acc_x -= manual_acc_roll_cal_value;         // Subtract from acc_x with the value calculated on this line.
  gyro_roll -= manual_gyro_roll_cal_value;    // Subtract from gyro_roll with the value calculated on this line.
  gyro_pitch -= manual_gyro_pitch_cal_value;  // Subtract from gyro_pitch with the value calculated on this line.
  gyro_yaw -= manual_gyro_yaw_cal_value;      // Subtract from gyro_yaw with the value calculated on this line.

  return true;  // Report that one complete fresh MPU6050 sample was accepted and published.
}  // Close the current declaration or execution block.

// -------------------------------
// Calibrate gyro offsets
// -------------------------------
void calibrateGyro() {                         // Begin the calibrateGyro function implementation.
  if (use_manual_calibration) cal_int = 2000;  // Evaluate this condition before executing its protected branch.
  else {                                       // Enter the alternative branch.
    cal_int = 0;                               // Update cal_int with the value calculated on this line.
    manual_gyro_pitch_cal_value = 0;           // Update manual_gyro_pitch_cal_value with the value calculated on this line.
    manual_gyro_roll_cal_value = 0;            // Update manual_gyro_roll_cal_value with the value calculated on this line.
    manual_gyro_yaw_cal_value = 0;             // Update manual_gyro_yaw_cal_value with the value calculated on this line.
  }                                            // Close the current declaration or execution block.

  if (cal_int != 2000) {         // Evaluate this condition before executing its protected branch.
    int32_t gyro_roll_cal = 0;   // Initialize gyro_roll_cal with its required starting value.
    int32_t gyro_pitch_cal = 0;  // Initialize gyro_pitch_cal with its required starting value.
    int32_t gyro_yaw_cal = 0;    // Initialize gyro_yaw_cal with its required starting value.

    cal_int = 0;                                                    // Count only complete and successful MPU6050 samples.
    while (cal_int < 2000) {                                        // Continue until exactly two thousand valid samples have been accumulated.
      if (!gyroSignalen()) {                                        // Reject failed or short I2C transactions from the calibration average.
        delay(4);                                                   // Preserve the original sampling pause before retrying.
        yield();                                                    // Allow ESP8266 background processing while waiting for a valid sensor sample.
        continue;                                                   // Retry without incrementing the valid-sample counter.
      }                                                             // Finish validating this calibration sample.
      if (cal_int % 25 == 0) digitalWrite(LED, !digitalRead(LED));  // Blink only as valid calibration samples progress.
      gyro_roll_cal += gyro_roll;                                   // Accumulate this fresh valid roll sample.
      gyro_pitch_cal += gyro_pitch;                                 // Accumulate this fresh valid pitch sample.
      gyro_yaw_cal += gyro_yaw;                                     // Accumulate this fresh valid yaw sample.
      cal_int++;                                                    // Count this sample only after a successful complete MPU6050 read.
      delay(4);                                                     // Preserve the original calibration sample spacing.
      yield();                                                      // Keep ESP8266 background work serviced during the calibration period.
    }                                                               // Finish after exactly two thousand valid samples.
    digitalWrite(LED, HIGH);                                        // Invoke or continue digitalWrite for the current operation.

    // Compute average offsets
    manual_gyro_pitch_cal_value = gyro_pitch_cal / 2000;  // Update manual_gyro_pitch_cal_value with the value calculated on this line.
    manual_gyro_roll_cal_value = gyro_roll_cal / 2000;    // Update manual_gyro_roll_cal_value with the value calculated on this line.
    manual_gyro_yaw_cal_value = gyro_yaw_cal / 2000;      // Update manual_gyro_yaw_cal_value with the value calculated on this line.
  }                                                       // Close the current declaration or execution block.
}  // Close the current declaration or execution block.

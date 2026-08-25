#ifndef IMU_H  // Start the IMU_H include guard.
#define IMU_H  // Define the IMU_H preprocessor symbol.

#include <Arduino.h>  // Import declarations supplied by <Arduino.h>.
#include <stdint.h>   // Import declarations supplied by <stdint.h>.

// -------------------------------
// Function declarations
// -------------------------------

// Raw accelerometer data
extern int16_t raw_acc_x;  // Declare the shared raw_acc_x symbol defined by another module.
extern int16_t raw_acc_y;  // Declare the shared raw_acc_y symbol defined by another module.
extern int16_t raw_acc_z;  // Declare the shared raw_acc_z symbol defined by another module.

// Processed accelerometer data
extern int16_t acc_x;  // Declare the shared acc_x symbol defined by another module.
extern int16_t acc_y;  // Declare the shared acc_y symbol defined by another module.
extern int16_t acc_z;  // Declare the shared acc_z symbol defined by another module.

// Raw gyro data
extern int16_t gyro_roll_old;   // Declare the shared gyro_roll_old symbol defined by another module.
extern int16_t gyro_pitch_old;  // Declare the shared gyro_pitch_old symbol defined by another module.
extern int16_t gyro_yaw_old;    // Declare the shared gyro_yaw_old symbol defined by another module.

// Processed gyro data
extern int16_t gyro_roll;   // Declare the shared gyro_roll symbol defined by another module.
extern int16_t gyro_pitch;  // Declare the shared gyro_pitch symbol defined by another module.
extern int16_t gyro_yaw;    // Declare the shared gyro_yaw symbol defined by another module.

// Calibration offsets
extern int16_t gyro_roll_cal;   // Declare the shared gyro_roll_cal symbol defined by another module.
extern int16_t gyro_pitch_cal;  // Declare the shared gyro_pitch_cal symbol defined by another module.
extern int16_t gyro_yaw_cal;    // Declare the shared gyro_yaw_cal symbol defined by another module.

// I2C communication error counter
extern uint8_t i2cError;  // Declare the shared i2cError symbol defined by another module.

// Calibration loop counter
extern int16_t cal_int;  // Declare the shared cal_int symbol defined by another module.

// Optional flags / config
extern bool use_manual_calibration;          // Declare the shared use_manual_calibration symbol defined by another module.
extern int16_t manual_acc_roll_cal_value;    // Declare the shared manual_acc_roll_cal_value symbol defined by another module.
extern int16_t manual_acc_pitch_cal_value;   // Declare the shared manual_acc_pitch_cal_value symbol defined by another module.
extern int16_t manual_gyro_roll_cal_value;   // Declare the shared manual_gyro_roll_cal_value symbol defined by another module.
extern int16_t manual_gyro_pitch_cal_value;  // Declare the shared manual_gyro_pitch_cal_value symbol defined by another module.
extern int16_t manual_gyro_yaw_cal_value;    // Declare the shared manual_gyro_yaw_cal_value symbol defined by another module.
extern int32_t acc_total_vector;             // Declare the shared acc_total_vector symbol defined by another module.

// Setup MPU-6050 registers
bool gyroSetup();  // Configure and read-back verify every mandatory MPU6050 register.

// Read raw gyro & accelerometer data
bool gyroSignalen();  // Read one complete MPU6050 sample and report whether the transaction succeeded.

// Calibrate gyro offsets
void calibrateGyro();  // Invoke or continue calibrateGyro for the current operation.

#endif  // Close the current preprocessor guard.

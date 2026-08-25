#include "pid.h"  // Provide the public attitude PID variables and functions.

// -----------------------------------------------------------------------------
// Preserved Roll PID gains and output limit
// -----------------------------------------------------------------------------
float pid_p_gain_roll = 1.3f;   // Preserve the tested Roll proportional gain.
float pid_i_gain_roll = 0.04f;  // Preserve the tested Roll integral gain.
float pid_d_gain_roll = 18.0f;  // Preserve the tested Roll derivative gain.
int pid_max_roll = 800;         // Preserve the tested Roll PID output limit.

// -----------------------------------------------------------------------------
// Preserved Pitch PID gains and output limit
// -----------------------------------------------------------------------------
float pid_p_gain_pitch = pid_p_gain_roll;  // Preserve the original Pitch gain mirroring the Roll proportional gain.
float pid_i_gain_pitch = pid_i_gain_roll;  // Preserve the original Pitch gain mirroring the Roll integral gain.
float pid_d_gain_pitch = pid_d_gain_roll;  // Preserve the original Pitch gain mirroring the Roll derivative gain.
int pid_max_pitch = pid_max_roll;          // Preserve the original Pitch output limit mirroring the Roll output limit.

// -----------------------------------------------------------------------------
// Preserved Yaw PID gains and output limit
// -----------------------------------------------------------------------------
float pid_p_gain_yaw = 4.0f;   // Preserve the tested Yaw proportional gain.
float pid_i_gain_yaw = 0.02f;  // Preserve the tested Yaw integral gain.
float pid_d_gain_yaw = 0.0f;   // Preserve the tested Yaw derivative gain.
int pid_max_yaw = 800;         // Preserve the tested Yaw PID output limit.

// -----------------------------------------------------------------------------
// Preserved Altitude PID gains and output limit
// -----------------------------------------------------------------------------
float pid_p_gain_alt = 0.35f;   // Preserve the previously flight-tested altitude proportional gain.
float pid_i_gain_alt = 0.006f;  // Preserve the previously flight-tested altitude integral gain.
float pid_d_gain_alt = 85.0f;   // Preserve the previously flight-tested altitude derivative gain.
int pid_max_alt = 800;          // Preserve the previously flight-tested altitude PID output limit.

bool auto_level = true;  // Preserve automatic attitude leveling as enabled by default.

// -----------------------------------------------------------------------------
// Roll PID runtime variables
// -----------------------------------------------------------------------------
float pid_output_roll = 0.0f;        // Start the Roll PID output at zero.
float gyro_roll_input = 0.0f;        // Start the filtered Roll gyro input at zero.
float pid_roll_setpoint = 0.0f;      // Start the Roll rate setpoint at zero.
float pid_i_mem_roll = 0.0f;         // Start the Roll integral accumulator at zero.
float pid_last_roll_d_error = 0.0f;  // Start the previous Roll derivative error at zero.

// -----------------------------------------------------------------------------
// Pitch PID runtime variables
// -----------------------------------------------------------------------------
float pid_output_pitch = 0.0f;        // Start the Pitch PID output at zero.
float gyro_pitch_input = 0.0f;        // Start the filtered Pitch gyro input at zero.
float pid_pitch_setpoint = 0.0f;      // Start the Pitch rate setpoint at zero.
float pid_i_mem_pitch = 0.0f;         // Start the Pitch integral accumulator at zero.
float pid_last_pitch_d_error = 0.0f;  // Start the previous Pitch derivative error at zero.

// -----------------------------------------------------------------------------
// Yaw PID runtime variables
// -----------------------------------------------------------------------------
float pid_output_yaw = 0.0f;        // Start the Yaw PID output at zero.
float gyro_yaw_input = 0.0f;        // Start the filtered Yaw gyro input at zero.
float pid_yaw_setpoint = 0.0f;      // Start the Yaw rate setpoint at zero.
float pid_i_mem_yaw = 0.0f;         // Start the Yaw integral accumulator at zero.
float pid_last_yaw_d_error = 0.0f;  // Start the previous Yaw derivative error at zero.

// -----------------------------------------------------------------------------
// Altitude PID runtime variables
// -----------------------------------------------------------------------------
float pid_output_alt = 0.0f;        // Start the altitude throttle correction at zero.
float pid_alt_setpoint = 0.0f;      // Start without an active altitude target.
float pid_i_mem_alt = 0.0f;         // Start the altitude integral accumulator at zero.
float pid_last_alt_d_error = 0.0f;  // Start the previous altitude derivative error at zero.

float calculateSinglePid(float input, float setpoint,                        // Calculate one PID axis using the original formula.
                         float &i_mem, float i_gain, float max_output,       // Receive the integral state, integral gain, and output limit.
                         float p_gain, float d_gain, float &last_d_error) {  // Receive the proportional gain, derivative gain, and previous error.
  float error = input - setpoint;                                            // Calculate the original controller error direction.

  i_mem += i_gain * error;                            // Accumulate the original integral contribution.
  if (i_mem > max_output) i_mem = max_output;         // Limit positive integral windup to the configured output limit.
  else if (i_mem < -max_output) i_mem = -max_output;  // Limit negative integral windup to the configured output limit.

  float output = p_gain * error + i_mem + d_gain * (error - last_d_error);  // Calculate the original proportional, integral, and derivative sum.

  if (output > max_output) output = max_output;         // Limit the positive PID output.
  else if (output < -max_output) output = -max_output;  // Limit the negative PID output.

  last_d_error = error;  // Store the current error for the next derivative calculation.
  return output;         // Return the bounded PID output.
}  // Finish calculating one PID axis.

void primePidDerivativeHistory() {                                      // Prime only the derivative history without changing gains, integrators, or PID outputs.
  pid_last_roll_d_error = gyro_roll_input - pid_roll_setpoint;          // Match the stored Roll derivative error to the current Roll controller error.
  pid_last_pitch_d_error = gyro_pitch_input - pid_pitch_setpoint;       // Match the stored Pitch derivative error to the current Pitch controller error.
  pid_last_yaw_d_error = gyro_yaw_input - pid_yaw_setpoint;             // Match the stored Yaw derivative error to the current Yaw controller error.
}  // Finish priming derivative history for a kick-free PID restart.

void calculatePid(void) {                                      // Calculate the original attitude PID outputs for the retained flight axes.
  pid_output_roll = calculateSinglePid(                        // Calculate the Roll PID output.
    gyro_roll_input, pid_roll_setpoint,                        // Use the filtered Roll input and commanded Roll setpoint.
    pid_i_mem_roll, pid_i_gain_roll, pid_max_roll,             // Use the Roll integral state, gain, and output limit.
    pid_p_gain_roll, pid_d_gain_roll, pid_last_roll_d_error);  // Use the Roll proportional gain, derivative gain, and previous error.

  pid_output_pitch = calculateSinglePid(                          // Calculate the Pitch PID output.
    gyro_pitch_input, pid_pitch_setpoint,                         // Use the filtered Pitch input and commanded Pitch setpoint.
    pid_i_mem_pitch, pid_i_gain_pitch, pid_max_pitch,             // Use the Pitch integral state, gain, and output limit.
    pid_p_gain_pitch, pid_d_gain_pitch, pid_last_pitch_d_error);  // Use the Pitch proportional gain, derivative gain, and previous error.

  pid_output_yaw = calculateSinglePid(                      // Calculate the Yaw PID output.
    gyro_yaw_input, pid_yaw_setpoint,                       // Use the filtered Yaw input and commanded Yaw setpoint.
    pid_i_mem_yaw, pid_i_gain_yaw, pid_max_yaw,             // Use the Yaw integral state, gain, and output limit.
    pid_p_gain_yaw, pid_d_gain_yaw, pid_last_yaw_d_error);  // Use the Yaw proportional gain, derivative gain, and previous error.
}  // Finish calculating all attitude PID outputs.

void calculateAltitudePid(float filteredDistanceMm) {  // Calculate the altitude correction with the exact error direction used by the previous working firmware.
  pid_output_alt = calculateSinglePid(                       // Reuse the same tested PID formula as the attitude axes.
    pid_alt_setpoint, filteredDistanceMm,                     // Preserve old argument order so error = target height - measured height.
    pid_i_mem_alt, pid_i_gain_alt, pid_max_alt,               // Use preserved altitude integral state, gain, and output limit.
    pid_p_gain_alt, pid_d_gain_alt, pid_last_alt_d_error);    // Use preserved altitude proportional/derivative gains and history.
}

void resetAltitudePidRuntime() {  // Clear altitude PID memory without altering its tested tuning values.
  pid_output_alt = 0.0f;          // Remove the current altitude throttle correction.
  pid_alt_setpoint = 0.0f;        // Clear the current altitude target.
  pid_i_mem_alt = 0.0f;           // Clear accumulated altitude integral state.
  pid_last_alt_d_error = 0.0f;    // Clear derivative history for the next assisted-flight session.
}

void resetPidRuntime() {          // Clear every attitude PID memory and output without changing configured gains.
  pid_output_roll = 0.0f;         // Clear the Roll PID output.
  pid_i_mem_roll = 0.0f;          // Clear the Roll integral accumulator.
  pid_last_roll_d_error = 0.0f;   // Clear the previous Roll derivative error.
  pid_output_pitch = 0.0f;        // Clear the Pitch PID output.
  pid_i_mem_pitch = 0.0f;         // Clear the Pitch integral accumulator.
  pid_last_pitch_d_error = 0.0f;  // Clear the previous Pitch derivative error.
  pid_output_yaw = 0.0f;          // Clear the Yaw PID output.
  pid_i_mem_yaw = 0.0f;           // Clear the Yaw integral accumulator.
  pid_last_yaw_d_error = 0.0f;    // Clear the previous Yaw derivative error.
}  // Finish clearing all attitude PID runtime state.

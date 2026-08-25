#ifndef PID_H  // Prevent this PID header from being included more than once.
#define PID_H  // Define the include guard used by this PID header.

#include <Arduino.h>  // Provide Arduino Boolean and numeric types.

// -----------------------------------------------------------------------------
// Roll PID gains and output limit
// -----------------------------------------------------------------------------
extern float pid_p_gain_roll;  // Store the proportional gain used by the Roll controller.
extern float pid_i_gain_roll;  // Store the integral gain used by the Roll controller.
extern float pid_d_gain_roll;  // Store the derivative gain used by the Roll controller.
extern int pid_max_roll;       // Store the absolute Roll PID output limit.

// -----------------------------------------------------------------------------
// Pitch PID gains and output limit
// -----------------------------------------------------------------------------
extern float pid_p_gain_pitch;  // Store the proportional gain used by the Pitch controller.
extern float pid_i_gain_pitch;  // Store the integral gain used by the Pitch controller.
extern float pid_d_gain_pitch;  // Store the derivative gain used by the Pitch controller.
extern int pid_max_pitch;       // Store the absolute Pitch PID output limit.

// -----------------------------------------------------------------------------
// Yaw PID gains and output limit
// -----------------------------------------------------------------------------
extern float pid_p_gain_yaw;  // Store the proportional gain used by the Yaw controller.
extern float pid_i_gain_yaw;  // Store the integral gain used by the Yaw controller.
extern float pid_d_gain_yaw;  // Store the derivative gain used by the Yaw controller.
extern int pid_max_yaw;       // Store the absolute Yaw PID output limit.

extern bool auto_level;  // Preserve the original automatic attitude-leveling enable flag.

// -----------------------------------------------------------------------------
// Roll PID runtime state
// -----------------------------------------------------------------------------
extern float pid_output_roll;        // Store the newest Roll PID output.
extern float gyro_roll_input;        // Store the filtered Roll gyro input in degrees per second.
extern float pid_roll_setpoint;      // Store the commanded Roll rate setpoint.
extern float pid_i_mem_roll;         // Store the Roll integral accumulator.
extern float pid_last_roll_d_error;  // Store the previous Roll error used by the derivative term.

// -----------------------------------------------------------------------------
// Pitch PID runtime state
// -----------------------------------------------------------------------------
extern float pid_output_pitch;        // Store the newest Pitch PID output.
extern float gyro_pitch_input;        // Store the filtered Pitch gyro input in degrees per second.
extern float pid_pitch_setpoint;      // Store the commanded Pitch rate setpoint.
extern float pid_i_mem_pitch;         // Store the Pitch integral accumulator.
extern float pid_last_pitch_d_error;  // Store the previous Pitch error used by the derivative term.

// -----------------------------------------------------------------------------
// Yaw PID runtime state
// -----------------------------------------------------------------------------
extern float pid_output_yaw;        // Store the newest Yaw PID output.
extern float gyro_yaw_input;        // Store the filtered Yaw gyro input in degrees per second.
extern float pid_yaw_setpoint;      // Store the commanded Yaw rate setpoint.
extern float pid_i_mem_yaw;         // Store the Yaw integral accumulator.
extern float pid_last_yaw_d_error;  // Store the previous Yaw error used by the derivative term.

float calculateSinglePid(float input, float setpoint,                       // Calculate one preserved PID axis from its input and setpoint.
                         float &i_mem, float i_gain, float max_output,      // Receive the integral state, integral gain, and output limit.
                         float p_gain, float d_gain, float &last_d_error);  // Receive the proportional gain, derivative gain, and previous error.


// -----------------------------------------------------------------------------
// Altitude PID runtime state (preserved from the previous working VL53L0X build)
// -----------------------------------------------------------------------------
extern float pid_p_gain_alt;          // Preserve the tested altitude proportional gain.
extern float pid_i_gain_alt;          // Preserve the tested altitude integral gain.
extern float pid_d_gain_alt;          // Preserve the tested altitude derivative gain.
extern int pid_max_alt;               // Preserve the tested altitude PID output limit.
extern float pid_output_alt;          // Store the newest altitude PID throttle correction.
extern float pid_alt_setpoint;        // Store the requested altitude target in millimeters.
extern float pid_i_mem_alt;           // Store the altitude integral accumulator.
extern float pid_last_alt_d_error;    // Store the previous altitude PID error for the derivative term.

void calculateAltitudePid(float filteredDistanceMm);  // Calculate only the preserved altitude PID correction using the same argument order as the previous working firmware.
void resetAltitudePidRuntime();                       // Clear altitude PID runtime state without changing its tested gains.

void calculatePid(void);                    // Calculate the preserved Roll, Pitch, and Yaw PID outputs.
void primePidDerivativeHistory();           // Align derivative history with the current error so the next PID calculation has no D-kick.
void resetPidRuntime();                     // Clear all attitude PID memories and outputs after arming changes or safety shutdowns.

#endif  // Close the PID_H include guard.

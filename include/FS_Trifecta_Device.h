/// Driver for the Trifecta series of IMU/AHRS/INS devices
/// Copyright 2024 4rge.ai and/or Triangle Man LLC
/// Usage and redistribution of this code is permitted
/// but this notice must be retained in all copies of the code.

/// THIS SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
/// INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE,
/// AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
/// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#ifndef TRIFECTA_DEVICE_H
#define TRIFECTA_DEVICE_H

#include "FS_Trifecta_Defs.h"
#include "FS_Trifecta_Interfaces.h"

// Command definitions

#define CMD_RESTART 'R'       // Restart the device...
#define CMD_CLEAR_CONFIG 'C'  // Clear all saved settings...
#define CMD_SET_SSID 'S'      // Set SSID for wifi connection (UART only)
#define CMD_SET_PASSWORD 'P'  // Set password for wifi connection (UART only)
#define CMD_SET_FILT_BETA 'B' // Set the beta of the orientation filter
#define CMD_SET_DEV_NAME 'N'  // Set device name
#define CMD_SETUP_FINISH 'F'  // Exit setup and begin the telemetry mode

#define CMD_IDENTIFY 'I'                      // Request identification (device name)
#define CMD_IDENTIFY_PARAM_DEVMODEL 'm'       // Respond with device model name
#define CMD_IDENTIFY_PARAM_ACCELRANGE 'a'     // Respond with max acceleration range
#define CMD_IDENTIFY_PARAM_GYRORANGE 'g'      // Respond with max gyro range
#define CMD_IDENTIFY_PARAM_REFRESHRATE 'r'    // Refresh rate (typically 200 Hz)
#define CMD_IDENTIFY_PARAM_UART_BAUD_RATE 'b' // UART baud rate
#define CMD_IDENTIFY_PARAM_SSID 's'           // Currently set SSID for wifi connection
#define CMD_IDENTIFY_PARAM_TRANSMIT 't'       // Transmit mode (see communication_mode_t in d_config.h)
#define CMD_IDENTIFY_PARAM_I2C_ENABLED 'i'    // Respond with i2c address, or 0 if i2c streaming is off

#define CMD_REZERO_IMUS 'Z'            // Re-calibrate the IMUs (should only do on a flat plane and stationary)
#define CMD_TOGGLE_REZERO_AT_START 'K' // Whether or not to re-zero the IMU each time the device reboots

#define CMD_REZERO_INS '0'  // Set the INS position back to zero (such as when an external update is done using GNSS)
#define CMD_SET_YAW_DEG 'y' // Set yaw angle to the given argument (degrees)

#define CMD_STREAM 'A' // Start or stop streaming data (parameter 1 == start, anything else == stop)

#define SET_I2C_OUTPUT_ADDRESS '@' // Set I2C address - not available for now!
#define SET_I2C_OUTPUT_ENABLE '2'  // Enable I2C data output (if available on hardware) - 0 to turn off, 1 to turn on

#define CMD_DISPLAYMODE 'D' // Set display mode (0 = debug - ACCEL, 1 = debug - GYRO, 2 = debug - ORIENTATION, 3 = show axis)

#ifdef __cplusplus
extern "C"
{
#endif

    int fs_handle_received_commands(fs_device_info *device_info, const void *cmd_buf, size_t buf_len);
    int fs_device_parse_packet(fs_device_info *device_info, const void *rx_buf, size_t rx_len, fs_communication_mode source);
    void fs_q_to_euler_angles(float *estRoll, float *estPitch, float *estYaw, float q0, float q1, float q2, float q3, bool degrees);

#ifdef __cplusplus
}
#endif

#endif
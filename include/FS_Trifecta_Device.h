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

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum
    {
        CMD_RESTART = 'R',         // Restart the device "R0;"
        CMD_CLEAR_CONFIG = 'C',    // Clear all saved settings "C0;"
        CMD_SET_SSID = 'S',        // Set SSID for WiFi connection (UART only) "S<STA_SSID>;"
        CMD_SET_PASSWORD = 'P',    // Set password for WiFi connection (UART only) "P<STA_PASSWORD>;"
        CMD_SET_SSID_AP = 'q',     // Set SSID for access point WiFi connection (UART only) "q<AP_SSID>;"
        CMD_SET_PASSWORD_AP = 'w', // Set password for access point WiFi connection (UART only) "w<AP_PASSWORD>"
        CMD_SET_FILT_BETA = 'B',   // Set the beta of the orientation filter ? Unused?
        CMD_SET_DEV_NAME = 'N',    // Set device name "N<DEVICE_NAME>;"
        CMD_SETUP_FINISH = 'F',    // Exit setup and begin telemetry mode "F0;"

        CMD_IDENTIFY = 'I',                      // Request identification (device name) "I0;"
        CMD_IDENTIFY_PARAM_DEV_SN = 'p',         // Respond with device serial number (will be set by eFuse on production model) "p0;"
        CMD_IDENTIFY_PARAM_DEVMODEL = 'm',       // Respond with device model name "m0;"
        CMD_IDENTIFY_PARAM_DEVFWVERSION = 'f',   // Respond with device firmware version "f0;"
        CMD_IDENTIFY_PARAM_DEVDESC = 'd',        // Respond with device description "d0;"
        CMD_IDENTIFY_PARAM_ACCELRANGE = 'a',     // Respond with max acceleration range "a0;"
        CMD_IDENTIFY_PARAM_GYRORANGE = 'g',      // Respond with max gyro range "g0;"
        CMD_IDENTIFY_PARAM_REFRESHRATE = 'r',    // Respond with refresh rate (typically 200 Hz) "r0;"
        CMD_IDENTIFY_PARAM_UART_BAUD_RATE = 'b', // Respond with UART baud rate "b0;"
        CMD_IDENTIFY_PARAM_SSID = 's',           // Respond with current SSID (STA) for WiFi connection "s0;"
        CMD_IDENTIFY_PARAM_SSID_AP = '1',        // Respond with current SSID (AP) for WiFi connection "l0;"
        CMD_IDENTIFY_PARAM_TRANSMIT = 't',       // Respond with transmit mode (serial/UDP/etc.) "t0;" to query, "t<COMMUNICATION_MODE_1 | 2 | ... |>;" to set

        CMD_IDENTIFY_PARAM_I2C_ENABLED = 'i', // Respond with I2C address or 0 if I2C streaming is off (unused?)

        CMD_REZERO_IMUS = 'Z',            // Re-calibrate the IMUs (should only do on a flat plane and stationary) "Z<NUM_CALIBRATION_POINTS>;"
        CMD_TOGGLE_REZERO_AT_START = 'K', // Toggle re-zeroing IMU at device reboot - this rezero affects accelerometers "K<1 == DO NOT ELSE 0>;"

        CMD_REZERO_INS = '0',  // Reset INS position to zero "00;"
        CMD_SET_YAW_DEG = 'y', // Set yaw angle to the given argument (degrees) "y<DEG>;"

        CMD_STREAM = 'A',             // Start or stop streaming data "A<0 == STOP, 1 == STREAM, 2 == ONE SHOT READ>;"
        CMD_SET_LISTENING_PORT = 'l', // Set the target port for UDP listener on host device (default: 8888) "l<PORT 1024-65535>;"

        SET_I2C_OUTPUT_ADDRESS = '@', // Set I2C address (unused?)
        SET_I2C_OUTPUT_ENABLE = '2',  // Enable I2C data output (0 to turn off, 1 to turn on) (unused?)

        CMD_DISPLAYMODE = 'D' // Set display mode (0 = ACCEL, 1 = GYRO, etc.) "D<MODE 0-9>;"
    } fs_command_t;

    int fs_handle_received_commands(fs_device_info *device_info, const void *cmd_buf, size_t buf_len);
    int fs_device_parse_packet(fs_device_info *device_handle, const void *rx_buf, size_t rx_len, fs_communication_mode source);
    void fs_q_to_euler_angles(float *estRoll, float *estPitch, float *estYaw, float q0, float q1, float q2, float q3, bool degrees);

#ifdef __cplusplus
}
#endif

#endif
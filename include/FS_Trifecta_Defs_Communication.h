/// Driver for the Trifecta series of IMU/AHRS/INS devices
/// Copyright 2025 4rge.ai and/or Triangle Man LLC
/// Usage and redistribution of this code is permitted
/// but this notice must be retained in all copies of the code.

/// THIS SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
/// INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE,
/// AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
/// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#ifndef TRIFECTA_DEFS_COMMUNICATION_H
#define TRIFECTA_DEFS_COMMUNICATION_H

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>

#define FS_TRIFECTA_PORT 8888

#define FS_TRIFECTA_SERIAL_BAUDRATE 2000000
#define FS_MAX_DATA_LENGTH 512

#define FS_MAX_CMD_QUEUE_LENGTH 16
#define FS_MAX_CMD_LENGTH 72

#ifdef __cplusplus
extern "C"
{
#endif

    /// @brief Command definitions
    typedef enum
    {
        PACKET_HEADER = ':',       // The packet header indicates the start of a packet (legacy version)
        PACKET_FOOTER = '!',       // The packet footer indicates the end of a packet (legacy version)
        PACKET_HEADER_V2 = '@',    // The packet header indicates the start of a packet (new version)
        CMD_TERMINATOR = ';',      // The command terminator denotes the end of a command.
        CMD_RESTART = 'R',         // Restart the device "R0;"
        CMD_CLEAR_CONFIG = 'C',    // Clear all saved settings "C0;"
        CMD_SET_SSID = 'S',        // Set SSID for WiFi connection (UART only) "S<STA_SSID>;"
        CMD_SET_PASSWORD = 'P',    // Set password for WiFi connection (UART only) "P<STA_PASSWORD>;"
        CMD_SET_SSID_AP = 'q',     // Set SSID for access point WiFi connection (UART only) "q<AP_SSID>;"
        CMD_SET_PASSWORD_AP = 'w', // Set password for access point WiFi connection (UART only) "w<AP_PASSWORD>"
        CMD_SET_DEV_NAME = 'N',    // Set device name "N<DEVICE_NAME>;"
        CMD_SETUP_FINISH = 'F',    // Restart the device following setup "F0;"

        CMD_IDENTIFY = 'I',                      // Request identification (device name) "I0;"
        CMD_IDENTIFY_PARAM_DEV_SN = 'p',         // Respond with device serial number "p0;"
        CMD_IDENTIFY_PARAM_DEVMODEL = 'm',       // Respond with device model name "m0;"
        CMD_IDENTIFY_PARAM_DEVFWVERSION = 'f',   // Respond with device firmware version "f0;"
        CMD_IDENTIFY_PARAM_DEVDESC = 'd',        // Respond with device description "d0;"
        CMD_IDENTIFY_PARAM_UART_BAUD_RATE = 'b', // Respond with UART baud rate "b0;"
        CMD_IDENTIFY_PARAM_SSID = 's',           // Respond with current SSID (STA) for WiFi connection "s0;"
        CMD_IDENTIFY_PARAM_SSID_AP = '1',        // Respond with current SSID (AP) for WiFi connection "10;"
        CMD_IDENTIFY_PARAM_PASSWORD_AP = '3',    // Respond with current SSID (AP) for WiFi connection "30;"
        CMD_IDENTIFY_PARAM_TRANSMIT = 't',       // Respond with transmit mode (serial/UDP/etc.) "t-1;" to query, "t<COMMUNICATION_MODE_1 | 2 | ... |>;" to set

        CMD_REZERO_IMUS = 'Z', // Re-calibrate the IMUs (should only do on a flat plane and stationary) "Z<NUM_CALIBRATION_POINTS>;"

        CMD_SET_POSITION = '0',           // Reset INS position: "0<type>,<X>,<Y>,<Z>;" where <type> is a fs_gnss_position_format_t
        CMD_SET_ORIENTATION_OFFSET = 'Q', // Sets the offset orientation: "Q0;" to clear, "Q1;" to use current orientation, else "Q<W>,<X>,<Y>,<Z>;"
        CMD_SET_YAW_DEG = 'y',            // Set yaw angle to the given argument (degrees) "y<DEG>;"

        CMD_STREAM = 'A',             // Start or stop streaming data "A<0 == STOP, 1 == STREAM, 2 == ONE SHOT READ>;"
        CMD_SET_LISTENING_PORT = 'l', // Set the target port for UDP listener on host device (default: 8888) "l<PORT 1024-65535>;"

        CMD_OTA = '@', // Signal start of OTA - upon receiving the command "@1;", the device will begin waiting for OTA to commence.
                       // Upon receiving the command "@0;", the device will end OTA update.
                       // It will then validate the OTA data and restart the system.
                       // If the OTA was bad, it will revert to factory firmware or previous known working firmware.
                       // OTA sources can be received from UART, USB, or TCP.

        CMD_DISPLAYMODE = 'D' // Set display mode (0 = ACCEL, 1 = GYRO, etc.) "D<MODE 0-9>;"
    } fs_command_t;

    /// @brief Communication mode between the device and the host (this machine)
    typedef enum fs_communication_mode
    {
        FS_COMMUNICATION_MODE_UNINITIALIZED = -1, // This is used when not initialized
        FS_COMMUNICATION_MODE_USB_CDC = 0,        // USB-CDC always on
        FS_COMMUNICATION_MODE_UART = 1,           // UART
        FS_COMMUNICATION_MODE_TCP_UDP = 2,        // UDP streaming (Station mode)
        FS_COMMUNICATION_MODE_TCP_UDP_AP = 4,     // UDP streaming (Access point mode)
        FS_COMMUNICATION_MODE_CAN = 8,            // CAN bus -
        FS_COMMUNICATION_MODE_I2C = 16,           // I2C bus -
        FS_COMMUNICATION_MODE_ESPN = 32,          // ESP-NOW (2.4 GHz for ESP32 only) -
        FS_COMMUNICATION_MODE_SPI = 64,           // SPI -
        FS_COMMUNICATION_MODE_BLE = 128,          // Bluetooth Low Energy -
    } fs_communication_mode_t;

    /// @brief Struct primarily for defining inbound commands
    typedef struct fs_command_info
    {
        fs_communication_mode_t source;
        uint8_t payload[FS_MAX_CMD_LENGTH];
    } fs_command_info_t;

#ifdef __cplusplus
}
#endif

#endif
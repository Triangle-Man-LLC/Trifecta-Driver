/// Driver for the Trifecta series of IMU/AHRS/INS devices
/// Copyright 2024 4rge.ai and/or Triangle Man LLC
/// Usage and redistribution of this code is permitted
/// but this notice must be retained in all copies of the code.

/// THIS SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
/// INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE,
/// AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
/// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#ifndef TRIFECTA_DEFS_H
#define TRIFECTA_DEFS_H

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>

#define FS_PI 3.14159265358979f

#define FS_TRIFECTA_PORT 8888

#define FS_TRIFECTA_SERIAL_BAUDRATE 2000000
#define FS_MAX_DATA_LENGTH 512

#define FS_MAX_NUMBER_DEVICES 16

#define FS_GYRO_SCALER_DPS 500
#define FS_ACCEL_SCALER_Gs 4

#define DEGREES_TO_RADIANS 0.0174533f

#define FS_MAX_DEVICE_NUMBER 1

#define FS_MAX_CMD_QUEUE_LENGTH 4
#define FS_MAX_CMD_LENGTH 64

#define FS_MAX_PACKET_QUEUE_LENGTH 4
#define FS_MAX_PACKET_LENGTH 256

#define FS_SERIAL_PACKET_HEADER ':'
#define FS_SERIAL_PACKET_FOOTER '!'
#define FS_SERIAL_COMMAND_TERMINATOR ';'

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct fs_quaternion
    {
        float w; // Quaternion scalar term
        float x; // Quaternion vector terms
        float y; // Quaternion vector terms
        float z; // Quaternion vector terms
    } fs_quaternion;

    typedef struct fs_vector3
    {
        float x;
        float y;
        float z;
    } fs_vector3;

    /// @brief Type of packet indication
    typedef enum fs_packet_type
    {
        // The following are verbose packets - fs_imu_composite_packet
        C_PACKET_TYPE_IMU = 0,  // IMU - raw acceleration and gyroscope measurements
        C_PACKET_TYPE_AHRS = 1, // AHRS - IMU + orientation (uncorrected)
        C_PACKET_TYPE_INS = 2,  // INS - corrected AHRS + position/velocity estimation
        C_PACKET_TYPE_GNSS = 3, // GNSS - GPS and INS closed-loop - this one may need to be NMEA strings

        // The following are simplified packets - imu_packet_t
        S_PACKET_TYPE_IMU = 4,  // IMU - raw acceleration and gyroscope measurements
        S_PACKET_TYPE_AHRS = 5, // AHRS - IMU + orientation (uncorrected)
        S_PACKET_TYPE_INS = 6,  // INS - corrected AHRS + position/velocity estimation
        S_PACKET_TYPE_GNSS = 7, // GNSS - GPS and INS closed-loop - this one may need to be NMEA strings

        // The following are the new verbose packets - imu_composite_packet_2_t
        C2_PACKET_TYPE_IMU = 8,   // IMU - raw acceleration and gyroscope measurements
        C2_PACKET_TYPE_AHRS = 9,  // AHRS - IMU + orientation (uncorrected)
        C2_PACKET_TYPE_INS = 10,  // INS - corrected AHRS + position/velocity estimation
        C2_PACKET_TYPE_GNSS = 11, // GNSS - GPS and INS closed-loop - this one may need to be NMEA strings
    } fs_packet_type;

    /// @brief Command definitions
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
        CMD_IDENTIFY_PARAM_TRANSMIT = 't',       // Respond with transmit mode (serial/UDP/etc.) "t-1;" to query, "t<COMMUNICATION_MODE_1 | 2 | ... |>;" to set

        CMD_REZERO_IMUS = 'Z',            // Re-calibrate the IMUs (should only do on a flat plane and stationary) "Z<NUM_CALIBRATION_POINTS>;"
        CMD_TOGGLE_REZERO_AT_START = 'K', // Toggle re-zeroing IMU at device reboot - this rezero affects accelerometers "K<1 == DO NOT ELSE 0>;"

        CMD_REZERO_INS = '0',  // Reset INS position to zero "00;"
        CMD_SET_YAW_DEG = 'y', // Set yaw angle to the given argument (degrees) "y<DEG>;"

        CMD_STREAM = 'A',             // Start or stop streaming data "A<0 == STOP, 1 == STREAM, 2 == ONE SHOT READ>;"
        CMD_SET_LISTENING_PORT = 'l', // Set the target port for UDP listener on host device (default: 8888) "l<PORT 1024-65535>;"
    } fs_command_t;

    /// @brief This data format used for TCP/UDP sending, it has all the data but will saturate serial transmitters
    typedef struct fs_imu_composite_packet
    {
        uint8_t type;  // Packet type (0 = telemetry only, 1 = orientation only, 2 = orientation and velocity, 3 = orientation and positioning, 4 = GPS)
        uint32_t time; // Current time (in RTOS ticks)

        float ax0; // Unprocessed accelerometer value x
        float ay0; // Unprocessed accelerometer value y
        float az0; // Unprocessed accelerometer value z
        float gx0; // Unprocessed gyroscope value x
        float gy0; // Unprocessed gyroscope value y
        float gz0; // Unprocessed gyroscope value z

        float ax1; // Unprocessed accelerometer value x
        float ay1; // Unprocessed accelerometer value y
        float az1; // Unprocessed accelerometer value z
        float gx1; // Unprocessed gyroscope value x
        float gy1; // Unprocessed gyroscope value y
        float gz1; // Unprocessed gyroscope value z

        float ax2; // Unprocessed accelerometer value x
        float ay2; // Unprocessed accelerometer value y
        float az2; // Unprocessed accelerometer value z
        float gx2; // Unprocessed gyroscope value x
        float gy2; // Unprocessed gyroscope value y
        float gz2; // Unprocessed gyroscope value z

        float q0; // Quaternion for orientation of device
        float q1;
        float q2;
        float q3;

        float ax; // Acceleration [m s^-1] - Absolute
        float ay;
        float az;

        float vx; // Velocity [m s^-1] - Relative to starting (should be initially zero)
        float vy;
        float vz;

        float rx; // Position [m] - In mode 0, 1, and 2, is relative to starting point (not very useful)
        float ry; // In mode 3 (GNSS), this is instead a fixed-point of GNSS coordinates 
        float rz;

        int16_t grav_x; // Gravity vector component x (local frame - for reference only, not used in calculations)
        int16_t grav_y; // Gravity vector component y (local frame - for reference only, not used in calculations)
        int16_t grav_z; // Gravity vector component z (local frame - for reference only, not used in calculations)

        int8_t device_in_motion; // 1 if stationary, 2 if in motion
        int8_t label_2; // Reserved for future use

        int8_t temperature[3]; // Temperature of the IMUs, rounded to nearest int [deg C]
        int8_t c;              // Reserved for future use
        int32_t d;             // Reserved for future use
    } __attribute__((packed)) fs_imu_composite_packet;

    /// @brief
    typedef struct fs_imu_regular_packet
    {
        uint8_t type;  // Packet type (see typedef packet_type_t)
        uint32_t time; // Current time (in RTOS ticks = milliseconds)

        float omega_x0; // Angular velocity x - deg/s
        float omega_y0; // Angular velocity y - deg/s
        float omega_z0; // Angular velocity z - deg/s

        float q0; // Quaternion for orientation of device
        float q1;
        float q2;
        float q3;

        float ax; // Acceleration [m s^-1] - Absolute
        float ay;
        float az;

        float vx; // Velocity [m s^-1] - Relative to starting (should be initially zero)
        float vy;
        float vz;

        float rx; // Position [m] - In mode 0, 1, and 2, is relative to starting point
        float ry;
        float rz;

        int16_t grav_x; // Gravity vector component x (local frame)
        int16_t grav_y; // Gravity vector component y (local frame)
        int16_t grav_z; // Gravity vector component z (local frame)

        int8_t device_in_motion; // 1 if stationary, 2 if in motion
        int8_t label_2; // Reserved for future use

        int8_t temperature[3]; // Temperature of the IMUs, rounded to nearest int [deg C]
        int8_t c;              // Reserved for future use
        int32_t d;             // Reserved for future use
    } __attribute__((packed)) fs_imu_regular_packet;

    /// @brief
    typedef struct fs_imu_composite_packet_2
    {
        uint8_t type;  // Packet type (0 = telemetry only, 1 = orientation only, 2 = orientation and velocity, 3 = orientation and positioning, 4 = GPS)
        uint32_t time; // Current time (in RTOS ticks)

        float ax0; // Unprocessed accelerometer value x
        float ay0; // Unprocessed accelerometer value y
        float az0; // Unprocessed accelerometer value z
        float gx0; // Unprocessed gyroscope value x
        float gy0; // Unprocessed gyroscope value y
        float gz0; // Unprocessed gyroscope value z

        float ax1; // Unprocessed accelerometer value x
        float ay1; // Unprocessed accelerometer value y
        float az1; // Unprocessed accelerometer value z
        float gx1; // Unprocessed gyroscope value x
        float gy1; // Unprocessed gyroscope value y
        float gz1; // Unprocessed gyroscope value z

        float ax2; // Unprocessed accelerometer value x
        float ay2; // Unprocessed accelerometer value y
        float az2; // Unprocessed accelerometer value z
        float gx2; // Unprocessed gyroscope value x
        float gy2; // Unprocessed gyroscope value y
        float gz2; // Unprocessed gyroscope value z

        float q0; // Quaternion for orientation of device
        float q1;
        float q2;
        float q3;

        float wx; // Angular velocity [rad/s] - Body relative - This is filtered data derived from gyroscopes
        float wy;
        float wz;

        float ax; // Acceleration [m s^-1] - Absolute
        float ay;
        float az;

        float vx; // Velocity [m s^-1] - Relative to starting (should be initially zero)
        float vy;
        float vz;

        float rx; // Position [m] - In mode 0, 1, and 2, is relative to starting point
        float ry;
        float rz;

        int16_t grav_x; // Gravity vector component x (local frame - for reference only, not used in calculations)
        int16_t grav_y; // Gravity vector component y (local frame - for reference only, not used in calculations)
        int16_t grav_z; // Gravity vector component z (local frame - for reference only, not used in calculations)

        int8_t device_in_motion; // 1 if stationary, 2 if in motion
        int8_t label_2; // Reserved for future use

        int8_t temperature[3]; // Temperature of the IMUs, rounded to nearest int [deg C]
        int8_t c;              // Reserved for future use
        int32_t d;             // Reserved for future use
    } __attribute__((packed)) fs_imu_composite_packet_2;

    /// @brief Union allowing common storage of all packet types
    typedef union fs_packet_union
    {
        fs_imu_composite_packet composite;
        fs_imu_regular_packet regular;
        fs_imu_composite_packet_2 composite2;
    } __attribute__((packed)) fs_packet_union;

    /// @brief Communication mode between the device and the host (this machine)
    typedef enum fs_communication_mode
    {
        FS_COMMUNICATION_MODE_UNINITIALIZED = -1, // This is used when not initialized
        FS_COMMUNICATION_MODE_USB_CDC = 0,    // USB-CDC always on
        FS_COMMUNICATION_MODE_UART = 1,       // UART
        FS_COMMUNICATION_MODE_TCP_UDP = 2,    // UDP streaming (Station mode)
        FS_COMMUNICATION_MODE_TCP_UDP_AP = 4, // UDP streaming (Access point mode)
        FS_COMMUNICATION_MODE_CAN = 8,        // CAN bus - 
        FS_COMMUNICATION_MODE_I2C = 16,       // I2C bus - 
        FS_COMMUNICATION_MODE_ESPN = 32,      // ESP-NOW (2.4 GHz for ESP32 only) - 
        FS_COMMUNICATION_MODE_SPI = 64,       // SPI - 
        FS_COMMUNICATION_MODE_BLE = 128,      // Bluetooth Low Energy - 
    } fs_communication_mode;

    typedef enum fs_run_status
    {
        FS_RUN_STATUS_ERROR = -1,
        FS_RUN_STATUS_IDLE = 0,
        FS_RUN_STATUS_RUNNING = 1,
    } fs_run_status;

    typedef struct fs_driver_config
    {
        int background_task_priority;      // Priority of the background task for obtaining updates from the device (leave as -1 if no preference)
        int background_task_core_affinity; // Core to pin the background task to (leave as -1 if no preference)
        int read_timeout_micros;           // How long to wait (microseconds) to read data
        int task_wait_ms;                  // How long to wait in between updates of the background task
        int task_stack_size_bytes;         // How much to allocate to the background task
    } fs_driver_config;

#define FS_DRIVER_CONFIG_DEFAULT { \
    6,                             \
    1,                             \
    1000,                          \
    3,                             \
    16384,                         \
}

    /// @brief Device information container
    typedef struct fs_device_info
    {
        char device_name[32];                     // Unique identifier for device name
        int32_t device_id;                        // Unique identifier for the device type
        fs_communication_mode communication_mode; // Communication mode
        int32_t ping;                             // Time since last received communication from device

        fs_driver_config driver_config; // Device driver configuration (each device has its own thread spawned)
        fs_run_status status;           // 0 = UNINITIALIZED/STOPPED, 1 = RUNNING, -1 = ERROR

        char ip_addr[39]; // If networking is used, this is the corresponding IP address string
        int ip_port;      // Always 8888
        int tcp_sock;     // TCP socket number
        int udp_sock;     // UDP socket number

        int serial_port;  // If serial communication of some kind is used, this is the corresponding port
        int32_t baudrate; // If serial communication is used, baud rate. NOTE: USB-CDC ignores this completely

        fs_packet_union last_received_packet; // The last received packet for this device

        fs_packet_union packet_buf_queue[FS_MAX_PACKET_QUEUE_LENGTH]; // Packet queue buffer for the device (read-only)
        size_t packet_buf_queue_size;                                 // Current number of received packets (read-only)

        char command_queue[FS_MAX_CMD_QUEUE_LENGTH][FS_MAX_CMD_LENGTH]; // Command buffer for the device (read-only)
        size_t command_queue_size;                                      // Current number of received commands (read-only)

    } fs_device_info;

#define FS_DEVICE_INFO_UNINITIALIZED {                                                    \
    .device_name = {0},                                         /* device_name */          \
    .device_id = 0,                                             /* device_id */            \
    .communication_mode = FS_COMMUNICATION_MODE_UNINITIALIZED, /* communication_mode */   \
    .ping = 9999,                                               /* ping */                 \
    .driver_config = FS_DRIVER_CONFIG_DEFAULT,                  /* driver_config */        \
    .status = FS_RUN_STATUS_IDLE,                               /* status */               \
    .ip_addr = {0},                                             /* ip_addr */              \
    .ip_port = 8888,                                            /* ip_port */              \
    .tcp_sock = -1,                                             /* tcp_sock */             \
    .udp_sock = -1,                                             /* udp_sock */             \
    .serial_port = -1,                                          /* serial_port */          \
    .baudrate = 0,                                              /* baudrate */             \
    .last_received_packet = {{0}},                              /* last_received_packet */ \
    .packet_buf_queue = {{{0}}},                                /* packet_buf_queue */     \
    .packet_buf_queue_size = 0,                                 /* packet_buf_queue_size */\
    .command_queue = {{{{0}}}},                                 /* command_queue */        \
    .command_queue_size = 0                                     /* command_queue_size */   \
}

#ifdef __cplusplus
}
#endif

#endif
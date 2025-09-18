/// Driver for the Trifecta series of IMU/AHRS/INS devices
/// Copyright 2025 4rge.ai and/or Triangle Man LLC
/// Usage and redistribution of this code is permitted
/// but this notice must be retained in all copies of the code.

/// THIS SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
/// INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE,
/// AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
/// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#ifndef TRIFECTA_DEFS_PACKETS_H
#define TRIFECTA_DEFS_PACKETS_H

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>

#define FS_MAX_PACKET_QUEUE_LENGTH 4
#define FS_MAX_PACKET_LENGTH 256

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
    } fs_quaternion_t;

    typedef struct fs_vector3
    {
        float x;
        float y;
        float z;
    } fs_vector3_t;

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
    } fs_packet_type_t;

    /// @brief The Trifecta-K device output data format.
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

        int16_t reserved[3]; //

        int8_t device_in_motion; // 1 if stationary, 2 if in motion
        int8_t label_2;          // Reserved for future use

        int8_t temperature[3]; // Temperature of the IMUs, rounded to nearest int [deg C]
        int8_t c;              // Reserved for future use
        int32_t d;             // Reserved for future use
    } __attribute__((packed)) fs_imu_composite_packet_t;

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

        int16_t reserved[3]; //

        int8_t device_in_motion; // 1 if stationary, 2 if in motion
        int8_t label_2;          // Reserved for future use

        int8_t temperature[3]; // Temperature of the IMUs, rounded to nearest int [deg C]
        int8_t c;              // Reserved for future use
        int32_t d;             // Reserved for future use
    } __attribute__((packed)) fs_imu_regular_packet_t;

    /// @brief The Trifecta-M and Super-Trifecta device output data formats.
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

        int16_t reserved[3]; //

        int8_t device_in_motion; // 1 if stationary, 2 if in motion
        int8_t label_2;          // Reserved for future use

        int8_t temperature[3]; // Temperature of the IMUs, rounded to nearest int [deg C]
        int8_t c;              // Reserved for future use
        int32_t d;             // Reserved for future use
    } __attribute__((packed)) fs_imu_composite_packet_2_t;

    /// @brief Union allowing common storage of all packet types
    typedef union fs_packet_union
    {
        fs_imu_composite_packet_t composite;
        fs_imu_regular_packet_t regular;
        fs_imu_composite_packet_2_t composite2;
    } __attribute__((packed)) fs_packet_union_t;

#ifdef __cplusplus
}
#endif

#endif
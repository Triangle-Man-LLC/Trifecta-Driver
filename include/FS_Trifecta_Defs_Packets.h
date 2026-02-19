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

#define FS_MAX_PACKET_QUEUE_LENGTH 16
#define FS_MAX_PACKET_LENGTH 256

#ifdef _MSC_VER
#define FS_PACKED_BEGIN __pragma(pack(push, 1))
#define FS_PACKED_END __pragma(pack(pop))
#else
#define FS_PACKED_BEGIN
#define FS_PACKED_END __attribute__((packed))
#endif

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct fs_quaternion
    {
        float w; // Quaternion scalar term
        float x; // Quaternion vector term
        float y; // Quaternion vector term
        float z; // Quaternion vector term
    } fs_quaternion_t;

    typedef struct fs_vector3
    {
        float x;
        float y;
        float z;
    } fs_vector3_t;

    typedef struct fs_vector3_d
    {
        double x;
        double y;
        double z;
    } fs_vector3_d_t;

    typedef struct fs_vector3_i32
    {
        int32_t x;
        int32_t y;
        int32_t z;
    } fs_vector3_i32_t;

    typedef enum fs_gnss_position_format
    {
        FS_GNSS_PF_UNKNOWN = 0,
        FS_GNSS_PF_ECEF = 1,  // ECEF
        FS_GNSS_PF_WGS84 = 2, // Lat/long coordinate system
        FS_GNSS_PF_FIXED = 3, // Fixed point 32-bit
    } fs_gnss_position_format_t;

    /// @brief Type of packet indication
    typedef enum fs_packet_type
    {
        // Standard packet format, used by Trifecta-K0, K1, and K2.
        C_PACKET_TYPE_IMU = 0,      // IMU - raw acceleration and gyroscope measurements
        C_PACKET_TYPE_AHRS = 1,     // AHRS - IMU + orientation
        C_PACKET_TYPE_INS = 2,      // INS - GNSS/INS packets
        C_PACKET_TYPE_RESERVED = 3, //

        // Simplified packet format, currently reserved.
        S_PACKET_TYPE_IMU = 4,      // IMU - raw acceleration and gyroscope measurements
        S_PACKET_TYPE_AHRS = 5,     // AHRS - IMU + orientation
        S_PACKET_TYPE_INS = 6,      // INS - GNSS/INS packets
        S_PACKET_TYPE_RESERVED = 7, //

        // Long packet formats, used by Trifecta-M.
        C2_PACKET_TYPE_IMU = 8,       // IMU - raw acceleration and gyroscope measurements
        C2_PACKET_TYPE_AHRS = 9,      // AHRS - IMU + orientation
        C2_PACKET_TYPE_INS = 10,      // INS - GNSS/INS packets
        C2_PACKET_TYPE_RESERVED = 11, //

        // High-rate packet, comprised of raw data suitable for high-rate controls.
        HR_PACKET_TYPE_IMU = 21,        // IMU - raw acceleration and gyroscope measurements
        HR_PACKET_TYPE_RESERVED_1 = 22, //
        HR_PACKET_TYPE_RESERVED_2 = 23, //
        HR_PACKET_TYPE_RESERVED_3 = 24, //

        // Standard packet format, used by Trifecta-K0, K1, and K2. Version supporting 64-bit UTC timestamp.
        C64_PACKET_TYPE_IMU = 100,      // IMU - raw acceleration and gyroscope measurements
        C64_PACKET_TYPE_AHRS = 101,     // AHRS - IMU + orientation
        C64_PACKET_TYPE_INS = 102,      // INS - GNSS/INS packets
        C64_PACKET_TYPE_RESERVED = 103, //

        // Simplified packet format, currently reserved. Version supporting 64-bit UTC timestamp.
        S64_PACKET_TYPE_IMU = 104,      // IMU - raw acceleration and gyroscope measurements
        S64_PACKET_TYPE_AHRS = 105,     // AHRS - IMU + orientation
        S64_PACKET_TYPE_INS = 106,      // INS - GNSS/INS packets
        S64_PACKET_TYPE_RESERVED = 107, //

        // Long packet formats, used by Trifecta-M. Version supporting 64-bit UTC timestamp.
        C642_PACKET_TYPE_IMU = 108,      // IMU - raw acceleration and gyroscope measurements
        C642_PACKET_TYPE_AHRS = 109,     // AHRS - IMU + orientation
        C642_PACKET_TYPE_INS = 110,      // INS - GNSS/INS packets
        C642_PACKET_TYPE_RESERVED = 111, //

        // High-rate packet, comprised of raw data suitable for high-rate controls. Version supporting 64-bit UTC timestamp.
        HR64_PACKET_TYPE_IMU = 121,        // IMU - raw acceleration and gyroscope measurements
        HR64_PACKET_TYPE_RESERVED_1 = 122, //
        HR64_PACKET_TYPE_RESERVED_2 = 123, //
        HR64_PACKET_TYPE_RESERVED_3 = 124, //
    } fs_packet_type_t;

    /// @brief
    typedef enum fs_device_diagnostic_flags
    {
        FS_DEVICE_DIAG_UTC_SYNCED = 0x01,
        FS_DEVICE_DIAG_GNSS_AVAILABLE = 0x02,
        FS_DEVICE_DIAG_PPS_LOCKED = 0x04,
        FS_DEVICE_DIAG_IMU_HEALTHY = 0x08,
        FS_DEVICE_DIAG_RESERVED_4 = 0x10,
        FS_DEVICE_DIAG_RESERVED_5 = 0x20,
        FS_DEVICE_DIAG_RESERVED_6 = 0x40,
        FS_DEVICE_DIAG_RESERVED_7 = 0x80,
    } fs_device_diagnostic_flags_t;

    FS_PACKED_BEGIN
    struct fs_imu_composite_packet
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

        float q0; // Quaternion orientation
        float q1;
        float q2;
        float q3;

        float mag_x; // Magnetometer [mG]
        float mag_y;
        float mag_z;

        float acc_x; // Compensated acceleration [m s^-2]
        float acc_y;
        float acc_z;

        float omega_x0; // Compensated angular velocity [deg/s]
        float omega_y0;
        float omega_z0;

        int16_t temperature[3]; // Temperature of the IMUs, scaled according to the RESCALE_TEMP_C_TO_INT16() macro

        int8_t device_motion_status; // == 1 if stationary, 2 if in motion, 0 if no status
        uint8_t diagnostic_flag;     // Bitflag of fs_device_diagnostic_flags_t showing the status

        int8_t reserved[3];        //
        int8_t c;                  // Reserved for future use
        float barometric_pressure; // Barometric pressure data [mbar]
    } FS_PACKED_END;
    typedef struct fs_imu_composite_packet fs_imu_composite_packet_t;
    _Static_assert(sizeof(fs_imu_composite_packet_t) == 145, "fs_imu_composite_packet_t size mismatch");

    FS_PACKED_BEGIN
    struct fs_imu_regular_packet
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

        float mag_x; // Magnetometer [mG]
        float mag_y;
        float mag_z;

        float acc_x; // Compensated acceleration [m s^-2]
        float acc_y;
        float acc_z;

        float reserved_0_1; // Reserved for future use
        float reserved_0_2; //
        float reserved_0_3; //

        int16_t temperature[3]; // Temperature of the IMUs, scaled according to the RESCALE_TEMP_C_TO_INT16() macro

        int8_t device_motion_status; // == 1 if stationary, 2 if in motion, 0 if no status
        uint8_t diagnostic_flag;     // Bitflag of fs_device_diagnostic_flags_t showing the status

        int8_t reserved[3];        //
        int8_t c;                  // Reserved for future use
        float barometric_pressure; // Barometric pressure data [mbar]
    } FS_PACKED_END;
    typedef struct fs_imu_regular_packet fs_imu_regular_packet_t;
    _Static_assert(sizeof(fs_imu_regular_packet_t) == 85, "fs_imu_regular_packet_t size mismatch");

    FS_PACKED_BEGIN
    struct fs_imu_composite_packet_2
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

        float mag_x; // Magnetometer [mG]
        float mag_y;
        float mag_z;

        float omega_x0; // Compensated angular velocity [deg/s]
        float omega_y0;
        float omega_z0;

        float acc_x; // Compensated acceleration [m s^-2]
        float acc_y;
        float acc_z;

        float vx; // Velocity [m s^-1] ENU frame
        float vy; //
        float vz; //

        double rx; // Position (deg latitude)
        double ry; // Position (deg longitude)
        double rz; // Height (m)

        int16_t temperature[3]; // Temperature of the IMUs, scaled according to the RESCALE_TEMP_C_TO_INT16() macro

        int8_t device_motion_status; // == 1 if stationary, 2 if in motion, 0 if no status
        uint8_t diagnostic_flag;     // Bitflag of fs_device_diagnostic_flags_t showing the status

        int8_t reserved[3];        //
        int8_t c;                  // Reserved for future use
        float barometric_pressure; // Barometric pressure data [mbar]
    } FS_PACKED_END;
    typedef struct fs_imu_composite_packet_2 fs_imu_composite_packet_2_t;
    _Static_assert(sizeof(fs_imu_composite_packet_2_t) == 181, "fs_imu_composite_packet_2_t size mismatch");

    FS_PACKED_BEGIN
    struct fs_imu_composite_packet_64
    {
        uint8_t type;  // Packet type (0 = telemetry only, 1 = orientation only, 2 = orientation and velocity, 3 = orientation and positioning, 4 = GPS)
        uint64_t time; // 64-bit timefield, capable of holding full UTC time

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

        float q0; // Quaternion orientation
        float q1;
        float q2;
        float q3;

        float mag_x; // Magnetometer [mG]
        float mag_y;
        float mag_z;

        float acc_x; // Compensated acceleration [m s^-2]
        float acc_y;
        float acc_z;

        float omega_x0; // Compensated angular velocity [deg/s]
        float omega_y0;
        float omega_z0;

        int16_t temperature[3]; // Temperature of the IMUs, scaled according to the RESCALE_TEMP_C_TO_INT16() macro

        int8_t device_motion_status; // == 1 if stationary, 2 if in motion, 0 if no status
        uint8_t diagnostic_flag;     // Bitflag of fs_device_diagnostic_flags_t showing the status

        int8_t reserved[3];        //
        int8_t c;                  // Reserved for future use
        float barometric_pressure; // Barometric pressure data [mbar]
    } FS_PACKED_END;
    typedef struct fs_imu_composite_packet_64 fs_imu_composite_packet_64_t;
    _Static_assert(sizeof(fs_imu_composite_packet_64_t) == 149, "fs_imu_composite_packet_t size mismatch");

    FS_PACKED_BEGIN
    struct fs_imu_regular_packet_64
    {
        uint8_t type;  // Packet type (see typedef packet_type_t)
        uint64_t time; // 64-bit timefield, capable of holding full UTC time

        float omega_x0; // Angular velocity x - deg/s
        float omega_y0; // Angular velocity y - deg/s
        float omega_z0; // Angular velocity z - deg/s

        float q0; // Quaternion for orientation of device
        float q1;
        float q2;
        float q3;

        float mag_x; // Magnetometer [mG]
        float mag_y;
        float mag_z;

        float acc_x; // Compensated acceleration [m s^-2]
        float acc_y;
        float acc_z;

        float reserved_0_1; // Reserved for future use
        float reserved_0_2; //
        float reserved_0_3; //

        int16_t temperature[3]; // Temperature of the IMUs, scaled according to the RESCALE_TEMP_C_TO_INT16() macro

        int8_t device_motion_status; // == 1 if stationary, 2 if in motion, 0 if no status
        uint8_t diagnostic_flag;     // Bitflag of fs_device_diagnostic_flags_t showing the status

        int8_t reserved[3];        //
        int8_t c;                  // Reserved for future use
        float barometric_pressure; // Barometric pressure data [mbar]
    } FS_PACKED_END;
    typedef struct fs_imu_regular_packet_64 fs_imu_regular_packet_64_t;
    _Static_assert(sizeof(fs_imu_regular_packet_64_t) == 89, "fs_imu_regular_packet_t size mismatch");

    FS_PACKED_BEGIN
    struct fs_imu_composite_packet_64_2
    {
        uint8_t type;  // Packet type (0 = telemetry only, 1 = orientation only, 2 = orientation and velocity, 3 = orientation and positioning, 4 = GPS)
        uint64_t time; // 64-bit timefield, capable of holding full UTC time

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

        float mag_x; // Magnetometer [mG]
        float mag_y;
        float mag_z;

        float omega_x0; // Compensated angular velocity [deg/s]
        float omega_y0;
        float omega_z0;

        float acc_x; // Compensated acceleration [m s^-2]
        float acc_y;
        float acc_z;

        float vx; // Velocity [m s^-1] ENU frame
        float vy; //
        float vz; //

        double rx; // Position (deg latitude)
        double ry; // Position (deg longitude)
        double rz; // Height (m)

        int16_t temperature[3]; // Temperature of the IMUs, scaled according to the RESCALE_TEMP_C_TO_INT16() macro

        int8_t device_motion_status; // == 1 if stationary, 2 if in motion, 0 if no status
        uint8_t diagnostic_flag;     // Bitflag of fs_device_diagnostic_flags_t showing the status

        int8_t reserved[3];        //
        int8_t c;                  // Reserved for future use
        float barometric_pressure; // Barometric pressure data [mbar]
    } FS_PACKED_END;
    typedef struct fs_imu_composite_packet_64_2 fs_imu_composite_packet_64_2_t;
    _Static_assert(sizeof(fs_imu_composite_packet_64_2_t) == 185, "fs_imu_composite_packet_2_t size mismatch");

    FS_PACKED_BEGIN
    union fs_packet_union
    {
        fs_imu_composite_packet_t composite;
        fs_imu_regular_packet_t regular;
        fs_imu_composite_packet_2_t composite2;
        fs_imu_composite_packet_64_t composite64;
        fs_imu_regular_packet_64_t regular64;
        fs_imu_composite_packet_64_2_t composite64_2;
    } FS_PACKED_END;
    typedef union fs_packet_union fs_packet_union_t;
    _Static_assert(sizeof(fs_packet_union_t) == 185, "fs_packet_union_t size mismatch");
#ifdef __cplusplus
}
#endif

#endif
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
#define FS_PACKED_STRUCT(name) __pragma(pack(push, 1)) struct name __pragma(pack(pop))
#define FS_PACKED_UNION(name) __pragma(pack(push, 1)) union name __pragma(pack(pop))
#else
#define FS_PACKED_STRUCT(name) struct __attribute__((packed)) name
#define FS_PACKED_UNION(name) union __attribute__((packed)) name
#endif

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

    FS_PACKED_STRUCT(fs_imu_composite_packet)
    {
        uint8_t type;
        uint32_t time;

        float ax0, ay0, az0, gx0, gy0, gz0;
        float ax1, ay1, az1, gx1, gy1, gz1;
        float ax2, ay2, az2, gx2, gy2, gz2;

        float q0, q1, q2, q3;

        float ax, ay, az;
        float vx, vy, vz;
        float rx, ry, rz;

        int16_t reserved[3];
        int8_t device_in_motion;
        int8_t label_2;
        int8_t temperature[3];
        int8_t c;
        int32_t d;
    };
    typedef struct fs_imu_composite_packet fs_imu_composite_packet_t;

    FS_PACKED_STRUCT(fs_imu_regular_packet)
    {
        uint8_t type;
        uint32_t time;

        float omega_x0, omega_y0, omega_z0;
        float q0, q1, q2, q3;
        float ax, ay, az;
        float vx, vy, vz;
        float rx, ry, rz;

        int16_t reserved[3];
        int8_t device_in_motion;
        int8_t label_2;
        int8_t temperature[3];
        int8_t c;
        int32_t d;
    };
    typedef struct fs_imu_regular_packet fs_imu_regular_packet_t;

    FS_PACKED_STRUCT(fs_imu_composite_packet_2)
    {
        uint8_t type;
        uint32_t time;

        float ax0, ay0, az0, gx0, gy0, gz0;
        float ax1, ay1, az1, gx1, gy1, gz1;
        float ax2, ay2, az2, gx2, gy2, gz2;

        float q0, q1, q2, q3;
        float wx, wy, wz;
        float ax, ay, az;
        float vx, vy, vz;
        float rx, ry, rz;

        int16_t reserved[3];
        int8_t device_in_motion;
        int8_t label_2;
        int8_t temperature[3];
        int8_t c;
        int32_t d;
    };
    typedef struct fs_imu_composite_packet_2 fs_imu_composite_packet_2_t;

    FS_PACKED_UNION(fs_packet_union)
    {
        fs_imu_composite_packet_t composite;
        fs_imu_regular_packet_t regular;
        fs_imu_composite_packet_2_t composite2;
    };
    typedef union fs_packet_union fs_packet_union_t;

#ifdef __cplusplus
}
#endif

#endif
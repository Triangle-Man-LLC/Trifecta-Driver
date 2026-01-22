/// Driver for the Trifecta series of IMU/AHRS/INS devices
/// Copyright 2025 4rge.ai and/or Triangle Man LLC
/// Usage and redistribution of this code is permitted
/// but this notice must be retained in all copies of the code.

/// THIS SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
/// INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE,
/// AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
/// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include "FS_Trifecta_Defs.h"
#include "FS_Trifecta_Device.h"

/// @brief Retrieve an euler angles from a packet (given that packets by default report quaternions)
/// @param packet The packet.
/// @param euler_angles_out Output buffer for Euler angles (deg).
/// @return 0 on success.
int fs_euler_angles_from_packet(const fs_packet_union_t *packet, fs_vector3_t *euler_angles_out)
{
    if (!packet || !euler_angles_out)
        return -1;
    switch (packet->composite.type)
    {
    case C_PACKET_TYPE_IMU:
    case C_PACKET_TYPE_AHRS:
    case C_PACKET_TYPE_RESERVED:
    case C_PACKET_TYPE_INS:
    {
        fs_q_to_euler_angles(&(euler_angles_out->x), &(euler_angles_out->y), &(euler_angles_out->z),
                             packet->composite.q0, packet->composite.q1, packet->composite.q2, packet->composite.q2, true);
        break;
    }
    case S_PACKET_TYPE_IMU:
    case S_PACKET_TYPE_AHRS:
    case S_PACKET_TYPE_RESERVED:
    case S_PACKET_TYPE_INS:
    {
        fs_q_to_euler_angles(&(euler_angles_out->x), &(euler_angles_out->y), &(euler_angles_out->z),
                             packet->regular.q0, packet->regular.q1, packet->regular.q2, packet->regular.q2, true);
        break;
    }
    case C2_PACKET_TYPE_IMU:
    case C2_PACKET_TYPE_AHRS:
    case C2_PACKET_TYPE_RESERVED:
    case C2_PACKET_TYPE_INS:
    {
        fs_q_to_euler_angles(&(euler_angles_out->x), &(euler_angles_out->y), &(euler_angles_out->z),
                             packet->composite2.q0, packet->composite2.q1, packet->composite2.q2, packet->composite2.q2, true);
        break;
    }
    default:
        return -1;
    }
    return 0;
}

/// @brief Retrieve the latitude, longitude, and height from the packet.
/// @param packet The packet.
/// @param lat_long_height Output buffer for a fs_vector3_d_t containing .x == LATITUDE, .y == LONGITUDE, .z == HEIGHT (m)
/// @return 0 on success.
int fs_lat_long_from_packet(const fs_packet_union_t *packet, fs_vector3_d_t *lat_long_height)
{
    if (!packet || !lat_long_height)
        return -1;
    switch (packet->composite.type)
    {
    case C_PACKET_TYPE_IMU:
    case C_PACKET_TYPE_AHRS:
    case C_PACKET_TYPE_RESERVED:
    case C_PACKET_TYPE_INS:
    {
        return -1; // IMU-only packets do not contain GNSS information.
    }
    case S_PACKET_TYPE_IMU:
    case S_PACKET_TYPE_AHRS:
    case S_PACKET_TYPE_RESERVED:
    case S_PACKET_TYPE_INS:
    {
        return -1; // IMU-only packets do not contain GNSS information.
    }
    case C2_PACKET_TYPE_IMU:
    case C2_PACKET_TYPE_AHRS:
    case C2_PACKET_TYPE_RESERVED:
    case C2_PACKET_TYPE_INS:
    {
        lat_long_height->x = packet->composite2.rx; // Latitude [deg]
        lat_long_height->y = packet->composite2.ry; // Longitude [deg]
        lat_long_height->z = packet->composite2.rz; // Height [m]
        break;
    }
    default:
        return -1;
    }
    return 0;
}

/// @brief Retrieve the angular velocity (deg/s) from the packet.
/// @param packet The packet.
/// @param angular_velocity Output buffer for the angular velocity, in deg/s, with axes in the sensor body frame.
/// @return 0 on success.
int fs_angular_velocity_from_packet(const fs_packet_union_t *packet, fs_vector3_t *angular_velocity)
{
    if (!packet || !angular_velocity)
        return -1;
    switch (packet->composite.type)
    {
    case C_PACKET_TYPE_IMU:
    case C_PACKET_TYPE_AHRS:
    case C_PACKET_TYPE_RESERVED:
    case C_PACKET_TYPE_INS:
    {
        angular_velocity->x = packet->composite.omega_x0;
        angular_velocity->y = packet->composite.omega_y0;
        angular_velocity->z = packet->composite.omega_z0;
        break;
    }
    case S_PACKET_TYPE_IMU:
    case S_PACKET_TYPE_AHRS:
    case S_PACKET_TYPE_RESERVED:
    case S_PACKET_TYPE_INS:
    {
        angular_velocity->x = packet->regular.omega_x0;
        angular_velocity->y = packet->regular.omega_y0;
        angular_velocity->z = packet->regular.omega_z0;
        break;
    }
    case C2_PACKET_TYPE_IMU:
    case C2_PACKET_TYPE_AHRS:
    case C2_PACKET_TYPE_RESERVED:
    case C2_PACKET_TYPE_INS:
    {
        angular_velocity->x = packet->composite2.omega_x0;
        angular_velocity->y = packet->composite2.omega_y0;
        angular_velocity->z = packet->composite2.omega_z0;
        break;
    }
    default:
        return -1;
    }
    return 0;
}

int fs_angular_velocity_raw_from_packet(const fs_packet_union_t *packet, fs_vector3_t *angular_velocity)
{
    if (!packet || !angular_velocity)
        return -1;
    switch (packet->composite.type)
    {
    case C_PACKET_TYPE_IMU:
    case C_PACKET_TYPE_AHRS:
    case C_PACKET_TYPE_RESERVED:
    case C_PACKET_TYPE_INS:
    {
        angular_velocity->x = (packet->composite.gx0 + packet->composite.gx1 + packet->composite.gx2) * FS_GYRO_SCALER_DPS / (3.0f * (float)INT16_MAX);
        angular_velocity->y = (packet->composite.gy0 + packet->composite.gy1 + packet->composite.gy2) * FS_GYRO_SCALER_DPS / (3.0f * (float)INT16_MAX);
        angular_velocity->z = (packet->composite.gz0 + packet->composite.gz1 + packet->composite.gz2) * FS_GYRO_SCALER_DPS / (3.0f * (float)INT16_MAX);
        break;
    }
    case S_PACKET_TYPE_IMU:
    case S_PACKET_TYPE_AHRS:
    case S_PACKET_TYPE_RESERVED:
    case S_PACKET_TYPE_INS:
    {
        angular_velocity->x = (packet->regular.gx0 + packet->regular.gx1 + packet->regular.gx2) * FS_GYRO_SCALER_DPS / (3.0f * (float)INT16_MAX);
        angular_velocity->y = (packet->regular.gy0 + packet->regular.gy1 + packet->regular.gy2) * FS_GYRO_SCALER_DPS / (3.0f * (float)INT16_MAX);
        angular_velocity->z = (packet->regular.gz0 + packet->regular.gz1 + packet->regular.gz2) * FS_GYRO_SCALER_DPS / (3.0f * (float)INT16_MAX);
        break;
    }
    case C2_PACKET_TYPE_IMU:
    case C2_PACKET_TYPE_AHRS:
    case C2_PACKET_TYPE_RESERVED:
    case C2_PACKET_TYPE_INS:
    {
        angular_velocity->x = (packet->composite2.gx0 + packet->composite2.gx1 + packet->composite2.gx2) * FS_GYRO_SCALER_DPS / (3.0f * (float)INT16_MAX);
        angular_velocity->y = (packet->composite2.gy0 + packet->composite2.gy1 + packet->composite2.gy2) * FS_GYRO_SCALER_DPS / (3.0f * (float)INT16_MAX);
        angular_velocity->z = (packet->composite2.gz0 + packet->composite2.gz1 + packet->composite2.gz2) * FS_GYRO_SCALER_DPS / (3.0f * (float)INT16_MAX);
        break;
    }
    default:
        return -1;
    }
    return 0;
}

/// @brief Retrieve the acceleration (m/s^2) from the packet.
/// @param packet The packet.
/// @param angular_velocity Output buffer for the acceleration (m/s^2), with axes in the sensor body frame.
/// @return 0 on success.
int fs_acceleration_from_packet(const fs_packet_union_t *packet, fs_vector3_t *acceleration)
{
    if (!packet || !acceleration)
        return -1;
    switch (packet->composite.type)
    {
    case C_PACKET_TYPE_IMU:
    case C_PACKET_TYPE_AHRS:
    case C_PACKET_TYPE_RESERVED:
    case C_PACKET_TYPE_INS:
    {
        acceleration->x = packet->composite.acc_x * FS_ACCEL_SCALER_Gs / ((float)INT16_MAX);
        acceleration->y = packet->composite.acc_y * FS_ACCEL_SCALER_Gs / ((float)INT16_MAX);
        acceleration->z = packet->composite.acc_z * FS_ACCEL_SCALER_Gs / ((float)INT16_MAX);
        break;
    }
    case S_PACKET_TYPE_IMU:
    case S_PACKET_TYPE_AHRS:
    case S_PACKET_TYPE_RESERVED:
    case S_PACKET_TYPE_INS:
    {
        acceleration->x = packet->regular.acc_x * FS_ACCEL_SCALER_Gs / ((float)INT16_MAX);
        acceleration->y = packet->regular.acc_y * FS_ACCEL_SCALER_Gs / ((float)INT16_MAX);
        acceleration->z = packet->regular.acc_z * FS_ACCEL_SCALER_Gs / ((float)INT16_MAX);

        break;
    }
    case C2_PACKET_TYPE_IMU:
    case C2_PACKET_TYPE_AHRS:
    case C2_PACKET_TYPE_RESERVED:
    case C2_PACKET_TYPE_INS:
    {
        acceleration->x = packet->composite.acc_x * FS_ACCEL_SCALER_Gs / ((float)INT16_MAX);
        acceleration->y = packet->composite.acc_y * FS_ACCEL_SCALER_Gs / ((float)INT16_MAX);
        acceleration->z = packet->composite.acc_z * FS_ACCEL_SCALER_Gs / ((float)INT16_MAX);
        break;
    }
    default:
        return -1;
    }
    return 0;
}

int fs_acceleration_raw_from_packet(const fs_packet_union_t *packet, fs_vector3_t *acceleration)
{
    if (!packet || !angular_velocity)
        return -1;
    switch (packet->composite.type)
    {
    case C_PACKET_TYPE_IMU:
    case C_PACKET_TYPE_AHRS:
    case C_PACKET_TYPE_RESERVED:
    case C_PACKET_TYPE_INS:
    {
        angular_velocity->x = (packet->composite.ax0 + packet->composite.ax1 + packet->composite.ax2) * FS_ACCEL_SCALER_Gs / (3.0f * (float)INT16_MAX);
        angular_velocity->y = (packet->composite.ay0 + packet->composite.ay1 + packet->composite.ay2) * FS_ACCEL_SCALER_Gs / (3.0f * (float)INT16_MAX);
        angular_velocity->z = (packet->composite.az0 + packet->composite.az1 + packet->composite.az2) * FS_ACCEL_SCALER_Gs / (3.0f * (float)INT16_MAX);
        break;
    }
    case S_PACKET_TYPE_IMU:
    case S_PACKET_TYPE_AHRS:
    case S_PACKET_TYPE_RESERVED:
    case S_PACKET_TYPE_INS:
    {
        angular_velocity->x = (packet->regular.ax0 + packet->regular.ax1 + packet->regular.ax2) * FS_ACCEL_SCALER_Gs / (3.0f * (float)INT16_MAX);
        angular_velocity->y = (packet->regular.ay0 + packet->regular.ay1 + packet->regular.ay2) * FS_ACCEL_SCALER_Gs / (3.0f * (float)INT16_MAX);
        angular_velocity->z = (packet->regular.az0 + packet->regular.az1 + packet->regular.az2) * FS_ACCEL_SCALER_Gs / (3.0f * (float)INT16_MAX);
        break;
    }
    case C2_PACKET_TYPE_IMU:
    case C2_PACKET_TYPE_AHRS:
    case C2_PACKET_TYPE_RESERVED:
    case C2_PACKET_TYPE_INS:
    {
        angular_velocity->x = (packet->composite2.ax0 + packet->composite2.ax1 + packet->composite2.ax2) * FS_ACCEL_SCALER_Gs / (3.0f * (float)INT16_MAX);
        angular_velocity->y = (packet->composite2.ay0 + packet->composite2.ay1 + packet->composite2.ay2) * FS_ACCEL_SCALER_Gs / (3.0f * (float)INT16_MAX);
        angular_velocity->z = (packet->composite2.az0 + packet->composite2.az1 + packet->composite2.az2) * FS_ACCEL_SCALER_Gs / (3.0f * (float)INT16_MAX);
        break;
    }
    default:
        return -1;
    }
    return 0;
}

/// @brief Retrieve the magnetic field (mG) from the packet.
/// @param packet The packet.
/// @param mag_values Output buffer for the magnetic field (mG) with axes in the sensor body frame.
/// @return 0 on success.
int fs_magnetic_field_from_packet(const fs_packet_union_t *packet, fs_vector3_t *mag_values)
{
    if (!packet || !mag_values)
        return -1;
    switch (packet->composite.type)
    {
    case C_PACKET_TYPE_IMU:
    case C_PACKET_TYPE_AHRS:
    case C_PACKET_TYPE_RESERVED:
    case C_PACKET_TYPE_INS:
    {
        mag_values->x = packet->composite.mag_x;
        mag_values->y = packet->composite.mag_y;
        mag_values->z = packet->composite.mag_z;
        break;
    }
    case S_PACKET_TYPE_IMU:
    case S_PACKET_TYPE_AHRS:
    case S_PACKET_TYPE_RESERVED:
    case S_PACKET_TYPE_INS:
    {
        mag_values->x = packet->regular.mag_x;
        mag_values->y = packet->regular.mag_y;
        mag_values->z = packet->regular.mag_z;
        break;
    }
    case C2_PACKET_TYPE_IMU:
    case C2_PACKET_TYPE_AHRS:
    case C2_PACKET_TYPE_RESERVED:
    case C2_PACKET_TYPE_INS:
    {
        mag_values->x = packet->composite2.mag_x;
        mag_values->y = packet->composite2.mag_y;
        mag_values->z = packet->composite2.mag_z;
        break;
    }
    default:
        return -1;
    }
    return 0;
}

/// @brief Retrieve the barometric pressure (Pa) from the packet.
/// @param packet The packet.
/// @param pres_value Output buffer for the pressure, in Pa.
/// @return 0 on success.
int fs_barometric_pressure_from_packet(const fs_packet_union_t *packet, float *pres_value)
{
    if (!packet || !pres_value)
        return -1;
    switch (packet->composite.type)
    {
    case C_PACKET_TYPE_IMU:
    case C_PACKET_TYPE_AHRS:
    case C_PACKET_TYPE_RESERVED:
    case C_PACKET_TYPE_INS:
    {
        *pres_value = packet->composite.barometric_pressure;
        break;
    }
    case S_PACKET_TYPE_IMU:
    case S_PACKET_TYPE_AHRS:
    case S_PACKET_TYPE_RESERVED:
    case S_PACKET_TYPE_INS:
    {
        *pres_value = packet->regular.barometric_pressure;
        break;
    }
    case C2_PACKET_TYPE_IMU:
    case C2_PACKET_TYPE_AHRS:
    case C2_PACKET_TYPE_RESERVED:
    case C2_PACKET_TYPE_INS:
    {
        *pres_value = packet->composite2.barometric_pressure;
        break;
    }
    default:
        return -1;
    }
    return 0;
}

/// @brief Retrieve velocity (m/s) from the packet. This is only applicable to Trifecta-M devices.
/// @param packet The packet.
/// @param velocity Output buffer for the velocity (m/s).
/// @return 0 on success.
int fs_velocity_from_packet(const fs_packet_union_t *packet, fs_vector3_t *velocity)
{
    if (!packet || !velocity)
        return -1;
    switch (packet->composite.type)
    {
    case C_PACKET_TYPE_IMU:
    case C_PACKET_TYPE_AHRS:
    case C_PACKET_TYPE_RESERVED:
    case C_PACKET_TYPE_INS:
    {
        return -1; // IMU-only packets do not contain velocity vector information.
    }
    case S_PACKET_TYPE_IMU:
    case S_PACKET_TYPE_AHRS:
    case S_PACKET_TYPE_RESERVED:
    case S_PACKET_TYPE_INS:
    {
        return -1; // IMU-only packets do not contain velocity vector information.
    }
    case C2_PACKET_TYPE_IMU:
    case C2_PACKET_TYPE_AHRS:
    case C2_PACKET_TYPE_RESERVED:
    case C2_PACKET_TYPE_INS:
    {
        velocity->x = packet->composite2.vx;
        velocity->y = packet->composite2.vy;
        velocity->z = packet->composite2.vz;
        break;
    }
    default:
        return -1;
    }
    return 0;
}
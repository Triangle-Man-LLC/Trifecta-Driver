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
    case C_PACKET_TYPE_GNSS:
    case C_PACKET_TYPE_INS:
    {
        fs_q_to_euler_angles(&(euler_angles_out->x), &(euler_angles_out->y), &(euler_angles_out->z),
                             packet->composite.q0, packet->composite.q1, packet->composite.q2, packet->composite.q2, true);
        break;
    }
    case S_PACKET_TYPE_IMU:
    case S_PACKET_TYPE_AHRS:
    case S_PACKET_TYPE_GNSS:
    case S_PACKET_TYPE_INS:
    {
        fs_q_to_euler_angles(&(euler_angles_out->x), &(euler_angles_out->y), &(euler_angles_out->z),
                             packet->regular.q0, packet->regular.q1, packet->regular.q2, packet->regular.q2, true);
        break;
    }
    case C2_PACKET_TYPE_IMU:
    case C2_PACKET_TYPE_AHRS:
    case C2_PACKET_TYPE_GNSS:
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
    case C_PACKET_TYPE_GNSS:
    case C_PACKET_TYPE_INS:
    {

        break;
    }
    case S_PACKET_TYPE_IMU:
    case S_PACKET_TYPE_AHRS:
    case S_PACKET_TYPE_GNSS:
    case S_PACKET_TYPE_INS:
    {

        break;
    }
    case C2_PACKET_TYPE_IMU:
    case C2_PACKET_TYPE_AHRS:
    case C2_PACKET_TYPE_GNSS:
    case C2_PACKET_TYPE_INS:
    {

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
    case C_PACKET_TYPE_GNSS:
    case C_PACKET_TYPE_INS:
    {
        
        break;
    }
    case S_PACKET_TYPE_IMU:
    case S_PACKET_TYPE_AHRS:
    case S_PACKET_TYPE_GNSS:
    case S_PACKET_TYPE_INS:
    {

        break;
    }
    case C2_PACKET_TYPE_IMU:
    case C2_PACKET_TYPE_AHRS:
    case C2_PACKET_TYPE_GNSS:
    case C2_PACKET_TYPE_INS:
    {

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
    case C_PACKET_TYPE_GNSS:
    case C_PACKET_TYPE_INS:
    {

        break;
    }
    case S_PACKET_TYPE_IMU:
    case S_PACKET_TYPE_AHRS:
    case S_PACKET_TYPE_GNSS:
    case S_PACKET_TYPE_INS:
    {

        break;
    }
    case C2_PACKET_TYPE_IMU:
    case C2_PACKET_TYPE_AHRS:
    case C2_PACKET_TYPE_GNSS:
    case C2_PACKET_TYPE_INS:
    {

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
    case C_PACKET_TYPE_GNSS:
    case C_PACKET_TYPE_INS:
    {

        break;
    }
    case S_PACKET_TYPE_IMU:
    case S_PACKET_TYPE_AHRS:
    case S_PACKET_TYPE_GNSS:
    case S_PACKET_TYPE_INS:
    {

        break;
    }
    case C2_PACKET_TYPE_IMU:
    case C2_PACKET_TYPE_AHRS:
    case C2_PACKET_TYPE_GNSS:
    case C2_PACKET_TYPE_INS:
    {

        break;
    }
    default:
        return -1;
    }
    return 0;
}
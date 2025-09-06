/// Driver for the Trifecta series of IMU/AHRS/INS devices
/// Copyright 2024 4rge.ai and/or Triangle Man LLC
/// Usage and redistribution of this code is permitted
/// but this notice must be retained in all copies of the code.

/// THIS SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
/// INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE,
/// AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
/// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include "FS_Trifecta_Device.h"
#include "FS_Trifecta_Device_Utils.h"

/// @brief Handle all potential received commands from the buffer
/// @param device_handle Device handle
/// @param cmd_buf The buffer containing data blob to process
/// @param buf_len Size of cmd_buf
/// @return 0 on success, -1 on failure
int fs_handle_received_commands(fs_device_info_t *device_handle, const void *cmd_buf, size_t buf_len)
{
    if (fs_segment_commands(device_handle, cmd_buf, buf_len) != 0)
    {
        fs_log_output("[Trifecta-Device] Command parsing failed!");
        return -1;
    }

    char *command = NULL;

    for (int cq_index = 0; cq_index < device_handle->command_queue_size; cq_index++)
    {
        command = device_handle->command_queue[cq_index];
        size_t command_length = strnlen(command, FS_MAX_CMD_LENGTH);

        fs_log_output("[Trifecta] Command (len %ld): %c Params: %s", command_length, command[0], command + 1);

        if (command_length > 0 && command[0] == CMD_IDENTIFY)
        {
            size_t name_length = command_length - 1; // Exclude CMD_IDENTIFY and ';'
            strncpy(device_handle->device_name, command + 1, name_length);
            device_handle->device_name[name_length] = '\0';
        }
    }

    device_handle->command_queue_size = 0;
    return 0;
}

static int fs_device_process_packets_serial(fs_device_info_t *device_handle, const void *rx_buf, size_t rx_len)
{
    fs_log_output("[Trifecta] RX: Pushing %zu bytes into circular buffer", rx_len);
    fs_cb_push(&device_handle->data_buffer, (const uint8_t *)rx_buf, rx_len);

    uint8_t temp[FS_MAX_DATA_LENGTH];
    while (1)
    {
        size_t peeked = fs_cb_peek(&device_handle->data_buffer, temp, sizeof(temp));
        if (peeked == 0)
        {
            fs_log_output("[Trifecta] Buffer empty after peek, exiting loop");
            break;
        }

        fs_log_output("[Trifecta] Peeked %zu bytes from buffer", peeked);
        fs_log_output("[Trifecta] Peeked data: %.*s", (int)peeked, temp);

        fs_delimiter_indices_t indices = fs_scan_delimiters(temp, peeked);
        fs_log_output("[Trifecta] Delimiter scan: colon=%d, exclam=%d, semicolon=%d",
                      indices.colon_index, indices.exclam_index, indices.semicolon_index);

        if (indices.colon_index != -1 && indices.exclam_index > indices.colon_index)
        {
            size_t payload_start = indices.colon_index + 1;
            size_t payload_len = indices.exclam_index - payload_start;

            fs_log_output("[Trifecta] Detected packet framing from %d to %d (payload length %zu)",
                          indices.colon_index, indices.exclam_index, payload_len);

            fs_log_output("[Trifecta] Attempting to decode packet payload: %.*s",
                          (int)payload_len, &temp[payload_start]);

            if (base64_to_packet(device_handle, (char *)&temp[payload_start], payload_len) != 0)
            {
                fs_log_output("[Trifecta] Failed to decode packet");
            }

            size_t packet_len = indices.exclam_index + 1;
            fs_log_output("[Trifecta] Discarding %zu bytes from buffer after packet", packet_len);
            fs_cb_pop(&device_handle->data_buffer, NULL, packet_len);
        }
        else if (indices.semicolon_index != -1)
        {
            size_t cmd_len = indices.semicolon_index + 1;
            fs_log_output("[Trifecta] Detected command ending at index %d (length %zu)",
                          indices.semicolon_index, cmd_len);

            fs_log_output("[Trifecta] Attempting to segment commands...");
            if (fs_segment_commands(device_handle, temp, peeked) != 0)
                fs_log_output("[Trifecta] Failed to segment commands");

            fs_log_output("[Trifecta] Discarding %zu bytes from buffer after command", cmd_len);
            fs_cb_pop(&device_handle->data_buffer, NULL, cmd_len);
        }
        else
        {
            fs_log_output("[Trifecta] No complete segment found ... waiting for more data");
            return -1;
        }
    }

    return 0;
}

/// @brief Attempt to parse the packet(s) in the rx_buf
/// @param rx_buf The buffer to read from
/// @param rx_len The length to read
/// @param source FS_COMMUNICATION_MODE_SERIAL, FS_COMMUNICATION_MODE_TCP_UDP, etc. are handled differently
/// @return
int fs_device_parse_packet(fs_device_info_t *device_handle, const void *rx_buf, ssize_t rx_len, fs_communication_mode_t source)
{
    if (rx_len <= 0)
    {
        return -1;
    }

    for (int i = 0; i < FS_MAX_CMD_QUEUE_LENGTH; i++)
    {
        memset(&device_handle->command_queue[i], 0, FS_MAX_CMD_LENGTH);
    }

    for (int i = 0; i < FS_MAX_PACKET_QUEUE_LENGTH; i++)
    {
        memset(&device_handle->packet_buf_queue[i], 0, sizeof(fs_packet_union_t));
    }

    switch (source)
    {
    // UDP, I2C, and SPI modes all transfer binary data packets.
    case FS_COMMUNICATION_MODE_TCP_UDP:
    case FS_COMMUNICATION_MODE_I2C:
    case FS_COMMUNICATION_MODE_SPI: 
    {
        int packets = segment_packets(device_handle, rx_buf, rx_len);
        if (packets < 0)
        {
            fs_log_output("[Trifecta] Error parsing packets!");
            return -1;
        }
        fs_log_output("[Trifecta] Successfully processed packets (network format)!");
    }
    break;
    // UART and CDC modes all transfer Base64-encoded packets.
    case FS_COMMUNICATION_MODE_UART:
    case FS_COMMUNICATION_MODE_USB_CDC:
    {
        if (fs_device_process_packets_serial(device_handle, rx_buf, rx_len) != 0)
        {
            fs_log_output("[Trifecta] Error parsing packets!");
            return -1;
        }
        fs_log_output("[Trifecta] Successfully processed packets!");
    }
    break;
    // BLE/CAN driver support will be added in the future, for platforms which support them.
    case FS_COMMUNICATION_MODE_BLE:
    case FS_COMMUNICATION_MODE_CAN:
        fs_log_output("[Trifecta] These modes are not yet supported!");
        return -1;
    default:
        fs_log_output("[Trifecta] Unknown packet source!");
        return -1;
    }

    // Retrieve the last packet parsed
    if (device_handle->packet_buf_queue_size > 0)
    {
        size_t required_size = obtain_packet_length(device_handle->packet_buf_queue[device_handle->packet_buf_queue_size - 1].composite.type);

        // Erase the last received packet
        memset(&device_handle->last_received_packet, 0, sizeof(fs_packet_union_t));
        memcpy(&device_handle->last_received_packet, &device_handle->packet_buf_queue[device_handle->packet_buf_queue_size - 1], sizeof(fs_packet_union_t));
        device_handle->packet_buf_queue_size = 0; // Clean the queue

        fs_log_output("[Trifecta] Successful data parsing! Length %d Timestamp: %ld - Orientation: %0.4f %0.4f %0.4f %0.4f", required_size, device_handle->last_received_packet.composite.time, device_handle->last_received_packet.composite.q0, device_handle->last_received_packet.composite.q1, device_handle->last_received_packet.composite.q2, device_handle->last_received_packet.composite.q3);
        return 0;
    }

    return -1;
}

/// @brief Utility function to get Euler angles from quaternion
/// @param estRoll Pointer to buffer for storing roll value
/// @param estPitch Pointer to buffer for storing pitch value
/// @param estYaw Pointer to buffer for storing yaw value
/// @param q0 Quaternion component
/// @param q1 Quaternion component
/// @param q2 Quaternion component
/// @param q3 Quaternion component
/// @param degrees Whether to return result in degrees or radians
void fs_q_to_euler_angles(float *estRoll, float *estPitch, float *estYaw, float q0, float q1, float q2, float q3, bool degrees)
{
    *estRoll = atan2f(q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2);
    *estPitch = asinf(-2.0f * (q1 * q3 - q0 * q2));
    *estYaw = atan2f(q1 * q2 + q0 * q3, 0.5f - q2 * q2 - q3 * q3);

    if (degrees)
    {
        *estRoll *= 180.0f / FS_PI;
        *estPitch *= 180.0f / FS_PI;
        *estYaw *= 180.0f / FS_PI;
    }
}

int fs_get_last_timestamp(fs_device_info_t *device_handle, uint32_t *time)
{
    if (time == NULL)
    {
        return -1;
    }
    *time = device_handle->last_received_packet.composite.time;
    return 0;
}

int fs_get_raw_packet(fs_device_info_t *device_handle, fs_packet_union_t *packet_buffer)
{
    if (packet_buffer == NULL)
    {
        return -1;
    }

    memcpy(packet_buffer, &device_handle->last_received_packet, sizeof(fs_packet_union_t));
    return 0;
}

int fs_get_orientation(fs_device_info_t *device_handle, fs_quaternion_t *orientation_buffer)
{
    if (orientation_buffer == NULL)
    {
        return -1;
    }

    orientation_buffer->w = device_handle->last_received_packet.composite.q0;
    orientation_buffer->x = device_handle->last_received_packet.composite.q1;
    orientation_buffer->y = device_handle->last_received_packet.composite.q2;
    orientation_buffer->z = device_handle->last_received_packet.composite.q3;
    return 0;
}

int fs_get_orientation_euler(fs_device_info_t *device_handle, fs_vector3_t *orientation_buffer, bool degrees)
{
    if (orientation_buffer == NULL)
    {
        return -1;
    }

    fs_quaternion_t quat = {0};
    if (fs_get_orientation(device_handle, &quat) != 0)
    {
        return -1;
    }

    fs_q_to_euler_angles(&orientation_buffer->x, &orientation_buffer->y, &orientation_buffer->z, quat.w, quat.x, quat.y, quat.z, degrees);
    return 0;
}

int fs_get_acceleration(fs_device_info_t *device_handle, fs_vector3_t *acceleration_buffer)
{
    if (acceleration_buffer == NULL)
    {
        return -1;
    }

    acceleration_buffer->x = device_handle->last_received_packet.composite.ax0 + device_handle->last_received_packet.composite.ax1 + device_handle->last_received_packet.composite.ax2;
    acceleration_buffer->y = device_handle->last_received_packet.composite.ay0 + device_handle->last_received_packet.composite.ay1 + device_handle->last_received_packet.composite.ay2;
    acceleration_buffer->z = device_handle->last_received_packet.composite.az0 + device_handle->last_received_packet.composite.az1 + device_handle->last_received_packet.composite.az2;

    acceleration_buffer->x /= (3.0f * (float)INT16_MAX);
    acceleration_buffer->y /= (3.0f * (float)INT16_MAX);
    acceleration_buffer->z /= (3.0f * (float)INT16_MAX);

    acceleration_buffer->x *= (float)FS_ACCEL_SCALER_Gs;
    acceleration_buffer->y *= (float)FS_ACCEL_SCALER_Gs;
    acceleration_buffer->z *= (float)FS_ACCEL_SCALER_Gs;

    return 0;
}

int fs_get_angular_velocity(fs_device_info_t *device_handle, fs_vector3_t *angular_velocity_buffer)
{
    if (angular_velocity_buffer == NULL)
    {
        return -1;
    }

    switch (device_handle->last_received_packet.composite.type)
    {
    case C_PACKET_TYPE_IMU:
    case C_PACKET_TYPE_AHRS:
    case C_PACKET_TYPE_INS:
    case C_PACKET_TYPE_GNSS:
        angular_velocity_buffer->x = device_handle->last_received_packet.composite.gx0 + device_handle->last_received_packet.composite.gx1 + device_handle->last_received_packet.composite.gx2;
        angular_velocity_buffer->y = device_handle->last_received_packet.composite.gy0 + device_handle->last_received_packet.composite.gy1 + device_handle->last_received_packet.composite.gy2;
        angular_velocity_buffer->z = device_handle->last_received_packet.composite.gz0 + device_handle->last_received_packet.composite.gz1 + device_handle->last_received_packet.composite.gz2;

        angular_velocity_buffer->x /= (3.0f * (float)INT16_MAX);
        angular_velocity_buffer->y /= (3.0f * (float)INT16_MAX);
        angular_velocity_buffer->z /= (3.0f * (float)INT16_MAX);

        angular_velocity_buffer->x *= (float)FS_GYRO_SCALER_DPS;
        angular_velocity_buffer->y *= (float)FS_GYRO_SCALER_DPS;
        angular_velocity_buffer->z *= (float)FS_GYRO_SCALER_DPS;
        return 0;
    }
    return -1;
}

int fs_get_velocity(fs_device_info_t *device_handle, fs_vector3_t *velocity_buffer)
{
    if (velocity_buffer == NULL)
    {
        return -1;
    }

    velocity_buffer->x = device_handle->last_received_packet.composite.vx;
    velocity_buffer->y = device_handle->last_received_packet.composite.vy;
    velocity_buffer->z = device_handle->last_received_packet.composite.vz;
    return 0;
}

int fs_get_movement_state(fs_device_info_t *device_handle, fs_run_status_t *device_state_buffer)
{
    if (device_state_buffer == NULL)
    {
        return -1;
    }
    *device_state_buffer = (device_handle->last_received_packet.composite.device_in_motion == 1) ? FS_RUN_STATUS_IDLE : FS_RUN_STATUS_RUNNING;
    return 0;
}

int fs_get_position(fs_device_info_t *device_handle, fs_vector3_t *position_buffer)
{
    if (position_buffer == NULL)
    {
        return -1;
    }

    position_buffer->x = device_handle->last_received_packet.composite.rx;
    position_buffer->y = device_handle->last_received_packet.composite.ry;
    position_buffer->z = device_handle->last_received_packet.composite.rz;
    return 0;
}

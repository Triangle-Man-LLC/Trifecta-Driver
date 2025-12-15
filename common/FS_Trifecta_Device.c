/// Driver for the Trifecta series of IMU/AHRS/INS devices
/// Copyright 2025 4rge.ai and/or Triangle Man LLC
/// Usage and redistribution of this code is permitted
/// but this notice must be retained in all copies of the code.

/// THIS SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
/// INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE,
/// AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
/// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include "FS_Trifecta_Device.h"
#include "FS_Trifecta_Device_Utils.h"

/// @brief Get the current number of packets in device buffer.
/// Note that the oldest elements are overwritten if overflowed.
/// @param device_handle
/// @return Number of elements, between 0 to FS_MAX_PACKET_QUEUE_LENGTH
int fs_device_get_packet_count(const fs_device_info_t *device_handle)
{
    return device_handle->packet_buf_queue.count;
}

/// @brief Copy the packet at indicated index to the buffer for consumption.
/// @param device_handle
/// @param packet_buffer Destination buffer.
/// @param index
/// @return An fs_packet_type_t indicating packet ID, else -1 on failure
int fs_device_get_packet_at_index(const fs_device_info_t *device_handle, fs_packet_union_t *packet_buffer, int index)
{
    if (!device_handle || !packet_buffer)
        return -1;

    const fs_packet_ringbuffer_t *rb = &device_handle->packet_buf_queue;

    if (index < 0 || index >= rb->count)
        return -1;

    uint16_t pos = (rb->tail + index) % FS_MAX_PACKET_QUEUE_LENGTH;
    *packet_buffer = rb->buffer[pos];

    return packet_buffer->composite.type;
}

/// @brief Get a pointer to the packet at the indicated index.
/// @param device_handle Device handle
/// @param index Index into the packet queue (0 = oldest)
/// @return Pointer to packet, or NULL on failure
const fs_packet_union_t *fs_device_get_packet_pointer_at_index(const fs_device_info_t *device_handle, int index)
{
    if (!device_handle || index < 0)
        return NULL;

    const fs_packet_ringbuffer_t *rb = &device_handle->packet_buf_queue;

    if (index >= rb->count)
        return NULL;

    uint16_t pos = (rb->tail + index) % FS_MAX_PACKET_QUEUE_LENGTH;
    return &rb->buffer[pos];
}

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
    fs_command_info_t cmd = {0};
    while (FS_RINGBUFFER_POP(&device_handle->command_queue, FS_MAX_CMD_QUEUE_LENGTH, &cmd))
    {
        size_t command_length = fs_safe_strnlen((char *)cmd.payload, FS_MAX_CMD_LENGTH);
        fs_log_output("[Trifecta-Device] Command (len %ld): %c Params: %s", command_length, cmd.payload[0], cmd.payload + 1);
        const char cmd_char = cmd.payload[0];
        const char *params = (char *)&cmd.payload[1];
        switch (cmd_char)
        {
        case CMD_IDENTIFY:
        {
            fs_safe_strncpy(device_handle->device_descriptor.device_name, params, sizeof(device_handle->device_descriptor.device_name) - 1);
            device_handle->device_descriptor.device_name[sizeof(device_handle->device_descriptor.device_name) - 1] = '\0';
        }
        break;
        case CMD_IDENTIFY_PARAM_UART_BAUD_RATE:
        {
            int baud = atoi(params);
            if (baud > 0)
            {
                device_handle->device_params.baudrate = baud;
                fs_log_output("[Trifecta-Device] Baudrate set to: %d", baud);
            }
            else
            {
                fs_log_output("[Trifecta-Device] Invalid baudrate: %s", params);
            }
        }
        break;
        case CMD_IDENTIFY_PARAM_SSID:
            fs_safe_strncpy(device_handle->device_params.ssid, params, sizeof(device_handle->device_params.ssid) - 1);
            device_handle->device_params.ssid[sizeof(device_handle->device_params.ssid) - 1] = '\0';
            fs_log_output("[Trifecta-Device] STA SSID set to: %s", device_handle->device_params.ssid);
            break;
        case CMD_IDENTIFY_PARAM_SSID_AP:
            fs_safe_strncpy(device_handle->device_params.ssid_ap, params, sizeof(device_handle->device_params.ssid_ap) - 1);
            device_handle->device_params.ssid_ap[sizeof(device_handle->device_params.ssid_ap) - 1] = '\0';
            fs_log_output("[Trifecta-Device] AP SSID set to: %s", device_handle->device_params.ssid_ap);
            break;
        case CMD_IDENTIFY_PARAM_PASSWORD_AP:
            fs_safe_strncpy(device_handle->device_params.pw_ap, params, sizeof(device_handle->device_params.pw_ap) - 1);
            device_handle->device_params.pw_ap[sizeof(device_handle->device_params.pw_ap) - 1] = '\0';
            fs_log_output("[Trifecta-Device] AP Password updated.");
            break;
        case CMD_IDENTIFY_PARAM_TRANSMIT:
            int mode = atoi(params);
            device_handle->device_params.all_enabled_interfaces = (fs_communication_mode_t)mode;
            fs_log_output("[Trifecta-Device] Comm mode set to: %d", mode);
            break;
        case CMD_IDENTIFY_PARAM_DEV_SN:
            fs_safe_strncpy(device_handle->device_descriptor.device_sn, params, sizeof(device_handle->device_descriptor.device_sn) - 1);
            device_handle->device_descriptor.device_sn[sizeof(device_handle->device_descriptor.device_sn) - 1] = '\0';
            fs_log_output("[Trifecta-Device] Serial Number set to: %s", device_handle->device_descriptor.device_sn);
            break;
        case CMD_IDENTIFY_PARAM_DEVMODEL:
            fs_safe_strncpy(device_handle->device_descriptor.device_model, params, sizeof(device_handle->device_descriptor.device_model) - 1);
            device_handle->device_descriptor.device_model[sizeof(device_handle->device_descriptor.device_model) - 1] = '\0';
            fs_log_output("[Trifecta-Device] Device Model set to: %s", device_handle->device_descriptor.device_model);
            break;
        case CMD_IDENTIFY_PARAM_DEVFWVERSION:
            fs_safe_strncpy(device_handle->device_descriptor.device_fw, params, sizeof(device_handle->device_descriptor.device_fw) - 1);
            device_handle->device_descriptor.device_fw[sizeof(device_handle->device_descriptor.device_fw) - 1] = '\0';
            fs_log_output("[Trifecta-Device] Firmware Version set to: %s", device_handle->device_descriptor.device_fw);
            break;
        case CMD_IDENTIFY_PARAM_DEVDESC:
            fs_safe_strncpy(device_handle->device_descriptor.device_desc, params, sizeof(device_handle->device_descriptor.device_desc) - 1);
            device_handle->device_descriptor.device_desc[sizeof(device_handle->device_descriptor.device_desc) - 1] = '\0';
            fs_log_output("[Trifecta-Device] Device Description set to: %s", device_handle->device_descriptor.device_desc);
            break;
        }
        memset(&cmd, 0, sizeof(cmd)); // Clear for next iteration
    }
    return 0;
}

int fs_device_process_packets_serial(fs_device_info_t *device_handle, const void *rx_buf, size_t rx_len)
{
    fs_log_output("[Trifecta-Device] RX: Pushing %zu bytes into circular buffer", rx_len);
    fs_cb_push(&device_handle->data_buffer, (const uint8_t *)rx_buf, rx_len);

    uint8_t temp[FS_MAX_DATA_LENGTH];
    while (1)
    {
        size_t peeked = fs_cb_peek(&device_handle->data_buffer, temp, sizeof(temp));
        if (peeked == 0)
        {
            fs_log_output("[Trifecta-Device] Buffer empty after peek, exiting loop");
            break;
        }

        fs_log_output("[Trifecta-Device] Peeked %zu bytes from buffer", peeked);
        fs_log_output("[Trifecta-Device] Peeked data: %.*s", (int)peeked, temp);

        fs_delimiter_indices_t indices = fs_scan_delimiters(temp, peeked);
        fs_log_output("[Trifecta-Device] Delimiter scan: colon=%d, binary=%d, exclam=%d, semicolon=%d",
                      indices.colon_index, indices.binary_index, indices.exclam_index, indices.semicolon_index);
        if (indices.binary_index != -1 && indices.exclam_index > indices.binary_index)
        {
            // Binary packet
            size_t packet_start = indices.binary_index + 1;
            size_t packet_len = indices.exclam_index - packet_start - 1;

            fs_log_output("[Trifecta-Device] Detected Binary packet framing from %d to %d (length %zu)",
                          indices.binary_index, indices.exclam_index, packet_len);

            if (segment_packets(device_handle, &temp[packet_start], packet_len) <= 0)
                fs_log_output("[Trifecta-Device] Failed to segment binary packet");

            fs_cb_pop(&device_handle->data_buffer, NULL, indices.exclam_index + 1);
        }

        else if (indices.colon_index != -1 && indices.exclam_index > indices.colon_index)
        {
            // Base64 packet
            size_t payload_start = indices.colon_index + 1;
            size_t payload_len = indices.exclam_index - payload_start;

            fs_log_output("[Trifecta-Device] Detected Base64 packet framing from %d to %d (payload length %zu)",
                          indices.colon_index, indices.exclam_index, payload_len);

            fs_log_output("[Trifecta-Device] Attempting to decode Base64 payload: %.*s",
                          (int)payload_len, &temp[payload_start]);

            if (base64_to_packet(device_handle, (char *)&temp[payload_start], payload_len) != 0)
                fs_log_output("[Trifecta-Device] Failed to decode Base64 packet");

            size_t packet_len = indices.exclam_index + 1;
            fs_cb_pop(&device_handle->data_buffer, NULL, packet_len);
        }

        else if (indices.semicolon_index != -1)
        {
            // Command segment
            size_t cmd_len = indices.semicolon_index + 1;
            fs_log_output("[Trifecta-Device] Detected command ending at index %d (length %zu)",
                          indices.semicolon_index, cmd_len);

            if (fs_handle_received_commands(device_handle, temp, peeked) != 0)
                fs_log_output("[Trifecta-Device] Failed to parse commands");

            fs_cb_pop(&device_handle->data_buffer, NULL, cmd_len);
        }
        else
        {
            fs_log_output("[Trifecta-Device] No complete segment found ... waiting for more data");
            return -1;
        }
    }
    return 0;
}

/// @brief Attempt to parse the packet(s) in the rx_buf
/// @param rx_buf The buffer to read from
/// @param rx_len The length to read
/// @param source FS_COMMUNICATION_MODE_SERIAL, FS_COMMUNICATION_MODE_TCP_UDP, etc. are handled differently
/// @return Number of parsed packets, or -1 on fail.
int fs_device_parse_packet(fs_device_info_t *device_handle, const void *rx_buf, ssize_t rx_len, fs_communication_mode_t source)
{
    if (rx_len <= 0)
    {
        return -1;
    }

    switch (source)
    {
    // UDP, I2C, and SPI modes all transfer binary data packets.
    case FS_COMMUNICATION_MODE_TCP_UDP:
    case FS_COMMUNICATION_MODE_TCP_UDP_AP:
    case FS_COMMUNICATION_MODE_I2C:
    case FS_COMMUNICATION_MODE_SPI:
    {
        int packets = segment_packets(device_handle, rx_buf, rx_len);
        if (packets < 0)
        {
            fs_log_output("[Trifecta-Device] Error parsing packets!");
            return -1;
        }
        fs_log_output("[Trifecta-Device] Successfully processed packets (network format)!");
    }
    break;
    // UART and CDC modes all transfer Base64-encoded packets.
    case FS_COMMUNICATION_MODE_UART:
    case FS_COMMUNICATION_MODE_USB_CDC:
    {
        if (fs_device_process_packets_serial(device_handle, rx_buf, rx_len) != 0)
        {
            fs_log_output("[Trifecta-Device] Error parsing packets!");
            return -1;
        }
        fs_log_output("[Trifecta-Device] Successfully processed packets!");
    }
    break;
    // BLE/CAN driver support will be added in the future, for platforms which support them.
    case FS_COMMUNICATION_MODE_BLE:
    case FS_COMMUNICATION_MODE_CAN:
        fs_log_output("[Trifecta-Device] These modes are not yet supported!");
        return -1;
    default:
        fs_log_output("[Trifecta-Device] Unknown packet source!");
        return -1;
    }
    return 0;
}

/// @brief Utility function to get Euler angles from quaternion
/// @param x Pointer to buffer for storing roll value
/// @param y Pointer to buffer for storing pitch value
/// @param z Pointer to buffer for storing yaw value
/// @param q0 Quaternion component
/// @param q1 Quaternion component
/// @param q2 Quaternion component
/// @param q3 Quaternion component
/// @param degrees Whether to return result in degrees or radians
void fs_q_to_euler_angles(float *x, float *y, float *z, float q0, float q1, float q2, float q3, bool degrees)
{
    *x = atan2f(q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2);
    *y = asinf(-2.0f * (q1 * q3 - q0 * q2));
    *z = atan2f(q1 * q2 + q0 * q3, 0.5f - q2 * q2 - q3 * q3);

    if (degrees)
    {
        *x *= 180.0f / FS_PI;
        *y *= 180.0f / FS_PI;
        *z *= 180.0f / FS_PI;
    }
}

int fs_get_last_timestamp(fs_device_info_t *device_handle, uint32_t *time)
{
    if (!device_handle || !time)
        return -1;

    const fs_packet_union_t *pkt = &device_handle->last_received_packet;
    if (!pkt)
        return -1;

    *time = pkt->composite.time;
    return 0;
}

int fs_get_raw_packet(fs_device_info_t *device_handle, fs_packet_union_t *packet_buffer)
{
    if (!device_handle || !packet_buffer)
        return -1;

    const fs_packet_union_t *pkt = &device_handle->last_received_packet;
    if (!pkt)
        return -1;

    memcpy(packet_buffer, pkt, sizeof(fs_packet_union_t));
    return 0;
}

int fs_get_raw_packet_queue_size(fs_device_info_t *device_handle)
{
    if (!device_handle)
        return -1;

    return device_handle->packet_buf_queue.count;
}

int fs_get_raw_packet_from_queue(fs_device_info_t *device_handle, fs_packet_union_t *packet_buffer, int pos)
{
    if (!device_handle || !packet_buffer || pos < 0 || pos >= device_handle->packet_buf_queue.count)
        return -1;

    const fs_packet_union_t *pkt = fs_device_get_packet_pointer_at_index(device_handle, pos);
    if (!pkt)
        return -1;

    memcpy(packet_buffer, pkt, sizeof(fs_packet_union_t));
    return 0;
}

int fs_get_orientation(fs_device_info_t *device_handle, fs_quaternion_t *orientation_buffer)
{
    if (!device_handle || !orientation_buffer || device_handle->packet_buf_queue.count == 0)
        return -1;

    int last_index = device_handle->packet_buf_queue.count - 1;
    const fs_packet_union_t *pkt = &device_handle->last_received_packet;

    if (!pkt)
        return -1;
    switch (pkt->composite.type)
    {
    case C_PACKET_TYPE_IMU:
    case C_PACKET_TYPE_AHRS:
    case C_PACKET_TYPE_GNSS:
    case C_PACKET_TYPE_INS:
    {
        orientation_buffer->w = pkt->composite.q0;
        orientation_buffer->x = pkt->composite.q1;
        orientation_buffer->y = pkt->composite.q2;
        orientation_buffer->z = pkt->composite.q3;
        break;
    }
    case S_PACKET_TYPE_IMU:
    case S_PACKET_TYPE_AHRS:
    case S_PACKET_TYPE_GNSS:
    case S_PACKET_TYPE_INS:
    {
        orientation_buffer->w = pkt->regular.q0;
        orientation_buffer->x = pkt->regular.q1;
        orientation_buffer->y = pkt->regular.q2;
        orientation_buffer->z = pkt->regular.q3;
        break;
    }
    case C2_PACKET_TYPE_IMU:
    case C2_PACKET_TYPE_AHRS:
    case C2_PACKET_TYPE_GNSS:
    case C2_PACKET_TYPE_INS:
    {
        orientation_buffer->w = pkt->composite2.q0;
        orientation_buffer->x = pkt->composite2.q1;
        orientation_buffer->y = pkt->composite2.q2;
        orientation_buffer->z = pkt->composite2.q3;
        break;
    }
    default:
        return -1;
    }
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
    if (!device_handle || !acceleration_buffer)
        return -1;

    const fs_packet_union_t *pkt = &device_handle->last_received_packet;
    if (!pkt)
        return -1;
    switch (pkt->composite.type)
    {
    case C_PACKET_TYPE_IMU:
    case C_PACKET_TYPE_AHRS:
    case C_PACKET_TYPE_GNSS:
    case C_PACKET_TYPE_INS:
    {
        acceleration_buffer->x = (pkt->composite.ax0 + pkt->composite.ax1 + pkt->composite.ax2) / (3.0f * (float)INT16_MAX);
        acceleration_buffer->y = (pkt->composite.ay0 + pkt->composite.ay1 + pkt->composite.ay2) / (3.0f * (float)INT16_MAX);
        acceleration_buffer->z = (pkt->composite.az0 + pkt->composite.az1 + pkt->composite.az2) / (3.0f * (float)INT16_MAX);

        acceleration_buffer->x *= FS_ACCEL_SCALER_Gs;
        acceleration_buffer->y *= FS_ACCEL_SCALER_Gs;
        acceleration_buffer->z *= FS_ACCEL_SCALER_Gs;
        break;
    }
    case S_PACKET_TYPE_IMU:
    case S_PACKET_TYPE_AHRS:
    case S_PACKET_TYPE_GNSS:
    case S_PACKET_TYPE_INS:
    {
        acceleration_buffer->x = pkt->regular.ax;
        acceleration_buffer->y = pkt->regular.ay;
        acceleration_buffer->z = pkt->regular.az;
        break;
    }
    case C2_PACKET_TYPE_IMU:
    case C2_PACKET_TYPE_AHRS:
    case C2_PACKET_TYPE_GNSS:
    case C2_PACKET_TYPE_INS:
    {
        acceleration_buffer->x = (pkt->composite2.ax0 + pkt->composite2.ax1 + pkt->composite2.ax2) / (3.0f * (float)INT16_MAX);
        acceleration_buffer->y = (pkt->composite2.ay0 + pkt->composite2.ay1 + pkt->composite2.ay2) / (3.0f * (float)INT16_MAX);
        acceleration_buffer->z = (pkt->composite2.az0 + pkt->composite2.az1 + pkt->composite2.az2) / (3.0f * (float)INT16_MAX);

        acceleration_buffer->x *= FS_ACCEL_SCALER_Gs;
        acceleration_buffer->y *= FS_ACCEL_SCALER_Gs;
        acceleration_buffer->z *= FS_ACCEL_SCALER_Gs;

        break;
    }
    default:
        return -1;
    }
    return 0;
}

int fs_get_angular_velocity(fs_device_info_t *device_handle, fs_vector3_t *angular_velocity_buffer)
{
    if (!device_handle || !angular_velocity_buffer)
        return -1;

    const fs_packet_union_t *pkt = &device_handle->last_received_packet;
    if (!pkt)
        return -1;
    switch (pkt->composite.type)
    {
    case C_PACKET_TYPE_IMU:
    case C_PACKET_TYPE_AHRS:
    case C_PACKET_TYPE_GNSS:
    case C_PACKET_TYPE_INS:
    {
        angular_velocity_buffer->x = (pkt->composite.gx0 + pkt->composite.gx1 + pkt->composite.gx2) / (3.0f * (float)INT16_MAX);
        angular_velocity_buffer->y = (pkt->composite.gy0 + pkt->composite.gy1 + pkt->composite.gy2) / (3.0f * (float)INT16_MAX);
        angular_velocity_buffer->z = (pkt->composite.gz0 + pkt->composite.gz1 + pkt->composite.gz2) / (3.0f * (float)INT16_MAX);

        angular_velocity_buffer->x *= FS_GYRO_SCALER_DPS;
        angular_velocity_buffer->y *= FS_GYRO_SCALER_DPS;
        angular_velocity_buffer->z *= FS_GYRO_SCALER_DPS;
        break;
    }
    case S_PACKET_TYPE_IMU:
    case S_PACKET_TYPE_AHRS:
    case S_PACKET_TYPE_GNSS:
    case S_PACKET_TYPE_INS:
    {
        angular_velocity_buffer->x = (pkt->regular.omega_x0);
        angular_velocity_buffer->y = (pkt->regular.omega_y0);
        angular_velocity_buffer->z = (pkt->regular.omega_z0);
        break;
    }
    case C2_PACKET_TYPE_IMU:
    case C2_PACKET_TYPE_AHRS:
    case C2_PACKET_TYPE_GNSS:
    case C2_PACKET_TYPE_INS:
    {
        angular_velocity_buffer->x = (pkt->composite2.gx0 + pkt->composite2.gx1 + pkt->composite2.gx2) / (3.0f * (float)INT16_MAX);
        angular_velocity_buffer->y = (pkt->composite2.gy0 + pkt->composite2.gy1 + pkt->composite2.gy2) / (3.0f * (float)INT16_MAX);
        angular_velocity_buffer->z = (pkt->composite2.gz0 + pkt->composite2.gz1 + pkt->composite2.gz2) / (3.0f * (float)INT16_MAX);

        angular_velocity_buffer->x *= FS_GYRO_SCALER_DPS;
        angular_velocity_buffer->y *= FS_GYRO_SCALER_DPS;
        angular_velocity_buffer->z *= FS_GYRO_SCALER_DPS;
        break;
    }
    default:
        return -1;
    }
    return 0;
}

int fs_get_velocity(fs_device_info_t *device_handle, fs_vector3_t *velocity_buffer)
{
    if (!device_handle || !velocity_buffer)
        return -1;

    const fs_packet_union_t *pkt = &device_handle->last_received_packet;
    if (!pkt)
        return -1;
    switch (pkt->composite.type)
    {
    case C_PACKET_TYPE_IMU:
    case C_PACKET_TYPE_AHRS:
    case S_PACKET_TYPE_IMU:
    case S_PACKET_TYPE_AHRS:
    case C2_PACKET_TYPE_IMU:
    case C2_PACKET_TYPE_AHRS:
    {
        // Note: No velocity available from IMU
        return -1;
    }
    case C_PACKET_TYPE_INS:
    case S_PACKET_TYPE_INS:
    case C2_PACKET_TYPE_INS:
    case C_PACKET_TYPE_GNSS:
    case S_PACKET_TYPE_GNSS:
    case C2_PACKET_TYPE_GNSS:
    {
        velocity_buffer->x = pkt->composite2.vx;
        velocity_buffer->y = pkt->composite2.vy;
        velocity_buffer->z = pkt->composite2.vz;
        break;
    }
    default:
        return -1;
    }
    return 0;
}

int fs_get_movement_state(fs_device_info_t *device_handle, fs_run_status_t *device_state_buffer)
{
    if (!device_handle || !device_state_buffer)
        return -1;

    const fs_packet_union_t *pkt = &device_handle->last_received_packet;
    if (!pkt)
        return -1;
    switch (pkt->composite.type)
    {
    case C_PACKET_TYPE_IMU:
    case C_PACKET_TYPE_AHRS:
    case C_PACKET_TYPE_GNSS:
    case C_PACKET_TYPE_INS:
    {
        *device_state_buffer = (pkt->composite.device_motion_status == 1) ? FS_RUN_STATUS_IDLE : FS_RUN_STATUS_RUNNING;
        break;
    }
    case S_PACKET_TYPE_IMU:
    case S_PACKET_TYPE_AHRS:
    case S_PACKET_TYPE_GNSS:
    case S_PACKET_TYPE_INS:
    {
        *device_state_buffer = (pkt->regular.device_motion_status == 1) ? FS_RUN_STATUS_IDLE : FS_RUN_STATUS_RUNNING;
        break;
    }
    case C2_PACKET_TYPE_IMU:
    case C2_PACKET_TYPE_AHRS:
    case C2_PACKET_TYPE_GNSS:
    case C2_PACKET_TYPE_INS:
    {
        *device_state_buffer = (pkt->composite2.device_motion_status == 1) ? FS_RUN_STATUS_IDLE : FS_RUN_STATUS_RUNNING;
        break;
    }
    default:
        return -1;
    }
    return 0;
}

int fs_get_position(fs_device_info_t *device_handle, fs_vector3_d_t *position_buffer)
{
    if (!device_handle || !position_buffer)
        return -1;

    const fs_packet_union_t *pkt = &device_handle->last_received_packet;
    if (!pkt)
        return -1;
    switch (pkt->composite.type)
    {
    case C_PACKET_TYPE_IMU:
    case C_PACKET_TYPE_AHRS:
    case S_PACKET_TYPE_IMU:
    case S_PACKET_TYPE_AHRS:
    case C2_PACKET_TYPE_IMU:
    case C2_PACKET_TYPE_AHRS:
    {
        // Note: No position available from IMU
        return -1;
    }
    case C_PACKET_TYPE_INS:
    case S_PACKET_TYPE_INS:
    case C2_PACKET_TYPE_INS:
    case C_PACKET_TYPE_GNSS:
    case S_PACKET_TYPE_GNSS:
    case C2_PACKET_TYPE_GNSS:
    {
        position_buffer->x = pkt->composite2.rx;
        position_buffer->y = pkt->composite2.ry;
        position_buffer->z = pkt->composite2.rz;
        break;
    }
    default:
        return -1;
    }
    return 0;
}

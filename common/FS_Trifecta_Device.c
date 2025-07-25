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

// Base64 character set
static const char base64_chars[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

// static fs_packet_union last_received_packet;

/// @brief Internal function to enqueue a command into the command queue...
/// @param str Command string
/// @param len Command length
/// @return 0 if successful
static int fs_enqueue_into_command_queue(fs_device_info *device_handle, char *str, size_t len)
{
    if (device_handle->command_queue_size >= FS_MAX_CMD_QUEUE_LENGTH)
    {
        fs_log_output("[Trifecta] Device command queue was full!");
        return -1;
    }
    device_handle->command_queue_size++;
    memset(&device_handle->command_queue[device_handle->command_queue_size - 1], 0, FS_MAX_CMD_LENGTH);
    memcpy(&device_handle->command_queue[device_handle->command_queue_size - 1], str, len);
    return 0;
}

/// @brief Parse all commands from a received string - used for TCP and serial command handlers
/// @param cmd_buf The buffer containing all commands
/// @param buf_len The length of the received buffer
/// @returns Error code (if any)
static int fs_segment_commands(fs_device_info *device_handle, const void *cmd_buf, size_t buf_len)
{
    char input_line[FS_MAX_DATA_LENGTH];
    unsigned int input_pos = 0;

    enum
    {
        WAITING,
        READING
    } state = WAITING;

    for (size_t i = 0; i < buf_len; i++)
    {
        char inByte = *((char *)cmd_buf + i);

        switch (inByte)
        {
        case ';':                         // Command terminated by ; character, this is in case we need to parse a longer command
            input_line[input_pos] = '\0'; // terminating null byte

            if (fs_enqueue_into_command_queue(device_handle, input_line, strnlen(input_line, FS_MAX_CMD_LENGTH)) != 0)
            {
                fs_log_output("[Trifecta] Error with device command segment/enqueue!");
                return -1;
            }

            state = WAITING; // Return back to waiting state
            // terminator reached! process input_line here ...
            // reset buffer for next time
            input_pos = 0;

            break;
        case '\n':
        case '\r':
        case '\0':
            if (state == WAITING)
            {
                input_line[input_pos] = '\0';
                input_pos = 0;
                break;
            }
            else
            {
                input_line[input_pos] = '\0';
                input_pos = 0;
                return 3;
            }
            break;

        default:
            state = READING; // Reading state
            // keep adding if not full ... allow for terminating null byte
            if (input_pos < (FS_MAX_DATA_LENGTH - 1))
                input_line[input_pos++] = inByte;
            break;
        }
    }

    // Clear the input_line buffer
    memset(input_line, 0, sizeof(input_line));

    return 0;
}

/// @brief Handle all potential received commands from the buffer
/// @param device_handle Device handle
/// @param cmd_buf The buffer containing data blob to process
/// @param buf_len Size of cmd_buf
/// @return 0 on success, -1 on failure
int fs_handle_received_commands(fs_device_info *device_handle, const void *cmd_buf, size_t buf_len)
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

// Helper function to map Base64 characters to values
static int base64_char_to_value(char c)
{
    if (c >= 'A' && c <= 'Z')
        return c - 'A';
    if (c >= 'a' && c <= 'z')
        return c - 'a' + 26;
    if (c >= '0' && c <= '9')
        return c - '0' + 52;
    if (c == '+')
        return 62;
    if (c == '/')
        return 63;
    return -1; // Should not happen for valid Base64 strings
}

// Base64 encode function
static int fs_base64_encode(const void *data, size_t len, char *output_buffer, size_t buffer_size)
{
    const unsigned char *input = (const unsigned char *)data;
    size_t i, j;

    if (buffer_size < 4 * ((len + 2) / 3) + 1)
    {
        // Buffer size is too small
        fprintf(stderr, "Buffer size is too small for Base64 encoding.\n");
        return -1;
    }

    for (i = 0, j = 0; i < len;)
    {
        uint32_t octet_a = i < len ? input[i++] : 0;
        uint32_t octet_b = i < len ? input[i++] : 0;
        uint32_t octet_c = i < len ? input[i++] : 0;

        uint32_t triple = (octet_a << 0x10) + (octet_b << 0x08) + octet_c;

        output_buffer[j++] = base64_chars[(triple >> 3 * 6) & 0x3F];
        output_buffer[j++] = base64_chars[(triple >> 2 * 6) & 0x3F];
        output_buffer[j++] = base64_chars[(triple >> 1 * 6) & 0x3F];
        output_buffer[j++] = base64_chars[(triple >> 0 * 6) & 0x3F];
    }

    // Add padding characters if needed
    for (i = 0; i < (4 * ((len + 2) / 3) - j); i++)
    {
        output_buffer[j++] = '=';
    }

    output_buffer[j] = '\0'; // Null terminator
    return 0;
}

// Base64 decode function
static int fs_base64_decode(const char *input, void *output_buffer, size_t buffer_size, size_t *decoded_length)
{
    if (buffer_size > FS_MAX_DATA_LENGTH)
    {
        return -1; // Prevent overflow
    }

    unsigned char *output = (unsigned char *)output_buffer;
    size_t input_length = buffer_size;
    size_t i, j;
    int padding = 0;

    // Calculate padding
    if (input_length >= 1 && input[input_length - 1] == '=')
        padding++;
    if (input_length >= 2 && input[input_length - 2] == '=')
        padding++;

    if (buffer_size < 3 * (input_length / 4) - padding)
    {
        // Buffer size is too small
        fprintf(stderr, "Buffer size is too small for Base64 decoding.\n");
        return -1;
    }

    *decoded_length = 0;
    for (i = 0, j = 0; i < input_length;)
    {
        uint32_t sextet_a = input[i] == '=' ? 0 : base64_char_to_value(input[i++]);
        uint32_t sextet_b = input[i] == '=' ? 0 : base64_char_to_value(input[i++]);
        uint32_t sextet_c = input[i] == '=' ? 0 : base64_char_to_value(input[i++]);
        uint32_t sextet_d = input[i] == '=' ? 0 : base64_char_to_value(input[i++]);

        if (sextet_a == -1 || sextet_b == -1 || sextet_c == -1 || sextet_d == -1)
        {
            // Invalid Base64 character detected
            // fprintf(stderr, "Invalid Base64 character detected at index %u: 0x%X\n", i, input[i - 1]);
            // fprintf(stderr, "String in question (len %d): %s\n", input_length, input);
            // if(input[i-1] == 0x0)
            // {
            //     continue; // Ignore the space which appears for some reason...
            // }
            return -1;
        }

        uint32_t triple = (sextet_a << 3 * 6) + (sextet_b << 2 * 6) + (sextet_c << 1 * 6) + (sextet_d << 0 * 6);

        if (j < buffer_size)
            output[j++] = (triple >> 2 * 8) & 0xFF;
        if (j < buffer_size)
            output[j++] = (triple >> 1 * 8) & 0xFF;
        if (j < buffer_size)
            output[j++] = (triple >> 0 * 8) & 0xFF;

        *decoded_length += 3;
    }

    // Correct length for padding '=' characters
    *decoded_length -= padding;
    return 0;
}

/// @brief
/// @param packet_type The packet type indication
/// @return
static int obtain_packet_length(int packet_type)
{
    size_t packet_length = 0;
    switch (packet_type)
    {
    case C_PACKET_TYPE_IMU:
    case C_PACKET_TYPE_AHRS:
    case C_PACKET_TYPE_INS:
    case C_PACKET_TYPE_GNSS:
        packet_length = sizeof(fs_imu_composite_packet);
        break;
    case S_PACKET_TYPE_IMU:
    case S_PACKET_TYPE_AHRS:
    case S_PACKET_TYPE_INS:
    case S_PACKET_TYPE_GNSS:
        packet_length = sizeof(fs_imu_regular_packet);
        break;
    case C2_PACKET_TYPE_IMU:
    case C2_PACKET_TYPE_AHRS:
    case C2_PACKET_TYPE_INS:
    case C2_PACKET_TYPE_GNSS:
        packet_length = sizeof(fs_imu_composite_packet_2);
        break;
    default:
        // Assume that data is string
        return 0;
    }

    return packet_length;
}

/// @brief Internal function to enqueue a packet into the packet queue...
/// @param str Command string
/// @param len Command length
/// @return 0 if successful
static int fs_enqueue_into_packet_queue(fs_device_info *device_handle, const fs_packet_union *packet, size_t len)
{
    if (device_handle->packet_buf_queue_size >= FS_MAX_PACKET_QUEUE_LENGTH)
    {
        fs_log_output("[Trifecta] Warning: Device packet queue was full! Clearing!");
        device_handle->packet_buf_queue_size = 0; // Clear and reset the queue...
        // return -1;
    }

    device_handle->packet_buf_queue_size++;
    memset(&device_handle->packet_buf_queue[device_handle->packet_buf_queue_size - 1], 0, sizeof(fs_packet_union));
    memcpy(&device_handle->packet_buf_queue[device_handle->packet_buf_queue_size - 1], packet, sizeof(fs_packet_union));
    return 0;
}

/// @brief Parse the packet buffer and attempt to separate out the packets
/// @param rx_buf
/// @param rx_len
/// @return
static int segment_packets(fs_device_info *device_handle, const void *rx_buf, size_t rx_len)
{
    if (rx_buf == NULL || rx_len < 1 || rx_len > FS_MAX_DATA_LENGTH)
    {
        fs_log_output("[Trifecta] Cannot segment packets from invalid receive buffer!");
        return -1;
    }
    uint8_t *buf = (uint8_t *)rx_buf;
    size_t pos = 0;
    int packet_count = 0;
    while (pos < rx_len)
    {
        uint8_t packet_type = (uint8_t)buf[pos];
        size_t packet_length = obtain_packet_length(packet_type);
        // Validate packet length
        if (packet_length == 0)
        {
            // It must be in the "CSV" format!
            return 0;
        }
        if (pos + packet_length > rx_len)
        {
            fs_log_output("[Trifecta] Error: Packet length %ld is out of bounds! RX_len: %ld", packet_length, rx_len);
            return -1;
        }
        // Emplace the packet into the queue
        fs_enqueue_into_packet_queue(device_handle, (fs_packet_union *)(buf + pos), packet_length);
        packet_count++;
        pos += packet_length;
    }

    return packet_count;
}

/// @brief Parse a Base64 string into the appropriate packet structure
/// @param segment The Base64 encoded string segment
/// @param length The length of the Base64 encoded string segment
/// @return Status code
static int base64_to_packet(fs_device_info *device_handle, char *segment, size_t length)
{
    // Check for null pointer
    if (segment == NULL)
    {
        return -1;
    }

    fs_packet_union packet_union;
    memset(&packet_union, 0, sizeof(fs_packet_union));
    size_t decoded_size = 0;

    if (fs_base64_decode(segment, &packet_union, length, &decoded_size) != 0)
    {
        return -1;
    }

    if (fs_enqueue_into_packet_queue(device_handle, &packet_union, sizeof(packet_union)) != 0)
    {
        fs_log_output("[Trifecta] Error: Could not place packet into packet queue!");
        return -1;
    }

    fs_log_output("[Trifecta] Scanned packet (B64)! Timestamp: %ld - Type: %d", packet_union.composite.time, packet_union.composite.type);
    return 0;
}

static int fs_device_process_packets_serial(fs_device_info *device_handle, const void *rx_buf, size_t rx_len)
{
    enum
    {
        SEARCHING_COMMAND_OR_PACKET_START = 0,
        SEARCHING_PACKET_TERMINATOR = 1,
        SEARCHING_COMMAND_TERMINATOR = 2,
    } scanner_state = SEARCHING_COMMAND_OR_PACKET_START;

    const char *dataString = (char *)rx_buf;

    static char last_data_string[FS_MAX_DATA_LENGTH] = {0};
    static int last_data_string_index = 0;

    size_t dataStringLen = rx_len;
    size_t last_data_string_length = strnlen(last_data_string, FS_MAX_PACKET_LENGTH);

    fs_log_output("At start, scanner state: %d, last_data_string (len %d) - %s", scanner_state, last_data_string_length, last_data_string);
    // Concatenate any remaining data from the last call
    if (last_data_string_length > 0)
    {
        if (last_data_string_length + dataStringLen < FS_MAX_DATA_LENGTH)
        {
            strncat(last_data_string, dataString, dataStringLen);
        }
        else
        {
            // Buffer overflow protection, discard data if concatenation would exceed buffer
            memset(last_data_string, 0, FS_MAX_DATA_LENGTH);
            last_data_string_length = 0;
            fs_log_output("[Trifecta] Warning: Buffer overflow! Requested buffer size: %ld", last_data_string_length + dataStringLen);
            return -1;
        }
    }
    else
    {
        strncpy(last_data_string, dataString, dataStringLen);
    }

    dataStringLen = strnlen(last_data_string, FS_MAX_DATA_LENGTH);
    last_data_string_length = dataStringLen;

    fs_log_output("After concat, scanner state: %d, last_data_string (len %d) - %s", scanner_state, dataStringLen, last_data_string);

    int startIndex = -1;
    int endIndex = -1;

    for (int index = 0; index < dataStringLen; index++)
    {
        switch (scanner_state)
        {
        case SEARCHING_COMMAND_OR_PACKET_START: // Looking for FS_SERIAL_PACKET_HEADER or any other character...
        {
            if (last_data_string[index] == FS_SERIAL_PACKET_HEADER)
            {
                // If we found FS_SERIAL_PACKET_HEADER
                if (index == dataStringLen - 1)
                {
                    memset(last_data_string, 0, sizeof(last_data_string));
                    last_data_string[0] = FS_SERIAL_PACKET_HEADER;
                }
                else
                {
                    startIndex = index;
                    scanner_state = SEARCHING_PACKET_TERMINATOR;
                }
            }
            else
            {
                startIndex = index;
                scanner_state = SEARCHING_COMMAND_TERMINATOR;
            }
            break;
        }
        case SEARCHING_PACKET_TERMINATOR: // Looking for FS_SERIAL_PACKET_FOOTER
        {
            if (last_data_string[index] == FS_SERIAL_PACKET_HEADER)
            {
                // If we found FS_SERIAL_PACKET_HEADER
                if (index == dataStringLen - 1)
                {
                    memset(last_data_string, 0, sizeof(last_data_string));
                    last_data_string[0] = FS_SERIAL_PACKET_HEADER;
                }
                else
                {
                    startIndex = index;
                    scanner_state = SEARCHING_PACKET_TERMINATOR;
                }
            }
            else if (last_data_string[index] == FS_SERIAL_PACKET_FOOTER)
            {
                endIndex = index;
                scanner_state = SEARCHING_COMMAND_OR_PACKET_START;

                // Acquire the section of the string between startIndex and endIndex
                // Then send it to the processor. In the meantime, we return to
                // SEARCHING_COMMAND_OR_PACKET_START mode
                char segment[FS_MAX_PACKET_LENGTH];
                memset(segment, 0, sizeof(segment));
                strncpy(segment, last_data_string + startIndex + 1, endIndex - startIndex - 1);

                // if (base64_to_packet(segment, endIndex - startIndex - 1) != 0 && csv_to_packet(segment, endIndex - startIndex - 1) != 0)
                if (base64_to_packet(device_handle, segment, endIndex - startIndex - 1) != 0)
                {
                    fs_log_output("[Trifecta] Warning: Failed to scan packet!");
                }
                fs_log_output("[Trifecta] Scanner state: %d, PCKT (len %d): %s\n", scanner_state, strnlen(segment, endIndex - startIndex - 1), segment);

                // Reset the buffer for the next search
                memset(&last_data_string, 0, index + 1);

                startIndex = index + 1;
                endIndex = -1;
            }
            else if (index == dataStringLen - 1)
            {
                // In this case, we reached the end without finding the FS_SERIAL_PACKET_FOOTER
                // This means that we should retain this segment of last_data_string for the next use!
                memmove(last_data_string, last_data_string + startIndex, dataStringLen - startIndex);
                // Correct the length of last_data_string to the retained portion
                last_data_string[dataStringLen - startIndex] = 0;
            }
            break;
        }
        case SEARCHING_COMMAND_TERMINATOR:
        {
            if (last_data_string[index] == FS_SERIAL_COMMAND_TERMINATOR)
            {
                endIndex = index;
                scanner_state = SEARCHING_COMMAND_OR_PACKET_START;

                if (endIndex - startIndex > FS_MAX_CMD_LENGTH)
                {
                    // If the command was too long, then ignore and retry
                    scanner_state = SEARCHING_COMMAND_OR_PACKET_START;
                    startIndex = -1;
                    endIndex = -1;
                    fs_log_output("[Trifecta] Scanner state: %d, Command length %d exceeded maximum of %d, skipping!", scanner_state, endIndex - startIndex, FS_MAX_CMD_LENGTH);
                    continue;
                }

                // Acquire the section of the string between startIndex and endIndex
                // Then send it to the processor. In the meantime, we return to
                // SEARCHING_COMMAND_OR_PACKET_START mode
                char segment[FS_MAX_CMD_LENGTH] = {0};
                memset(segment, 0, sizeof(segment));
                strncpy(segment, last_data_string + startIndex, endIndex - startIndex + 1);
                if (fs_handle_received_commands(device_handle, segment, endIndex - startIndex + 1) != 0)
                {
                    fs_log_output("[Trifecta] Scanner state: %d,  Failed to handle received commands!", scanner_state);
                }

                fs_log_output("CMD: %s\n", segment);
                if (index == dataStringLen - 1)
                {
                    // Reset the buffer for the next search
                    memmove(last_data_string, last_data_string + endIndex + 1, dataStringLen - endIndex - 1);
                    last_data_string[dataStringLen - endIndex] = 0;
                }

                startIndex = -1;
                endIndex = -1;
            }
            else if (last_data_string[index] == FS_SERIAL_PACKET_HEADER)
            {
                startIndex = index;
                endIndex = -1;
                scanner_state = SEARCHING_PACKET_TERMINATOR;
                if (index == dataStringLen - 1)
                {
                    // No terminator found; discard up to this header and retry next time
                    memmove(last_data_string, last_data_string + startIndex, dataStringLen - startIndex);
                    dataStringLen -= startIndex;
                    last_data_string[dataStringLen] = '\0';
                }
            }
            else if (index == dataStringLen - 1 && (last_data_string[index] != FS_SERIAL_COMMAND_TERMINATOR))
            {
                // In this case, we reached the end without finding the FS_SERIAL_COMMAND_TERMINATOR
                // This means that we should retain this segment of last_data_string for the next use!
                memmove(last_data_string, last_data_string + startIndex, dataStringLen - startIndex);
                last_data_string[dataStringLen - startIndex] = 0;
            }
            break;
        }
        default:
            break;
        }
    }

    last_data_string_length = strnlen(last_data_string, FS_MAX_DATA_LENGTH);
    fs_log_output("After processing, scanner state: %d, last_data_string remainder (len %d) - %s", scanner_state, last_data_string_length, last_data_string);

    return 0;
}

/// @brief Attempt to parse the packet(s) in the rx_buf
/// @param rx_buf The buffer to read from
/// @param rx_len The length to read
/// @param source FS_COMMUNICATION_MODE_SERIAL, FS_COMMUNICATION_MODE_TCP_UDP, etc. are handled differently
/// @return
int fs_device_parse_packet(fs_device_info *device_handle, const void *rx_buf, ssize_t rx_len, fs_communication_mode source)
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
        memset(&device_handle->packet_buf_queue[i], 0, sizeof(fs_packet_union));
    }

    switch (source)
    {
    case FS_COMMUNICATION_MODE_TCP_UDP:
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
    case FS_COMMUNICATION_MODE_SERIAL:
    {
        if (fs_device_process_packets_serial(device_handle, rx_buf, rx_len) != 0)
        {
            fs_log_output("[Trifecta] Error parsing packets!");
            return -1;
        }
        fs_log_output("[Trifecta] Successfully processed packets!");
    }
    break;
    case FS_COMMUNICATION_MODE_BLUETOOTH:
    case FS_COMMUNICATION_MODE_CAN:
    case FS_COMMUNICATION_MODE_I2C:
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
        memset(&device_handle->last_received_packet, 0, sizeof(fs_packet_union));
        memcpy(&device_handle->last_received_packet, &device_handle->packet_buf_queue[device_handle->packet_buf_queue_size - 1], sizeof(fs_packet_union));
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

int fs_get_last_timestamp(fs_device_info *device_handle, uint32_t *time)
{
    if (time == NULL)
    {
        return -1;
    }
    *time = device_handle->last_received_packet.composite.time;
    return 0;
}

int fs_get_raw_packet(fs_device_info *device_handle, fs_packet_union *packet_buffer)
{
    if (packet_buffer == NULL)
    {
        return -1;
    }

    memcpy(packet_buffer, &device_handle->last_received_packet, sizeof(fs_packet_union));
    return 0;
}

int fs_get_orientation(fs_device_info *device_handle, fs_quaternion *orientation_buffer)
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

int fs_get_orientation_euler(fs_device_info *device_handle, fs_vector3 *orientation_buffer, bool degrees)
{
    if (orientation_buffer == NULL)
    {
        return -1;
    }

    fs_quaternion quat = {0};
    if (fs_get_orientation(device_handle, &quat) != 0)
    {
        return -1;
    }

    fs_q_to_euler_angles(&orientation_buffer->x, &orientation_buffer->y, &orientation_buffer->z, quat.w, quat.x, quat.y, quat.z, degrees);
    return 0;
}

int fs_get_acceleration(fs_device_info *device_handle, fs_vector3 *acceleration_buffer)
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

int fs_get_angular_velocity(fs_device_info *device_handle, fs_vector3 *angular_velocity_buffer)
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

int fs_get_velocity(fs_device_info *device_handle, fs_vector3 *velocity_buffer)
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

int fs_get_movement_state(fs_device_info *device_handle, fs_run_status *device_state_buffer)
{
    if (device_state_buffer == NULL)
    {
        return -1;
    }
    *device_state_buffer = (device_handle->last_received_packet.composite.device_in_motion == 1) ? FS_RUN_STATUS_IDLE : FS_RUN_STATUS_RUNNING;
    return 0;
}

int fs_get_position(fs_device_info *device_handle, fs_vector3 *position_buffer)
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

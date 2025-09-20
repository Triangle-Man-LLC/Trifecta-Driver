
#include "FS_Trifecta_Defs.h"
#include "FS_Trifecta_Device_Utils.h"

// Base64 character set
static const char base64_chars[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

/// @brief Internal function to enqueue a command into the command queue...
/// @param str Command string
/// @param len Command length
/// @return 0 if successful
static int fs_enqueue_into_command_queue(fs_device_info_t *device_handle, const fs_command_info_t *cmd)
{
    if (!FS_RINGBUFFER_PUSH(&device_handle->command_queue, FS_MAX_CMD_QUEUE_LENGTH, cmd))
    {
        fs_log_output("[Trifecta] Device command queue was full!");
        return -1;
    }
    return 0;
}

/// @brief Parse all commands from a received string - used for TCP and serial command handlers
/// @param cmd_buf The buffer containing all commands
/// @param buf_len The length of the received buffer
/// @returns Error code (if any)
int fs_segment_commands(fs_device_info_t *device_handle, const void *cmd_buf, size_t buf_len)
{
    fs_command_info_t cmd = {0};
    unsigned int input_pos = 0;

    enum
    {
        WAITING,
        READING
    } state = WAITING;

    const char *input = (const char *)cmd_buf;

    for (size_t i = 0; i < buf_len; i++)
    {
        char inByte = input[i];

        if (inByte == FS_SERIAL_COMMAND_TERMINATOR)
        {
            cmd.payload[input_pos] = '\0';
            if (fs_enqueue_into_command_queue(device_handle, &cmd) != 0)
            {
                fs_log_output("[Trifecta] Error with device command segment/enqueue!");
                return -1;
            }
            input_pos = 0;
            memset(cmd.payload, 0, sizeof(cmd.payload));
            state = WAITING;
        }
        else if (inByte == '\n' || inByte == '\r' || inByte == '\0')
        {
            if (state == WAITING)
            {
                input_pos = 0;
                memset(cmd.payload, 0, sizeof(cmd.payload));
            }
            else
            {
                cmd.payload[input_pos] = '\0';
                input_pos = 0;
                memset(cmd.payload, 0, sizeof(cmd.payload));
                return 3; // Unexpected terminator mid-command
            }
        }
        else
        {
            state = READING;
            if (input_pos < (FS_MAX_CMD_LENGTH - 1))
            {
                cmd.payload[input_pos++] = inByte;
            }
            else
            {
                // Optional: truncate or log overflow
                fs_log_output("[Trifecta] Command too long, truncating.");
            }
        }
    }
    return 0;
}

static const int8_t base64_lookup[256] = {
    -1, -1, -1, -1, -1, -1, -1, -1, // 0–7
    -1, -1, -1, -1, -1, -1, -1, -1, // 8–15
    -1, -1, -1, -1, -1, -1, -1, -1, // 16–23
    -1, -1, -1, -1, -1, -1, -1, -1, // 24–31
    -1, -1, -1, -1, -1, -1, -1, -1, // 32–39
    -1, -1, -1, 62, -1, -1, -1, 63, // 40–47 ('+'=62, '/'=63)
    52, 53, 54, 55, 56, 57, 58, 59, // 48–55 ('0'–'7')
    60, 61, -1, -1, -1, -1, -1, -1, // 56–63 ('8','9')
    -1, 0, 1, 2, 3, 4, 5, 6,        // 64–71 ('A'–'G')
    7, 8, 9, 10, 11, 12, 13, 14,    // 72–79 ('H'–'O')
    15, 16, 17, 18, 19, 20, 21, 22, // 80–87 ('P'–'W')
    23, 24, 25, -1, -1, -1, -1, -1, // 88–95 ('X','Y','Z')
    -1, 26, 27, 28, 29, 30, 31, 32, // 96–103 ('a'–'g')
    33, 34, 35, 36, 37, 38, 39, 40, // 104–111 ('h'–'o')
    41, 42, 43, 44, 45, 46, 47, 48, // 112–119 ('p'–'w')
    49, 50, 51, -1, -1, -1, -1, -1, // 120–127 ('x','y','z')
    -1, -1, -1, -1, -1, -1, -1, -1, // 128–135
    -1, -1, -1, -1, -1, -1, -1, -1, // 136–143
    -1, -1, -1, -1, -1, -1, -1, -1, // 144–151
    -1, -1, -1, -1, -1, -1, -1, -1, // 152–159
    -1, -1, -1, -1, -1, -1, -1, -1, // 160–167
    -1, -1, -1, -1, -1, -1, -1, -1, // 168–175
    -1, -1, -1, -1, -1, -1, -1, -1, // 176–183
    -1, -1, -1, -1, -1, -1, -1, -1, // 184–191
    -1, -1, -1, -1, -1, -1, -1, -1, // 192–199
    -1, -1, -1, -1, -1, -1, -1, -1, // 200–207
    -1, -1, -1, -1, -1, -1, -1, -1, // 208–215
    -1, -1, -1, -1, -1, -1, -1, -1, // 216–223
    -1, -1, -1, -1, -1, -1, -1, -1, // 224–231
    -1, -1, -1, -1, -1, -1, -1, -1, // 232–239
    -1, -1, -1, -1, -1, -1, -1, -1, // 240–247
    -1, -1, -1, -1, -1, -1, -1, -1  // 248–255
};

// Helper function to map Base64 characters to values
static inline int base64_char_to_value(char c)
{
    return base64_lookup[(uint8_t)c];
}

// Base64 encode function
int fs_base64_encode(const void *data, size_t len, char *output_buffer, size_t buffer_size)
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
int fs_base64_decode(const char *input, void *output_buffer, size_t buffer_size, size_t *decoded_length)
{
    if (buffer_size > FS_MAX_DATA_LENGTH)
    {
        fs_log_output("[Trifecta] Base64 decode failed: Size of %d is greater than allowed %d!", buffer_size, FS_MAX_DATA_LENGTH);
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
        fs_log_output("[Trifecta] Base64 decode failed: Buffer size %d is too small for Base64 decoding. Required: %d", buffer_size, 3 * (input_length / 4) - padding);
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
            fs_log_output("[Trifecta] Base64 decode failed: Invalid Base64 character detected at index %u: 0x%X\n", i, input[i - 1]);
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
int obtain_packet_length(int packet_type)
{
    size_t packet_length = 0;
    switch (packet_type)
    {
    case C_PACKET_TYPE_IMU:
    case C_PACKET_TYPE_AHRS:
    case C_PACKET_TYPE_INS:
    case C_PACKET_TYPE_GNSS:
        packet_length = sizeof(fs_imu_composite_packet_t);
        break;
    case S_PACKET_TYPE_IMU:
    case S_PACKET_TYPE_AHRS:
    case S_PACKET_TYPE_INS:
    case S_PACKET_TYPE_GNSS:
        packet_length = sizeof(fs_imu_regular_packet_t);
        break;
    case C2_PACKET_TYPE_IMU:
    case C2_PACKET_TYPE_AHRS:
    case C2_PACKET_TYPE_INS:
    case C2_PACKET_TYPE_GNSS:
        packet_length = sizeof(fs_imu_composite_packet_2_t);
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
int fs_enqueue_into_packet_queue(fs_device_info_t *device_handle, const fs_packet_union_t *packet)
{
    if (!FS_RINGBUFFER_PUSH_FORCE(&device_handle->packet_buf_queue, FS_MAX_PACKET_QUEUE_LENGTH, packet))
    {
        fs_log_output("[Trifecta] Warning: Device packet queue was full! Packet dropped.");
        return -1;
    }
    // Update the last received packet...
    memcpy(&device_handle->last_received_packet, packet, sizeof(device_handle->last_received_packet));
    return 0;
}

/// @brief Parse the packet buffer and attempt to separate out the packets
/// @param rx_buf
/// @param rx_len
/// @return
int segment_packets(fs_device_info_t *device_handle, const void *rx_buf, size_t rx_len)
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
        fs_log_output("[Trifecta] Packet type %hhu, len: %ld", packet_type, packet_length);
        // Emplace the packet into the queue
        fs_enqueue_into_packet_queue(device_handle, (fs_packet_union_t *)(buf + pos));
        packet_count++;
        pos += packet_length;
    }
    return packet_count;
}

/// @brief Parse a Base64 string into the appropriate packet structure
/// @param segment The Base64 encoded string segment
/// @param length The length of the Base64 encoded string segment
/// @return Status code
int base64_to_packet(fs_device_info_t *device_handle, char *segment, size_t length)
{
    // Check for null pointer
    if (segment == NULL)
    {
        fs_log_output("[Trifecta] Base64 decode failed: Segment was NULL!");
        return -1;
    }

    fs_packet_union_t packet_union;
    memset(&packet_union, 0, sizeof(fs_packet_union_t));
    size_t decoded_size = 0;

    if (fs_base64_decode(segment, &packet_union, length, &decoded_size) != 0)
    {
        fs_log_output("[Trifecta] Base64 decode failed!");
        return -1;
    }

    if (fs_enqueue_into_packet_queue(device_handle, &packet_union) != 0)
    {
        fs_log_output("[Trifecta] Error: Could not place packet into packet queue!");
        return -1;
    }

    fs_log_output("[Trifecta] Scanned packet (B64)! Timestamp: %ld - Type: %d", packet_union.composite.time, packet_union.composite.type);
    return 0;
}

/// @brief Scan a buffer for delimiter positions.
/// @param buffer Pointer to the byte buffer
/// @param len Length of the buffer
/// @return Struct containing first index of each delimiter, or -1 if not found
fs_delimiter_indices_t fs_scan_delimiters(const uint8_t *buffer, size_t len)
{
    fs_delimiter_indices_t result = {-1, -1, -1, -1}; // Add binary_index
    int b64_candidate = -1;
    int bin_candidate = -1;

    for (size_t i = 0; i < len; i++)
    {
        if (buffer[i] == FS_SERIAL_PACKET_HEADER_B64)
        {
            b64_candidate = (int)i;
        }
        else if (buffer[i] == FS_SERIAL_PACKET_HEADER_BIN)
        {
            bin_candidate = (int)i;
        }
        else if (buffer[i] == FS_SERIAL_PACKET_FOOTER)
        {
            if (result.exclam_index == -1)
                result.exclam_index = (int)i;

            if (result.colon_index == -1 && b64_candidate != -1)
                result.colon_index = b64_candidate;

            if (result.binary_index == -1 && bin_candidate != -1)
                result.binary_index = bin_candidate;
        }
        else if (buffer[i] == FS_SERIAL_COMMAND_TERMINATOR && result.semicolon_index == -1)
        {
            result.semicolon_index = (int)i;
        }

        if (result.colon_index != -1 &&
            result.binary_index != -1 &&
            result.exclam_index != -1 &&
            result.semicolon_index != -1)
            break;
    }

    return result;
}

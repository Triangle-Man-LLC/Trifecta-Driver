
#include <stdlib.h>
#include <assert.h>
#include "FS_Trifecta.h"
#include "FS_Trifecta_Device.h"
#include "FS_Trifecta_Device_Utils.h"

int fs_logging_level = 1;

/// @brief Logs output with formatting.
/// @param format Format string.
/// @param ... Additional arguments.
/// @return Number of characters printed.

int fs_log_output(const char *format, ...)
{
    int chars_printed = 0;

    if (fs_logging_level <= 0)
    {
        return 0;
    }

    va_list args;
    va_start(args, format);

    // Print formatted string
    chars_printed = vprintf(format, args);

    // Flush stdout to ensure output is available
    fflush(stdout);

    // If last char wasn't newline, add one
    if (chars_printed > 0) {
        // Use fputc instead of indexing format
        if (ferror(stdout) == 0) {
            // Can't directly check last char printed, so safer approach:
            // Always append newline unless format already ends with '\n'
            size_t len = strlen(format);
            if (len == 0 || format[len - 1] != '\n') {
                putchar('\n');
                chars_printed++;
            }
        }
    }

    va_end(args);
    return chars_printed;
}

int fs_log_critical(const char *format, ...)
{
    int chars_printed = 0;

    va_list args;
    va_start(args, format);

    // Print formatted string
    chars_printed = vprintf(format, args);

    // Flush stdout to ensure output is available
    fflush(stdout);

    // If last char wasn't newline, add one
    if (chars_printed > 0) {
        // Use fputc instead of indexing format
        if (ferror(stdout) == 0) {
            // Can't directly check last char printed, so safer approach:
            // Always append newline unless format already ends with '\n'
            size_t len = strlen(format);
            if (len == 0 || format[len - 1] != '\n') {
                putchar('\n');
                chars_printed++;
            }
        }
    }

    va_end(args);
    return chars_printed;
}

int main()
{
    fs_device_info_t device = FS_DEVICE_INFO_UNINITIALIZED;

#define NUM_SAMPLES 256
    fs_imu_composite_packet_t packets[NUM_SAMPLES] = {{0}};
    // Populate all packets with monotonically increasing values...

    for (int i = 0; i < NUM_SAMPLES; i++)
    {
        fs_imu_composite_packet_t *p = &packets[i];

        p->type = i % 5;   // Cycle through packet types 0–4
        p->time = i * 100; // Simulated RTOS tick time

        float base = (float)i;

        // Raw IMU values
        p->ax0 = base + 0.1f;
        p->ay0 = base + 0.2f;
        p->az0 = base + 0.3f;
        p->gx0 = base + 0.4f;
        p->gy0 = base + 0.5f;
        p->gz0 = base + 0.6f;

        p->ax1 = base + 1.1f;
        p->ay1 = base + 1.2f;
        p->az1 = base + 1.3f;
        p->gx1 = base + 1.4f;
        p->gy1 = base + 1.5f;
        p->gz1 = base + 1.6f;

        p->ax2 = base + 2.1f;
        p->ay2 = base + 2.2f;
        p->az2 = base + 2.3f;
        p->gx2 = base + 2.4f;
        p->gy2 = base + 2.5f;
        p->gz2 = base + 2.6f;

        // Orientation quaternion
        p->q0 = 0.1f * i;
        p->q1 = 0.2f * i;
        p->q2 = 0.3f * i;
        p->q3 = 0.4f * i;

        // Absolute acceleration
        p->ax = base + 3.1f;
        p->ay = base + 3.2f;
        p->az = base + 3.3f;

        // Velocity
        p->vx = base + 4.1f;
        p->vy = base + 4.2f;
        p->vz = base + 4.3f;

        // Position
        p->rx = base + 5.1f;
        p->ry = base + 5.2f;
        p->rz = base + 5.3f;

        // Reserved
        p->reserved[0] = 0;
        p->reserved[1] = 0;
        p->reserved[2] = 0;

        // Motion status
        p->device_in_motion = (i % 2) + 1; // Alternate between 1 and 2
        p->label_2 = 0;

        // Temperatures
        p->temperature[0] = 25 + i % 3;
        p->temperature[1] = 26 + i % 3;
        p->temperature[2] = 27 + i % 3;

        // Reserved
        p->c = 0;
        p->d = i;
    }

    // Change the packets into BaseNUM_SAMPLES-encoded strings delimited by :%s!
    char formatted_packets[NUM_SAMPLES][FS_MAX_DATA_LENGTH] = {{0}};
    for (int i = 0; i < NUM_SAMPLES; i++)
    {
        char base64_buffer[FS_MAX_DATA_LENGTH] = {0};

        // Encode the raw packet into BaseNUM_SAMPLES
        int encoded_len = fs_base64_encode(&packets[i], sizeof(packets[i]),
                                           base64_buffer, sizeof(base64_buffer));
        if (encoded_len < 0)
        {
            printf("Encoding failed for packet %d\n", i);
            continue;
        }

        // Format with delimiter :%s!
        int written = snprintf(formatted_packets[i], FS_MAX_DATA_LENGTH, ":%s!", base64_buffer);
        if (written < 0 || written >= FS_MAX_DATA_LENGTH)
        {
            printf("Formatting failed or overflow for packet %d\n", i);
            continue;
        }
    }

    for (int i = 0; i < NUM_SAMPLES; i++)
    {
        const char *encoded_packet = formatted_packets[i];
        ssize_t encoded_len = strnlen(encoded_packet, FS_MAX_PACKET_LENGTH);

        // Simulate truncation for every 5th packet
        if (i % 5 == 0 && encoded_len > 10)
        {
            // Truncate to a random length between 10 and encoded_len - 1
            encoded_len = 10 + rand() % (encoded_len - 10);
            printf("Feeding truncated packet %d (len=%zd)\n", i, encoded_len);
        }

        // Step 1: Parse the packet (decoding is handled internally)
        if (fs_device_parse_packet(&device, encoded_packet, encoded_len, FS_COMMUNICATION_MODE_UART) < 0)
        {
            printf("Packet parse failed for packet %d\n", i);
            continue;
        }

        // Step 2: Retrieve the parsed packet
        fs_packet_union_t parsed = {{0}};
        if (fs_get_raw_packet(&device, &parsed) < 0)
        {
            printf("Failed to retrieve parsed packet %d\n", i);
            continue;
        }

        // Step 3: Compare with original
        if (memcmp(&parsed.composite, &packets[i], sizeof(fs_imu_composite_packet_t)) != 0)
        {
            printf("Mismatch in packet %d\n", i);
            assert(0 && "Parsed packet does not match original");
        }
        else
        {
            printf("Packet %d validated successfully\n", i);
        }
    }

    const char name_cmd[] = "ITrifecta-Test-Harness;";
    fs_handle_received_commands(&device, &name_cmd, sizeof(name_cmd));
    if (strcmp(device.device_descriptor.device_name, "Trifecta-Test-Harness") == 0)
    {
        printf("Device name correctly set to Trifecta-Test-Harness\n");
    }
    else
    {
        printf("Device name mismatch: got '%s'\n", device.device_descriptor.device_name);
    }

    return 0;
}
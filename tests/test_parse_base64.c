
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
    if (chars_printed > 0)
    {
        // Use fputc instead of indexing format
        if (ferror(stdout) == 0)
        {
            // Can't directly check last char printed, so safer approach:
            // Always append newline unless format already ends with '\n'
            size_t len = strlen(format);
            if (len == 0 || format[len - 1] != '\n')
            {
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
    if (chars_printed > 0)
    {
        // Use fputc instead of indexing format
        if (ferror(stdout) == 0)
        {
            // Can't directly check last char printed, so safer approach:
            // Always append newline unless format already ends with '\n'
            size_t len = strlen(format);
            if (len == 0 || format[len - 1] != '\n')
            {
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
    fs_imu_regular_packet_t regular_packets[NUM_SAMPLES] = {{0}};
    fs_imu_composite_packet_2_t composite2_packets[NUM_SAMPLES] = {{0}};

    // Populate all packets with monotonically increasing values...
    for (int i = 0; i < NUM_SAMPLES; i++)
    {
        fs_imu_composite_packet_t *p = &packets[i];

        p->type = C_PACKET_TYPE_INS;
        p->time = i * 100;

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

        // Quaternion
        p->q0 = 0.1f * i;
        p->q1 = 0.2f * i;
        p->q2 = 0.3f * i;
        p->q3 = 0.4f * i;

        // Magnetometer
        p->mag_x = base + 3.1f;
        p->mag_y = base + 3.2f;
        p->mag_z = base + 3.3f;

        // Compensated acceleration
        p->acc_x = base + 4.1f;
        p->acc_y = base + 4.2f;
        p->acc_z = base + 4.3f;

        // Compensated angular velocity
        p->omega_x0 = base + 5.1f;
        p->omega_y0 = base + 5.2f;
        p->omega_z0 = base + 5.3f;
        
        // Motion status
        p->device_motion_status = (i % 2) + 1;
        p->diagnostic_flag = 0;

        // Temperatures
        p->temperature[0] = 25 + (i % 3);
        p->temperature[1] = 26 + (i % 3);
        p->temperature[2] = 27 + (i % 3);

        p->c = 0;
        p->barometric_pressure = i;
    }

    for (int i = 0; i < NUM_SAMPLES; i++)
    {
        float base = (float)i;

        // -----------------------------
        // Populate fs_imu_regular_packet
        // -----------------------------
        fs_imu_regular_packet_t *r = &regular_packets[i];

        r->type = S_PACKET_TYPE_INS; // Arbitrary type for testing
        r->time = i * 50;

        r->omega_x0 = base + 0.11f;
        r->omega_y0 = base + 0.22f;
        r->omega_z0 = base + 0.33f;

        r->q0 = 0.01f * i;
        r->q1 = 0.02f * i;
        r->q2 = 0.03f * i;
        r->q3 = 0.04f * i;

        r->mag_x = base + 1.1f;
        r->mag_y = base + 1.2f;
        r->mag_z = base + 1.3f;

        r->acc_x = base + 2.1f;
        r->acc_y = base + 2.2f;
        r->acc_z = base + 2.3f;

        r->reserved_0_1 = 0;
        r->reserved_0_2 = 0;
        r->reserved_0_3 = 0;

        r->device_motion_status = (i % 3);
        r->diagnostic_flag = 0;

        r->temperature[0] = 20 + (i % 3);
        r->temperature[1] = 21 + (i % 3);
        r->temperature[2] = 22 + (i % 3);

        r->c = 0;
        r->barometric_pressure = i;

        // -------------------------------------
        // Populate fs_imu_composite_packet_2
        // -------------------------------------
        fs_imu_composite_packet_2_t *c2 = &composite2_packets[i];

        c2->type = C2_PACKET_TYPE_INS;
        c2->time = i * 75;

        // Raw IMU values
        c2->ax0 = base + 0.1f;
        c2->ay0 = base + 0.2f;
        c2->az0 = base + 0.3f;
        c2->gx0 = base + 0.4f;
        c2->gy0 = base + 0.5f;
        c2->gz0 = base + 0.6f;

        c2->ax1 = base + 1.1f;
        c2->ay1 = base + 1.2f;
        c2->az1 = base + 1.3f;
        c2->gx1 = base + 1.4f;
        c2->gy1 = base + 1.5f;
        c2->gz1 = base + 1.6f;

        c2->ax2 = base + 2.1f;
        c2->ay2 = base + 2.2f;
        c2->az2 = base + 2.3f;
        c2->gx2 = base + 2.4f;
        c2->gy2 = base + 2.5f;
        c2->gz2 = base + 2.6f;

        // Orientation
        c2->q0 = 0.1f * i;
        c2->q1 = 0.2f * i;
        c2->q2 = 0.3f * i;
        c2->q3 = 0.4f * i;

        // Magnetometer
        c2->mag_x = base + 3.1f;
        c2->mag_y = base + 3.2f;
        c2->mag_z = base + 3.3f;

        // Compensated gyro
        c2->omega_x0 = base + 4.1f;
        c2->omega_y0 = base + 4.2f;
        c2->omega_z0 = base + 4.3f;

        // Compensated accel
        c2->acc_x = base + 5.1f;
        c2->acc_y = base + 5.2f;
        c2->acc_z = base + 5.3f;

        // Velocity
        c2->vx = base + 6.1f;
        c2->vy = base + 6.2f;
        c2->vz = base + 6.3f;

        // Position (double)
        c2->rx = base + 7.1;
        c2->ry = base + 7.2;
        c2->rz = base + 7.3;

        c2->device_motion_status = (i % 3);
        c2->diagnostic_flag = 0;

        c2->temperature[0] = 30 + (i % 3);
        c2->temperature[1] = 31 + (i % 3);
        c2->temperature[2] = 32 + (i % 3);

        c2->c = 0;
        c2->barometric_pressure = i;
    }

    // Change the packets into BaseNUM_SAMPLES-encoded strings delimited by :%s!
    char formatted_packets[NUM_SAMPLES][FS_MAX_DATA_LENGTH] = {{0}};
    char formatted_regular[NUM_SAMPLES][FS_MAX_DATA_LENGTH] = {{0}};
    char formatted_composite2[NUM_SAMPLES][FS_MAX_DATA_LENGTH] = {{0}};
    for (int i = 0; i < NUM_SAMPLES; i++)
    {
        char base64_buffer[FS_MAX_DATA_LENGTH] = {0};

        // Encode the raw packet into BaseNUM_SAMPLES
        int encoded_len = fs_base64_encode(&packets[i], sizeof(packets[i]),
                                           base64_buffer, sizeof(base64_buffer));
        // Format with delimiter :%s!
        int written = snprintf(formatted_packets[i], FS_MAX_DATA_LENGTH, ":%s!", base64_buffer);
    }

    for (int i = 0; i < NUM_SAMPLES; i++)
    {
        char buf[FS_MAX_DATA_LENGTH] = {0};

        int len = fs_base64_encode(&regular_packets[i], sizeof(regular_packets[i]),
                                   buf, sizeof(buf));
        snprintf(formatted_regular[i], FS_MAX_DATA_LENGTH, ":%s!", buf);

        int len2 = fs_base64_encode(&composite2_packets[i], sizeof(composite2_packets[i]),
                               buf, sizeof(buf));
        snprintf(formatted_composite2[i], FS_MAX_DATA_LENGTH, ":%s!", buf);
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

    // -----------------------------
    // Validate fs_imu_regular_packet
    // -----------------------------
    for (int i = 0; i < NUM_SAMPLES; i++)
    {
        const char *encoded = formatted_regular[i];
        ssize_t len = strnlen(encoded, FS_MAX_PACKET_LENGTH);

        if (fs_device_parse_packet(&device, encoded, len, FS_COMMUNICATION_MODE_UART) < 0)
        {
            printf("Regular packet parse failed %d\n", i);
            continue;
        }

        fs_packet_union_t parsed = {{0}};
        fs_get_raw_packet(&device, &parsed);

        if (memcmp(&parsed.regular, &regular_packets[i], sizeof(fs_imu_regular_packet_t)) != 0)
        {
            printf("Mismatch in regular packet %d\n", i);
            assert(0);
        }
    }

    // -------------------------------------
    // Validate fs_imu_composite_packet_2
    // -------------------------------------
    for (int i = 0; i < NUM_SAMPLES; i++)
    {
        const char *encoded = formatted_composite2[i];
        ssize_t len = strnlen(encoded, FS_MAX_PACKET_LENGTH);

        if (fs_device_parse_packet(&device, encoded, len, FS_COMMUNICATION_MODE_UART) < 0)
        {
            printf("Composite2 packet parse failed %d\n", i);
            continue;
        }

        fs_packet_union_t parsed = {{0}};
        fs_get_raw_packet(&device, &parsed);

        if (memcmp(&parsed.composite2, &composite2_packets[i], sizeof(fs_imu_composite_packet_2_t)) != 0)
        {
            printf("Mismatch in composite2 packet %d\n", i);
            assert(0);
        }
    }

    return 0;
}
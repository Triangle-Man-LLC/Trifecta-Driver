
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
    fs_packet_union_t packets[NUM_SAMPLES] = {{0}};

    // Populate all 3 packet types
    for (int i = 0; i < NUM_SAMPLES; i++)
    {
        float base = (float)i;

        // Composite packet (145 bytes)
        fs_imu_composite_packet_t *c = &packets[i].composite;

        c->type = C_PACKET_TYPE_INS;
        c->time = i * 100;

        c->ax0 = base + 0.1f;
        c->ay0 = base + 0.2f;
        c->az0 = base + 0.3f;
        c->gx0 = base + 0.4f;
        c->gy0 = base + 0.5f;
        c->gz0 = base + 0.6f;

        c->ax1 = base + 1.1f;
        c->ay1 = base + 1.2f;
        c->az1 = base + 1.3f;
        c->gx1 = base + 1.4f;
        c->gy1 = base + 1.5f;
        c->gz1 = base + 1.6f;

        c->ax2 = base + 2.1f;
        c->ay2 = base + 2.2f;
        c->az2 = base + 2.3f;
        c->gx2 = base + 2.4f;
        c->gy2 = base + 2.5f;
        c->gz2 = base + 2.6f;

        c->q0 = 0.1f * i;
        c->q1 = 0.2f * i;
        c->q2 = 0.3f * i;
        c->q3 = 0.4f * i;

        c->mag_x = base + 3.1f;
        c->mag_y = base + 3.2f;
        c->mag_z = base + 3.3f;

        c->acc_x = base + 4.1f;
        c->acc_y = base + 4.2f;
        c->acc_z = base + 4.3f;

        c->omega_x0 = base + 5.1f;
        c->omega_y0 = base + 5.2f;
        c->omega_z0 = base + 5.3f;

        c->device_motion_status = (i % 2) + 1;
        c->label_2 = 0;

        c->temperature[0] = 25 + (i % 3);
        c->temperature[1] = 26 + (i % 3);
        c->temperature[2] = 27 + (i % 3);

        c->c = 0;
        c->barometric_pressure = i;

        // Regular packet (85 bytes)
        fs_imu_regular_packet_t *r = &packets[i].regular;

        r->type = S_PACKET_TYPE_INS;
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
        r->label_2 = 0;

        r->temperature[0] = 20 + (i % 3);
        r->temperature[1] = 21 + (i % 3);
        r->temperature[2] = 22 + (i % 3);

        r->c = 0;
        r->barometric_pressure = i;

        // Composite2 packet (181 bytes)
        fs_imu_composite_packet_2_t *c2 = &packets[i].composite2;

        c2->type = C2_PACKET_TYPE_INS;
        c2->time = i * 75;

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

        c2->q0 = 0.1f * i;
        c2->q1 = 0.2f * i;
        c2->q2 = 0.3f * i;
        c2->q3 = 0.4f * i;

        c2->mag_x = base + 3.1f;
        c2->mag_y = base + 3.2f;
        c2->mag_z = base + 3.3f;

        c2->omega_x0 = base + 4.1f;
        c2->omega_y0 = base + 4.2f;
        c2->omega_z0 = base + 4.3f;

        c2->acc_x = base + 5.1f;
        c2->acc_y = base + 5.2f;
        c2->acc_z = base + 5.3f;

        c2->vx = base + 6.1f;
        c2->vy = base + 6.2f;
        c2->vz = base + 6.3f;

        c2->rx = base + 7.1;
        c2->ry = base + 7.2;
        c2->rz = base + 7.3;

        c2->device_motion_status = (i % 3);
        c2->label_2 = 0;

        c2->temperature[0] = 30 + (i % 3);
        c2->temperature[1] = 31 + (i % 3);
        c2->temperature[2] = 32 + (i % 3);

        c2->c = 0;
        c2->barometric_pressure = i;
    }

    // Validate all 3 packet types
    for (int i = 0; i < NUM_SAMPLES; i++)
    {
        if (fs_device_parse_packet(&device,
                                   &packets[i],
                                   sizeof(fs_packet_union_t),
                                   FS_COMMUNICATION_MODE_TCP_UDP) < 0)
        {
            printf("Packet parse failed for packet %d\n", i);
            continue;
        }

        fs_packet_union_t parsed = {{0}};
        if (fs_get_raw_packet(&device, &parsed) < 0)
        {
            printf("Failed to retrieve parsed packet %d\n", i);
            continue;
        }

        //
        // Type‑aware validation
        //
        switch (parsed.composite.type)
        {
        case C_PACKET_TYPE_IMU:
        case C_PACKET_TYPE_AHRS:
        case C_PACKET_TYPE_INS:
        case C_PACKET_TYPE_RESERVED:
            if (memcmp(&parsed.composite,
                       &packets[i].composite,
                       sizeof(fs_imu_composite_packet_t)) != 0)
            {
                printf("Composite mismatch at %d\n", i);
                assert(0);
            }
            break;

        case S_PACKET_TYPE_IMU:
        case S_PACKET_TYPE_AHRS:
        case S_PACKET_TYPE_INS:
        case S_PACKET_TYPE_RESERVED:
            if (memcmp(&parsed.regular,
                       &packets[i].regular,
                       sizeof(fs_imu_regular_packet_t)) != 0)
            {
                printf("Regular mismatch at %d\n", i);
                assert(0);
            }
            break;

        case C2_PACKET_TYPE_IMU:
        case C2_PACKET_TYPE_AHRS:
        case C2_PACKET_TYPE_INS:
        case C2_PACKET_TYPE_RESERVED:
            if (memcmp(&parsed.composite2,
                       &packets[i].composite2,
                       sizeof(fs_imu_composite_packet_2_t)) != 0)
            {
                printf("Composite2 mismatch at %d\n", i);
                assert(0);
            }
            break;

        default:
            printf("Unknown packet type %d at index %d\n",
                   parsed.composite.type, i);
            assert(0);
        }

        printf("Packet %d validated successfully\n", i);
    }

    return 0;
}

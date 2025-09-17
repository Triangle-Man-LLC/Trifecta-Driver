#include <stdlib.h>
#include <assert.h>
#include "FS_Trifecta.h"
#include "FS_Trifecta_Device.h"
#include "FS_Trifecta_Device_Utils.h"

int fs_logging_level = 1;

int fs_log_output(const char *format, ...)
{
    int chars_printed = 0;

    if (fs_logging_level > 0)
    {
        va_list args;
        va_start(args, format);
        chars_printed = vprintf(format, args);
        if (format[chars_printed - 1] != '\n')
        {
            printf("\n");
            chars_printed++;
        }
        va_end(args);
    }

    return chars_printed;
}

int main()
{
    fs_device_info_t device = FS_DEVICE_INFO_UNINITIALIZED;
#define NUM_SAMPLES 64
    fs_imu_composite_packet_t packets[NUM_SAMPLES] = {{0}};

    for (int i = 0; i < NUM_SAMPLES; i++)
    {
        fs_imu_composite_packet_t *p = &packets[i];
        p->type = i % 5;
        p->time = i * 100;
        float base = (float)i;

        p->ax0 = base + 0.1f; p->ay0 = base + 0.2f; p->az0 = base + 0.3f;
        p->gx0 = base + 0.4f; p->gy0 = base + 0.5f; p->gz0 = base + 0.6f;
        p->ax1 = base + 1.1f; p->ay1 = base + 1.2f; p->az1 = base + 1.3f;
        p->gx1 = base + 1.4f; p->gy1 = base + 1.5f; p->gz1 = base + 1.6f;
        p->ax2 = base + 2.1f; p->ay2 = base + 2.2f; p->az2 = base + 2.3f;
        p->gx2 = base + 2.4f; p->gy2 = base + 2.5f; p->gz2 = base + 2.6f;
        p->q0 = 0.1f * i; p->q1 = 0.2f * i; p->q2 = 0.3f * i; p->q3 = 0.4f * i;
        p->ax = base + 3.1f; p->ay = base + 3.2f; p->az = base + 3.3f;
        p->vx = base + 4.1f; p->vy = base + 4.2f; p->vz = base + 4.3f;
        p->rx = base + 5.1f; p->ry = base + 5.2f; p->rz = base + 5.3f;
        p->reserved[0] = 0; p->reserved[1] = 0; p->reserved[2] = 0;
        p->device_in_motion = (i % 2) + 1;
        p->label_2 = 0;
        p->temperature[0] = 25 + i % 3;
        p->temperature[1] = 26 + i % 3;
        p->temperature[2] = 27 + i % 3;
        p->c = 0;
        p->d = i;
    }

    // Format packets as binary: ?<raw><footer>
    char formatted_packets[NUM_SAMPLES][FS_MAX_DATA_LENGTH] = {{0}};
    for (int i = 0; i < NUM_SAMPLES; i++)
    {
        size_t raw_len = sizeof(fs_imu_composite_packet_t);
        if (raw_len + 2 > FS_MAX_DATA_LENGTH)
        {
            printf("Packet too large for buffer at index %d\n", i);
            continue;
        }

        formatted_packets[i][0] = FS_SERIAL_PACKET_HEADER_BIN;
        memcpy(&formatted_packets[i][1], &packets[i], raw_len);
        formatted_packets[i][1 + raw_len] = FS_SERIAL_PACKET_FOOTER;
    }

    for (int i = 0; i < NUM_SAMPLES; i++)
    {
        const uint8_t *packet_buf = (const uint8_t *)formatted_packets[i];
        size_t packet_len = sizeof(fs_imu_composite_packet_t) + 2;

        // Simulate truncation for every 7th packet
        if (i % 7 == 0 && packet_len > 10)
        {
            packet_len = 10 + rand() % (packet_len - 10);
            printf("Feeding truncated binary packet %d (len=%zu)\n", i, packet_len);
        }

        // Step 1: Parse the packet (decoding is handled internally)
        if (fs_device_parse_packet(&device, packet_buf, packet_len, FS_COMMUNICATION_MODE_UART) < 0)
        {
            printf("Packet parse failed for packet %d\n", i);
            continue;
        }

        // Step 2: Retrieve parsed packet
        fs_packet_union_t parsed = {{0}};
        if (fs_get_raw_packet(&device, &parsed) < 0)
        {
            printf("Failed to retrieve parsed binary packet %d\n", i);
            continue;
        }

        // Step 3: Compare with original
        if (memcmp(&parsed.composite, &packets[i], sizeof(fs_imu_composite_packet_t)) != 0)
        {
            printf("Mismatch in binary packet %d\n", i);
            assert(0 && "Parsed binary packet does not match original");
        }
        else
        {
            printf("Binary packet %d validated successfully\n", i);
        }
    }

    return 0;
}

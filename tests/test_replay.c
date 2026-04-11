#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "FS_Trifecta_Replay.h"
#include "FS_Trifecta_Defs.h"
#include "FS_Trifecta_Interfaces.h"

// Utility: float comparison
static int nearly_equal(float a, float b, float eps)
{
    return fabsf(a - b) < eps;
}

static int test_new_format_row(const fs_imu_composite_packet_2_t *p)
{
#define CHECKF(field, expected, eps, code)                                                                \
    if (!nearly_equal((field), (expected), (eps)))                                                        \
    {                                                                                                     \
        printf("FAIL NEW: %s expected %.9f but got %.9f\n", #field, (double)(expected), (double)(field)); \
        return (code);                                                                                    \
    }

#define CHECKD(field, expected, eps, code)                                                                \
    if (fabs((field) - (expected)) > (eps))                                                               \
    {                                                                                                     \
        printf("FAIL NEW: %s expected %.9f but got %.9f\n", #field, (double)(expected), (double)(field)); \
        return (code);                                                                                    \
    }

    if (p->type != 10)
    {
        printf("FAIL NEW: type expected 10 but got %d\n", p->type);
        return -1;
    }
    if (p->time != 29960154)
    {
        printf("FAIL NEW: time expected 29960154 but got %u\n", p->time);
        return -2;
    }

    CHECKF(p->ax0, 136.5269f, 1e-4f, -3);
    CHECKF(p->ay0, 94.19359f, 1e-4f, -4);
    CHECKF(p->az0, -8173.667f, 1e-3f, -5);

    CHECKF(p->gx0, -1.01735f, 1e-5f, -6);
    CHECKF(p->gy0, 3.64045f, 1e-5f, -7);
    CHECKF(p->gz0, -2.764529f, 1e-5f, -8);

    CHECKF(p->ax1, 136.5269f, 1e-4f, -9);
    CHECKF(p->ay1, 94.19359f, 1e-4f, -10);
    CHECKF(p->az1, -8173.667f, 1e-3f, -11);

    CHECKF(p->gx1, -1.01735f, 1e-5f, -12);
    CHECKF(p->gy1, 3.64045f, 1e-5f, -13);
    CHECKF(p->gz1, -2.764529f, 1e-5f, -14);

    CHECKF(p->ax2, 136.5269f, 1e-4f, -15);
    CHECKF(p->ay2, 94.19359f, 1e-4f, -16);
    CHECKF(p->az2, -8173.667f, 1e-3f, -17);

    CHECKF(p->gx2, -1.01735f, 1e-5f, -18);
    CHECKF(p->gy2, 3.64045f, 1e-5f, -19);
    CHECKF(p->gz2, -2.764529f, 1e-5f, -20);

    CHECKF(p->q0, 0.9485377f, 1e-6f, -21);
    CHECKF(p->q1, -0.007051024f, 1e-6f, -22);
    CHECKF(p->q2, 0.006401631f, 1e-6f, -23);
    CHECKF(p->q3, 0.316521f, 1e-6f, -24);

    CHECKF(p->omega_x0, -1.01735f, 1e-5f, -28);
    CHECKF(p->omega_y0, 3.64045f, 1e-5f, -29);
    CHECKF(p->omega_z0, -2.764529f, 1e-5f, -30);

    CHECKF(p->acc_x, -136.5269f, 1e-4f, -31);
    CHECKF(p->acc_y, -94.19359f, 1e-4f, -32);
    CHECKF(p->acc_z, 8173.667f, 1e-3f, -33);

    CHECKF(p->vx, 0.001153941f, 1e-6f, -34);
    CHECKF(p->vy, -0.002136487f, 1e-6f, -35);
    CHECKF(p->vz, 0.0f, 1e-6f, -36);

    CHECKD(p->rx, 0, 1e-6, -37);
    CHECKD(p->ry, 0, 1e-6, -38);
    CHECKD(p->rz, 0, 1e-6, -39);

    CHECKF(p->reserved[0], 0.0f, 1e-6f, -40);
    CHECKF(p->reserved[1], 0.0f, 1e-6f, -41);
    CHECKF(p->reserved[2], 0.0f, 1e-6f, -42);

    if (p->device_motion_status != 1)
    {
        printf("FAIL NEW: device_motion_status expected 1 but got %d\n", p->device_motion_status);
        return -43;
    }

    return 0;
}
static int test_old_format_row(const fs_imu_composite_packet_t *p)
{
#define CHECKF_OLD(field, expected, eps, code)                                                            \
    if (!nearly_equal((field), (expected), (eps)))                                                        \
    {                                                                                                     \
        printf("FAIL OLD: %s expected %.9f but got %.9f\n", #field, (double)(expected), (double)(field)); \
        return (code);                                                                                    \
    }

    if (p->type != 2)
    {
        printf("FAIL OLD: type expected 2 but got %d\n", p->type);
        return -1;
    }
    if (p->time != 9667)
    {
        printf("FAIL OLD: time expected 9667 but got %u\n", p->time);
        return -2;
    }

    CHECKF_OLD(p->ax0, -8.0f, 1e-6f, -3);
    CHECKF_OLD(p->ay0, -1969.0f, 1e-6f, -4);
    CHECKF_OLD(p->az0, -7907.0f, 1e-6f, -5);

    CHECKF_OLD(p->gx0, -2.659372f, 1e-6f, -6);
    CHECKF_OLD(p->gy0, -39.415825f, 1e-6f, -7);
    CHECKF_OLD(p->gz0, 13.495514f, 1e-6f, -8);

    CHECKF_OLD(p->ax1, 47.0f, 1e-6f, -9);
    CHECKF_OLD(p->ay1, -1922.0f, 1e-6f, -10);
    CHECKF_OLD(p->az1, -7989.0f, 1e-6f, -11);

    CHECKF_OLD(p->gx1, -22.011223f, 1e-6f, -12);
    CHECKF_OLD(p->gy1, 16.945568f, 1e-6f, -13);
    CHECKF_OLD(p->gz1, -21.099888f, 1e-6f, -14);

    CHECKF_OLD(p->ax2, 2.12132f, 1e-6f, -15);
    CHECKF_OLD(p->ay2, -1924.037476f, 1e-6f, -16);
    CHECKF_OLD(p->az2, -8021.0f, 1e-6f, -17);

    CHECKF_OLD(p->gx2, -16.816231f, 1e-6f, -18);
    CHECKF_OLD(p->gy2, -19.963673f, 1e-6f, -19);
    CHECKF_OLD(p->gz2, 11.235128f, 1e-6f, -20);

    CHECKF_OLD(p->q0, 0.999926f, 1e-6f, -21);
    CHECKF_OLD(p->q1, 0.012147f, 1e-6f, -22);
    CHECKF_OLD(p->q2, 0.000766f, 1e-6f, -23);
    CHECKF_OLD(p->q3, -0.000012f, 1e-6f, -24);

    if (p->device_motion_status != 0)
    {
        printf("FAIL OLD: device_motion_status expected 0 but got %d\n", p->device_motion_status);
        return -31;
    }

    return 0;
}

// Test a single file (OLD or NEW)
static int test_file(const char *path, int is_new_format)
{
    printf("=== Testing %s ===\n", path);

    fs_replay_t r;
    fs_packet_union_t pkt;

    if (fs_replay_open(&r, path, 200) != 0)
    {
        printf("ERROR: open failed\n");
        fs_replay_close(&r);
        return -1;
    }

    printf("  lines=%u blocks=%u\n", r.num_lines, r.num_blocks);

    // ---- Test 1: read first line ----
    if (fs_replay_read_line(&r, 0, &pkt) != 0)
    {
        printf("ERROR: read_line(0) failed\n");
        fs_replay_close(&r);
        return -2;
    }

    if (is_new_format)
    {
        int rc = test_new_format_row(&pkt.composite2);
        if (rc != 0)
        {
            printf("ERROR: NEW_FORMAT row mismatch (code=%d)\n", rc);
            fs_replay_close(&r);
            return -3;
        }
        printf("  NEW_FORMAT row 0 OK\n");
    }
    else
    {
        int rc = test_old_format_row(&pkt.composite);
        if (rc != 0)
        {
            printf("ERROR: OLD_FORMAT row mismatch (code=%d)\n", rc);
            fs_replay_close(&r);
            return -4;
        }
        printf("  OLD_FORMAT row 0 OK\n");
    }

    // ---- Test 2: read middle line ----
    uint32_t mid = r.num_lines / 2;
    if (fs_replay_read_line(&r, mid, &pkt) != 0)
    {
        printf("ERROR: read_line(mid) failed\n");
        fs_replay_close(&r);
        return -5;
    }

    // ---- Test 3: read last line ----
    uint32_t last = r.num_lines - 1;
    if (fs_replay_read_line(&r, last, &pkt) != 0)
    {
        printf("ERROR: read_line(last) failed\n");
        fs_replay_close(&r);
        return -6;
    }

    // ---- Test 4: sequential read ----
    fseek(r.fp, (long)r.offsets[0], SEEK_SET);
    int seq_count = 0;

    while (1)
    {
        int rc = fs_replay_read_next(&r, &pkt);
        if (rc == 1)
            break; // EOF
        if (rc != 0)
        {
            printf("ERROR: sequential read failed\n");
            fs_replay_close(&r);
            return -7;
        }
        seq_count++;
    }

    printf("  Sequential read OK (%d packets)\n", seq_count);

    fs_replay_close(&r);
    return 0;
}

int main(void)
{
    int rc1 = test_file("test_resources/OLD_FORMAT.csv", 0);
    int rc2 = test_file("test_resources/NEW_FORMAT.csv", 1);

    if (rc1 != 0 || rc2 != 0)
    {
        printf("Replay tests FAILED\n");
        return 1;
    }

    printf("Replay tests PASSED\n");
    return 0;
}

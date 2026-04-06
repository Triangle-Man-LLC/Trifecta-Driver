/// Driver for the Trifecta series of IMU/AHRS/INS devices
/// Copyright 2026 4rge.ai and/or Triangle Man LLC
/// Usage and redistribution of this code is permitted
/// but this notice must be retained in all copies of the code.

/// THIS SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
/// INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE,
/// AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
/// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include "FS_Trifecta_Replay.h"

#include "FS_Trifecta_Replay.h"

static char linebuf[512] = {0};

static int fs_strcasecmp(const char *a, const char *b)
{
    unsigned char ca, cb;

    while (*a && *b)
    {
        ca = (unsigned char)*a++;
        cb = (unsigned char)*b++;

        ca = (unsigned char)tolower(ca);
        cb = (unsigned char)tolower(cb);

        if (ca != cb)
            return (int)ca - (int)cb;
    }

    return (int)(unsigned char)tolower(*a) -
           (int)(unsigned char)tolower(*b);
}

static void fs_normalize_token(const char *in, char *out, size_t out_sz)
{
    while (*in && isspace((unsigned char)*in))
        in++;

    size_t len = 0;
    size_t last_non_space = 0;
    while (*in && *in != ',' && len + 1 < out_sz)
    {
        unsigned char c = (unsigned char)*in++;
        if (!isspace(c))
            last_non_space = len + 1;
        out[len++] = (char)tolower(c);
    }
    out[last_non_space] = '\0';
}

typedef struct
{
    const char *name;
    fs_field_id_t id;
} header_map_t;

static const header_map_t header_map[] = {
    {"type", FIELD_TYPE},
    {"packettype", FIELD_TYPE},
    {"time", FIELD_TIME},

    {"ax0", FIELD_AX0},
    {"ay0", FIELD_AY0},
    {"az0", FIELD_AZ0},
    {"gx0", FIELD_GX0},
    {"gy0", FIELD_GY0},
    {"gz0", FIELD_GZ0},

    {"ax1", FIELD_AX1},
    {"ay1", FIELD_AY1},
    {"az1", FIELD_AZ1},
    {"gx1", FIELD_GX1},
    {"gy1", FIELD_GY1},
    {"gz1", FIELD_GZ1},

    {"ax2", FIELD_AX2},
    {"ay2", FIELD_AY2},
    {"az2", FIELD_AZ2},
    {"gx2", FIELD_GX2},
    {"gy2", FIELD_GY2},
    {"gz2", FIELD_GZ2},

    {"q0", FIELD_Q0},
    {"q1", FIELD_Q1},
    {"q2", FIELD_Q2},
    {"q3", FIELD_Q3},

    {"omegax0", FIELD_OMEGAX0},
    {"omegay0", FIELD_OMEGY0},
    {"omegaz0", FIELD_OMEGAZ0},
    {"omegax1", FIELD_OMEGAX1},
    {"omegay1", FIELD_OMEGY1},
    {"omegaz1", FIELD_OMEGAZ1},

    {"accx", FIELD_ACCX},
    {"accy", FIELD_ACCY},
    {"accz", FIELD_ACCZ},

    {"vx", FIELD_VX},
    {"vy", FIELD_VY},
    {"vz", FIELD_VZ},
    {"rx", FIELD_RX},
    {"ry", FIELD_RY},
    {"rz", FIELD_RZ},

    {"reserved0_1", FIELD_RESERVED0_1},
    {"reserved0_2", FIELD_RESERVED0_2},
    {"reserved0_3", FIELD_RESERVED0_3},

    {"magx", FIELD_MAGX},
    {"magy", FIELD_MAGY},
    {"magz", FIELD_MAGZ},
    {"mag_x", FIELD_MAGX},
    {"mag_y", FIELD_MAGY},
    {"mag_z", FIELD_MAGZ},

    {"temp0", FIELD_TEMP0},
    {"temp1", FIELD_TEMP1},
    {"temp2", FIELD_TEMP2},

    {"diagnosticflag", FIELD_DIAG},

    {"barometricpressure", FIELD_BARO},
    {"baro", FIELD_BARO},

    {"devicemotionstatus", FIELD_DEVICEMOTION},
    {"label", FIELD_DEVICEMOTION},
};

int fs_replay_parse_header(fs_replay_t *r, const char *line)
{
    if (!r || !line)
        return -1;

    for (int i = 0; i < FS_REPLAY_MAX_COLUMNS; ++i)
        r->field_for_column[i] = FIELD_NONE;

    char tmp[512];
    strncpy(tmp, line, sizeof(tmp));
    tmp[sizeof(tmp) - 1] = '\0';

    int col = 0;
    const char *p = tmp;

    while (col < FS_REPLAY_MAX_COLUMNS)
    {
        char *comma = strchr(p, ',');

        char field[64];
        if (comma)
        {
            size_t len = (size_t)(comma - p);
            if (len >= sizeof(field))
                len = sizeof(field) - 1;
            memcpy(field, p, len);
            field[len] = '\0';
        }
        else
        {
            strncpy(field, p, sizeof(field));
            field[sizeof(field) - 1] = '\0';
        }

        char norm[64];
        fs_normalize_token(field, norm, sizeof(norm));

        /* match against header_map */
        size_t map_count = sizeof(header_map) / sizeof(header_map[0]);
        for (size_t i = 0; i < map_count; ++i)
        {
            if (!strcmp(norm, header_map[i].name))
            {
                r->field_for_column[col] = header_map[i].id;
                break;
            }
        }

        if (!comma)
            break;

        p = comma + 1;  // may point to empty field
        col++;
    }

    r->num_lines = 0;
    return 0;
}

int fs_replay_parse_packet(fs_replay_t *r,
                           const char *line,
                           fs_packet_union_t *out)
{
    if (!r || !line || !out)
        return -1;

    const char *tokens[FS_REPLAY_MAX_COLUMNS];
    int count = 0;

    char tmp[512];
    strncpy(tmp, line, sizeof(tmp));
    tmp[sizeof(tmp) - 1] = '\0';

    char *ca = tmp;

    while (count < FS_REPLAY_MAX_COLUMNS)
    {
        tokens[count] = ca;

        char *comma = strchr(p, ',');
        if (!comma)
        {
            count++;
            break;
        }

        *comma = '\0';      // terminate token
        ca = comma + 1;      // next token (may be empty)
        count++;
    }

    // Determine packet type from whatever column is mapped to FIELD_TYPE
    int type_i = 0;
    for (int col = 0; col < count; ++col)
    {
        if (r->field_for_column[col] == FIELD_TYPE)
        {
            type_i = atoi(tokens[col]);
            break;
        }
    }

    fs_packet_type_t ptype = (fs_packet_type_t)type_i;

    switch (ptype)
    {
    // Standard composite (no velocity/position)
    case C_PACKET_TYPE_IMU:
    case C_PACKET_TYPE_AHRS:
    case C_PACKET_TYPE_INS:
    {
        fs_imu_composite_packet_t *p = &out->composite;
        memset(p, 0, sizeof(*p));
        p->type = (uint8_t)type_i;

        for (int col = 0; col < count; ++col)
        {
            fs_field_id_t f = (fs_field_id_t)r->field_for_column[col];
            const char *s = tokens[col];

            switch (f)
            {
            case FIELD_TIME:
                p->time = (uint32_t)strtoul(s, NULL, 10);
                break;

            case FIELD_AX0:
                p->ax0 = strtof(s, NULL);
                break;
            case FIELD_AY0:
                p->ay0 = strtof(s, NULL);
                break;
            case FIELD_AZ0:
                p->az0 = strtof(s, NULL);
                break;
            case FIELD_GX0:
                p->gx0 = strtof(s, NULL);
                break;
            case FIELD_GY0:
                p->gy0 = strtof(s, NULL);
                break;
            case FIELD_GZ0:
                p->gz0 = strtof(s, NULL);
                break;

            case FIELD_AX1:
                p->ax1 = strtof(s, NULL);
                break;
            case FIELD_AY1:
                p->ay1 = strtof(s, NULL);
                break;
            case FIELD_AZ1:
                p->az1 = strtof(s, NULL);
                break;
            case FIELD_GX1:
                p->gx1 = strtof(s, NULL);
                break;
            case FIELD_GY1:
                p->gy1 = strtof(s, NULL);
                break;
            case FIELD_GZ1:
                p->gz1 = strtof(s, NULL);
                break;

            case FIELD_AX2:
                p->ax2 = strtof(s, NULL);
                break;
            case FIELD_AY2:
                p->ay2 = strtof(s, NULL);
                break;
            case FIELD_AZ2:
                p->az2 = strtof(s, NULL);
                break;
            case FIELD_GX2:
                p->gx2 = strtof(s, NULL);
                break;
            case FIELD_GY2:
                p->gy2 = strtof(s, NULL);
                break;
            case FIELD_GZ2:
                p->gz2 = strtof(s, NULL);
                break;

            case FIELD_Q0:
                p->q0 = strtof(s, NULL);
                break;
            case FIELD_Q1:
                p->q1 = strtof(s, NULL);
                break;
            case FIELD_Q2:
                p->q2 = strtof(s, NULL);
                break;
            case FIELD_Q3:
                p->q3 = strtof(s, NULL);
                break;

            case FIELD_OMEGAX0:
                p->omega_x0 = strtof(s, NULL);
                break;
            case FIELD_OMEGY0:
                p->omega_y0 = strtof(s, NULL);
                break;
            case FIELD_OMEGAZ0:
                p->omega_z0 = strtof(s, NULL);
                break;

            case FIELD_ACCX:
                p->acc_x = strtof(s, NULL);
                break;
            case FIELD_ACCY:
                p->acc_y = strtof(s, NULL);
                break;
            case FIELD_ACCZ:
                p->acc_z = strtof(s, NULL);
                break;

            case FIELD_MAGX:
                p->mag_x = strtof(s, NULL);
                break;
            case FIELD_MAGY:
                p->mag_y = strtof(s, NULL);
                break;
            case FIELD_MAGZ:
                p->mag_z = strtof(s, NULL);
                break;

            case FIELD_TEMP0:
                p->temperature[0] = (int16_t)atoi(s);
                break;
            case FIELD_TEMP1:
                p->temperature[1] = (int16_t)atoi(s);
                break;
            case FIELD_TEMP2:
                p->temperature[2] = (int16_t)atoi(s);
                break;

            case FIELD_DIAG:
                p->diagnostic_flag = (uint8_t)atoi(s);
                break;
            case FIELD_BARO:
                p->barometric_pressure = strtof(s, NULL);
                break;

            case FIELD_DEVICEMOTION:
                p->device_motion_status = (int8_t)atoi(s);
                break;

            case FIELD_RESERVED0_1:
            case FIELD_RESERVED0_2:
            case FIELD_RESERVED0_3:
            case FIELD_TYPE:
            case FIELD_NONE:
            default:
                break;
            }
        }
        break;
    }

    // Long composite with velocity/position
    case C2_PACKET_TYPE_IMU:
    case C2_PACKET_TYPE_AHRS:
    case C2_PACKET_TYPE_INS:
    {
        fs_imu_composite_packet_2_t *p = &out->composite2;
        memset(p, 0, sizeof(*p));
        p->type = (uint8_t)type_i;

        for (int col = 0; col < count; ++col)
        {
            fs_field_id_t f = (fs_field_id_t)r->field_for_column[col];
            const char *s = tokens[col];

            switch (f)
            {
            case FIELD_TIME:
                p->time = (uint32_t)strtoul(s, NULL, 10);
                break;

            case FIELD_AX0:
                p->ax0 = strtof(s, NULL);
                break;
            case FIELD_AY0:
                p->ay0 = strtof(s, NULL);
                break;
            case FIELD_AZ0:
                p->az0 = strtof(s, NULL);
                break;
            case FIELD_GX0:
                p->gx0 = strtof(s, NULL);
                break;
            case FIELD_GY0:
                p->gy0 = strtof(s, NULL);
                break;
            case FIELD_GZ0:
                p->gz0 = strtof(s, NULL);
                break;

            case FIELD_AX1:
                p->ax1 = strtof(s, NULL);
                break;
            case FIELD_AY1:
                p->ay1 = strtof(s, NULL);
                break;
            case FIELD_AZ1:
                p->az1 = strtof(s, NULL);
                break;
            case FIELD_GX1:
                p->gx1 = strtof(s, NULL);
                break;
            case FIELD_GY1:
                p->gy1 = strtof(s, NULL);
                break;
            case FIELD_GZ1:
                p->gz1 = strtof(s, NULL);
                break;

            case FIELD_AX2:
                p->ax2 = strtof(s, NULL);
                break;
            case FIELD_AY2:
                p->ay2 = strtof(s, NULL);
                break;
            case FIELD_AZ2:
                p->az2 = strtof(s, NULL);
                break;
            case FIELD_GX2:
                p->gx2 = strtof(s, NULL);
                break;
            case FIELD_GY2:
                p->gy2 = strtof(s, NULL);
                break;
            case FIELD_GZ2:
                p->gz2 = strtof(s, NULL);
                break;

            case FIELD_Q0:
                p->q0 = strtof(s, NULL);
                break;
            case FIELD_Q1:
                p->q1 = strtof(s, NULL);
                break;
            case FIELD_Q2:
                p->q2 = strtof(s, NULL);
                break;
            case FIELD_Q3:
                p->q3 = strtof(s, NULL);
                break;

            case FIELD_OMEGAX0:
                p->omega_x0 = strtof(s, NULL);
                break;
            case FIELD_OMEGY0:
                p->omega_y0 = strtof(s, NULL);
                break;
            case FIELD_OMEGAZ0:
                p->omega_z0 = strtof(s, NULL);
                break;

            case FIELD_ACCX:
                p->acc_x = strtof(s, NULL);
                break;
            case FIELD_ACCY:
                p->acc_y = strtof(s, NULL);
                break;
            case FIELD_ACCZ:
                p->acc_z = strtof(s, NULL);
                break;

            case FIELD_VX:
                p->vx = strtof(s, NULL);
                break;
            case FIELD_VY:
                p->vy = strtof(s, NULL);
                break;
            case FIELD_VZ:
                p->vz = strtof(s, NULL);
                break;

            case FIELD_RX:
                p->rx = strtod(s, NULL);
                break;
            case FIELD_RY:
                p->ry = strtod(s, NULL);
                break;
            case FIELD_RZ:
                p->rz = strtod(s, NULL);
                break;

            case FIELD_MAGX:
                p->mag_x = strtof(s, NULL);
                break;
            case FIELD_MAGY:
                p->mag_y = strtof(s, NULL);
                break;
            case FIELD_MAGZ:
                p->mag_z = strtof(s, NULL);
                break;

            case FIELD_TEMP0:
                p->temperature[0] = (int16_t)atoi(s);
                break;
            case FIELD_TEMP1:
                p->temperature[1] = (int16_t)atoi(s);
                break;
            case FIELD_TEMP2:
                p->temperature[2] = (int16_t)atoi(s);
                break;

            case FIELD_DIAG:
                p->diagnostic_flag = (uint8_t)atoi(s);
                break;
            case FIELD_BARO:
                p->barometric_pressure = strtof(s, NULL);
                break;

            case FIELD_DEVICEMOTION:
                p->device_motion_status = (int8_t)atoi(s);
                break;

            case FIELD_RESERVED0_1:
            case FIELD_RESERVED0_2:
            case FIELD_RESERVED0_3:
            case FIELD_TYPE:
            case FIELD_NONE:
            default:
                break;
            }
        }
        break;
    }

    // 64-bit time composite (no velocity/position)
    case C64_PACKET_TYPE_IMU:
    case C64_PACKET_TYPE_AHRS:
    case C64_PACKET_TYPE_INS:
    {
        fs_imu_composite_packet_64_t *p = &out->composite64;
        memset(p, 0, sizeof(*p));
        p->type = (uint8_t)type_i;

        for (int col = 0; col < count; ++col)
        {
            fs_field_id_t f = (fs_field_id_t)r->field_for_column[col];
            const char *s = tokens[col];

            switch (f)
            {
            case FIELD_TIME:
                p->time = (uint64_t)strtoull(s, NULL, 10);
                break;

            case FIELD_AX0:
                p->ax0 = strtof(s, NULL);
                break;
            case FIELD_AY0:
                p->ay0 = strtof(s, NULL);
                break;
            case FIELD_AZ0:
                p->az0 = strtof(s, NULL);
                break;
            case FIELD_GX0:
                p->gx0 = strtof(s, NULL);
                break;
            case FIELD_GY0:
                p->gy0 = strtof(s, NULL);
                break;
            case FIELD_GZ0:
                p->gz0 = strtof(s, NULL);
                break;

            case FIELD_AX1:
                p->ax1 = strtof(s, NULL);
                break;
            case FIELD_AY1:
                p->ay1 = strtof(s, NULL);
                break;
            case FIELD_AZ1:
                p->az1 = strtof(s, NULL);
                break;
            case FIELD_GX1:
                p->gx1 = strtof(s, NULL);
                break;
            case FIELD_GY1:
                p->gy1 = strtof(s, NULL);
                break;
            case FIELD_GZ1:
                p->gz1 = strtof(s, NULL);
                break;

            case FIELD_AX2:
                p->ax2 = strtof(s, NULL);
                break;
            case FIELD_AY2:
                p->ay2 = strtof(s, NULL);
                break;
            case FIELD_AZ2:
                p->az2 = strtof(s, NULL);
                break;
            case FIELD_GX2:
                p->gx2 = strtof(s, NULL);
                break;
            case FIELD_GY2:
                p->gy2 = strtof(s, NULL);
                break;
            case FIELD_GZ2:
                p->gz2 = strtof(s, NULL);
                break;

            case FIELD_Q0:
                p->q0 = strtof(s, NULL);
                break;
            case FIELD_Q1:
                p->q1 = strtof(s, NULL);
                break;
            case FIELD_Q2:
                p->q2 = strtof(s, NULL);
                break;
            case FIELD_Q3:
                p->q3 = strtof(s, NULL);
                break;

            case FIELD_OMEGAX0:
                p->omega_x0 = strtof(s, NULL);
                break;
            case FIELD_OMEGY0:
                p->omega_y0 = strtof(s, NULL);
                break;
            case FIELD_OMEGAZ0:
                p->omega_z0 = strtof(s, NULL);
                break;

            case FIELD_ACCX:
                p->acc_x = strtof(s, NULL);
                break;
            case FIELD_ACCY:
                p->acc_y = strtof(s, NULL);
                break;
            case FIELD_ACCZ:
                p->acc_z = strtof(s, NULL);
                break;

            case FIELD_MAGX:
                p->mag_x = strtof(s, NULL);
                break;
            case FIELD_MAGY:
                p->mag_y = strtof(s, NULL);
                break;
            case FIELD_MAGZ:
                p->mag_z = strtof(s, NULL);
                break;

            case FIELD_TEMP0:
                p->temperature[0] = (int16_t)atoi(s);
                break;
            case FIELD_TEMP1:
                p->temperature[1] = (int16_t)atoi(s);
                break;
            case FIELD_TEMP2:
                p->temperature[2] = (int16_t)atoi(s);
                break;

            case FIELD_DIAG:
                p->diagnostic_flag = (uint8_t)atoi(s);
                break;
            case FIELD_BARO:
                p->barometric_pressure = strtof(s, NULL);
                break;

            case FIELD_DEVICEMOTION:
                p->device_motion_status = (int8_t)atoi(s);
                break;

            case FIELD_VX:
            case FIELD_VY:
            case FIELD_VZ:
            case FIELD_RX:
            case FIELD_RY:
            case FIELD_RZ:
            case FIELD_RESERVED0_1:
            case FIELD_RESERVED0_2:
            case FIELD_RESERVED0_3:
            case FIELD_TYPE:
            case FIELD_NONE:
            default:
                break;
            }
        }
        break;
    }

    // 64-bit time + velocity/position
    case C642_PACKET_TYPE_IMU:
    case C642_PACKET_TYPE_AHRS:
    case C642_PACKET_TYPE_INS:
    {
        fs_imu_composite_packet_64_2_t *p = &out->composite64_2;
        memset(p, 0, sizeof(*p));
        p->type = (uint8_t)type_i;

        for (int col = 0; col < count; ++col)
        {
            fs_field_id_t f = (fs_field_id_t)r->field_for_column[col];
            const char *s = tokens[col];

            switch (f)
            {
            case FIELD_TIME:
                p->time = (uint64_t)strtoull(s, NULL, 10);
                break;

            case FIELD_AX0:
                p->ax0 = strtof(s, NULL);
                break;
            case FIELD_AY0:
                p->ay0 = strtof(s, NULL);
                break;
            case FIELD_AZ0:
                p->az0 = strtof(s, NULL);
                break;
            case FIELD_GX0:
                p->gx0 = strtof(s, NULL);
                break;
            case FIELD_GY0:
                p->gy0 = strtof(s, NULL);
                break;
            case FIELD_GZ0:
                p->gz0 = strtof(s, NULL);
                break;

            case FIELD_AX1:
                p->ax1 = strtof(s, NULL);
                break;
            case FIELD_AY1:
                p->ay1 = strtof(s, NULL);
                break;
            case FIELD_AZ1:
                p->az1 = strtof(s, NULL);
                break;
            case FIELD_GX1:
                p->gx1 = strtof(s, NULL);
                break;
            case FIELD_GY1:
                p->gy1 = strtof(s, NULL);
                break;
            case FIELD_GZ1:
                p->gz1 = strtof(s, NULL);
                break;

            case FIELD_AX2:
                p->ax2 = strtof(s, NULL);
                break;
            case FIELD_AY2:
                p->ay2 = strtof(s, NULL);
                break;
            case FIELD_AZ2:
                p->az2 = strtof(s, NULL);
                break;
            case FIELD_GX2:
                p->gx2 = strtof(s, NULL);
                break;
            case FIELD_GY2:
                p->gy2 = strtof(s, NULL);
                break;
            case FIELD_GZ2:
                p->gz2 = strtof(s, NULL);
                break;

            case FIELD_Q0:
                p->q0 = strtof(s, NULL);
                break;
            case FIELD_Q1:
                p->q1 = strtof(s, NULL);
                break;
            case FIELD_Q2:
                p->q2 = strtof(s, NULL);
                break;
            case FIELD_Q3:
                p->q3 = strtof(s, NULL);
                break;

            case FIELD_OMEGAX0:
                p->omega_x0 = strtof(s, NULL);
                break;
            case FIELD_OMEGY0:
                p->omega_y0 = strtof(s, NULL);
                break;
            case FIELD_OMEGAZ0:
                p->omega_z0 = strtof(s, NULL);
                break;

            case FIELD_OMEGAX1:
            case FIELD_OMEGY1:
            case FIELD_OMEGAZ1:
                break;

            case FIELD_ACCX:
                p->acc_x = strtof(s, NULL);
                break;
            case FIELD_ACCY:
                p->acc_y = strtof(s, NULL);
                break;
            case FIELD_ACCZ:
                p->acc_z = strtof(s, NULL);
                break;

            case FIELD_VX:
                p->vx = strtof(s, NULL);
                break;
            case FIELD_VY:
                p->vy = strtof(s, NULL);
                break;
            case FIELD_VZ:
                p->vz = strtof(s, NULL);
                break;

            case FIELD_RX:
                p->rx = strtod(s, NULL);
                break;
            case FIELD_RY:
                p->ry = strtod(s, NULL);
                break;
            case FIELD_RZ:
                p->rz = strtod(s, NULL);
                break;

            case FIELD_MAGX:
                p->mag_x = strtof(s, NULL);
                break;
            case FIELD_MAGY:
                p->mag_y = strtof(s, NULL);
                break;
            case FIELD_MAGZ:
                p->mag_z = strtof(s, NULL);
                break;

            case FIELD_TEMP0:
                p->temperature[0] = (int16_t)atoi(s);
                break;
            case FIELD_TEMP1:
                p->temperature[1] = (int16_t)atoi(s);
                break;
            case FIELD_TEMP2:
                p->temperature[2] = (int16_t)atoi(s);
                break;

            case FIELD_DIAG:
                p->diagnostic_flag = (uint8_t)atoi(s);
                break;
            case FIELD_BARO:
                p->barometric_pressure = strtof(s, NULL);
                break;

            case FIELD_DEVICEMOTION:
                p->device_motion_status = (int8_t)atoi(s);
                break;

            case FIELD_RESERVED0_1:
            case FIELD_RESERVED0_2:
            case FIELD_RESERVED0_3:
            case FIELD_TYPE:
            case FIELD_NONE:
            default:
                break;
            }
        }
        break;
    }

    default:
        return -2;
    }

    return 0;
}

int fs_replay_open(fs_replay_t *r, const char *path, uint32_t sparse_step)
{
    if (!r || !path || sparse_step == 0)
        return -1;

    memset(r, 0, sizeof(*r));
    r->sparse_step = sparse_step;

    r->fp = fopen(path, "r");
    if (!r->fp)
        return -2;

    if (!fgets(linebuf, sizeof(linebuf), r->fp))
        return -3;

    if (fs_replay_parse_header(r, linebuf) != 0)
        return -4;

    // NOTE: Dynamic allocation allowable here,
    // since this is not being used in real-time application.
    uint32_t cap = 4096;
    r->offsets = (uint64_t *)malloc(cap * sizeof(uint64_t));
    if (!r->offsets)
        return -5;

    long pos = ftell(r->fp);
    uint32_t line = 0;
    uint32_t block = 0;

    while (fgets(linebuf, sizeof(linebuf), r->fp))
    {
        if (line % sparse_step == 0)
        {
            if (block == cap)
            {
                cap *= 2;
                uint64_t *tmp = (uint64_t *)realloc(r->offsets, cap * sizeof(uint64_t));
                if (!tmp)
                    return -6;
                r->offsets = tmp;
            }
            r->offsets[block++] = (uint64_t)pos;
        }

        pos = ftell(r->fp);
        line++;
    }

    r->num_lines = line;
    r->num_blocks = block;
    r->has_index = (block > 0) ? 1 : 0;

    if (block > 0)
        fseek(r->fp, (long)r->offsets[0], SEEK_SET);

    return 0;
}

int fs_replay_read_next(fs_replay_t *r, fs_packet_union_t *out)
{
    if (!r || !r->fp || !out)
        return -1;

    if (!fgets(linebuf, sizeof(linebuf), r->fp))
        return 1; // EOF

    return fs_replay_parse_packet(r, linebuf, out);
}

int fs_replay_read_line(fs_replay_t *r,
                        uint32_t line_index,
                        fs_packet_union_t *out)
{
    if (!r || !r->fp || !out)
        return -1;

    if (!r->has_index || !r->offsets)
        return -2;

    if (line_index >= r->num_lines)
        return -3;

    uint32_t N = r->sparse_step;
    uint32_t block = line_index / N;
    uint32_t block_start = block * N;

    uint32_t next_block = block + 1;
    uint32_t next_start = next_block * N;

    uint32_t dist_backward = line_index - block_start;
    uint32_t dist_forward = (next_block < r->num_blocks && next_start < r->num_lines)
                                ? (next_start - line_index)
                                : UINT32_MAX;

    uint32_t start_line;
    uint64_t start_offset;

    if (dist_backward <= dist_forward)
    {
        start_line = block_start;
        start_offset = r->offsets[block];
    }
    else
    {
        start_line = next_start;
        start_offset = r->offsets[next_block];
    }

    if (fseek(r->fp, (long)start_offset, SEEK_SET) != 0)
        return -4;

    for (uint32_t i = start_line; i < line_index; i++)
    {
        if (!fgets(linebuf, sizeof(linebuf), r->fp))
            return -5;
    }

    if (!fgets(linebuf, sizeof(linebuf), r->fp))
        return -6;

    return fs_replay_parse_packet(r, linebuf, out);
}

void fs_replay_close(fs_replay_t *r)
{
    if (!r)
        return;

    if (r->fp)
    {
        fclose(r->fp);
        r->fp = NULL;
    }

    if (r->offsets)
    {
        free(r->offsets);
        r->offsets = NULL;
    }

    r->num_lines = 0;
    r->num_blocks = 0;
    r->sparse_step = 0;
    r->has_index = 0;
}

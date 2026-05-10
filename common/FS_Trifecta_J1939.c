#include "FS_Trifecta_J1939.h"

static int fs_j1939_decode_ahrs_fused(fs_packet_union_t *packet, const fs_j1939_packet_t *j1939_data)
{
    int status = 0;
    switch (j1939_data->code)
    {
        case FS_J1939_CODE_ORIENTATION_QUAT_W:
            
        break;
        case FS_J1939_CODE_ORIENTATION_QUAT_X:
            
        break;
        case FS_J1939_CODE_ORIENTATION_QUAT_Y:
            
        break;
        case FS_J1939_CODE_ORIENTATION_QUAT_Z:
            
        break;
        default: status = -1;
    }
    return status;
}

static int fs_j1939_decode_imu_raw(fs_packet_union_t *packet, const fs_j1939_packet_t *j1939_data)
{
    return 0;
}

static int fs_j1939_decode_config(fs_packet_union_t *packet, const fs_j1939_packet_t *j1939_data)
{
    return 0;
}

static int fs_j1939_decode_status(fs_packet_union_t *packet, const fs_j1939_packet_t *j1939_data)
{
    return 0;
}

/// @brief Decode J1939 data from a J1939 packet and update the fs_packet_union_t.
/// @param packet
/// @param j1939_data
/// @return 0 on success
int fs_j1939_decode(fs_packet_union_t *packet, const fs_j1939_packet_t *j1939_data)
{
    int status = 0;
    if (!packet || !j1939_data)
        return -1;
    switch (j1939_data->pgn)
    {
    case FS_J1939_PGN_AHRS_FUSED:
        status = fs_j1939_decode_ahrs_fused(packet, j1939_data);
        break;
    case FS_J1939_PGN_IMU_RAW:
        status = fs_j1939_decode_imu_raw(packet, j1939_data);
        break;
    case FS_J1939_PGN_CONFIG:
        status = fs_j1939_decode_config(packet, j1939_data);
        break;
    case FS_J1939_PGN_STATUS:
        status = fs_j1939_decode_status(packet, j1939_data);
        break;
    default:
        return -2;
    }
    return status;
}
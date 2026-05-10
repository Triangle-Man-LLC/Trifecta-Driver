#ifndef TRIFECTA_DEFS_J1939_H
#define TRIFECTA_DEFS_J1939_H

#include "FS_Trifecta_Defs_Packets.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /// @brief PGN definitions
    typedef enum fs_j1939_pgn
    {
        FS_J1939_PGN_AHRS_FUSED = 0x00FF10,
        FS_J1939_PGN_IMU_RAW = 0x00FF11,
        FS_J1939_PGN_STATUS = 0x00FF12,
        FS_J1939_PGN_CONFIG = 0x00EF00,
    } fs_j1939_pgn_t;

    /// @brief Code section
    typedef enum fs_j1939_code
    {
        FS_J1939_CODE_TICKS_MS,
        FS_J1939_CODE_ACCEL_X,
        FS_J1939_CODE_ACCEL_Y,
        FS_J1939_CODE_ACCEL_Z,
        FS_J1939_CODE_ORIENTATION_QUAT_W,
        FS_J1939_CODE_ORIENTATION_QUAT_X,
        FS_J1939_CODE_ORIENTATION_QUAT_Y,
        FS_J1939_CODE_ORIENTATION_QUAT_Z,
        FS_J1939_CODE_ORIENTATION_ROT_X,
        FS_J1939_CODE_ORIENTATION_ROT_Y,
        FS_J1939_CODE_ORIENTATION_ROT_Z,
        FS_J1939_CODE_GYRO_X,
        FS_J1939_CODE_GYRO_Y,
        FS_J1939_CODE_GYRO_Z,
        FS_J1939_CODE_ACCEL_0_X,
        FS_J1939_CODE_ACCEL_0_Y,
        FS_J1939_CODE_ACCEL_0_Z,
        FS_J1939_CODE_GYRO_0_X,
        FS_J1939_CODE_GYRO_0_Y,
        FS_J1939_CODE_GYRO_0_Z,
        FS_J1939_CODE_ACCEL_1_X,
        FS_J1939_CODE_ACCEL_1_Y,
        FS_J1939_CODE_ACCEL_1_Z,
        FS_J1939_CODE_GYRO_1_X,
        FS_J1939_CODE_GYRO_1_Y,
        FS_J1939_CODE_GYRO_1_Z,
        FS_J1939_CODE_ACCEL_2_X,
        FS_J1939_CODE_ACCEL_2_Y,
        FS_J1939_CODE_ACCEL_2_Z,
        FS_J1939_CODE_GYRO_2_X,
        FS_J1939_CODE_GYRO_2_Y,
        FS_J1939_CODE_GYRO_2_Z,
    } fs_j1939_code_t;

    /// @brief Packet structure
    typedef struct fs_j1939_packet
    {
        uint32_t pgn;
        uint8_t src;
        uint8_t priority;
        uint8_t dlc;
        uint8_t data[8];
        uint32_t timestamp_ms;
        fs_j1939_code_t code;
        float value;
    } fs_j1939_packet_t;

    /// @brief 
    /// @param can_id 
    /// @param data 
    /// @param timestamp_ms 
    /// @return 0 on success
    fs_j1939_packet_t fs_j1939_decode(uint32_t can_id,
                                      const uint8_t data[8],
                                      uint32_t timestamp_ms);

#ifdef __cplusplus
}
#endif

#endif

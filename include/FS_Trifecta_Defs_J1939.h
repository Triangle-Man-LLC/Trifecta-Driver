/// Driver for the Trifecta series of IMU/AHRS/INS devices
/// Copyright 2026 4rge.ai and/or Triangle Man LLC
/// Usage and redistribution of this code is permitted
/// but this notice must be retained in all copies of the code.

/// THIS SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
/// INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE,
/// AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
/// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

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
        FS_J1939_CODE_ORIENTATION_QUAT_W,
        FS_J1939_CODE_ORIENTATION_QUAT_X,
        FS_J1939_CODE_ORIENTATION_QUAT_Y,
        FS_J1939_CODE_ORIENTATION_QUAT_Z,
        FS_J1939_CODE_ACCEL_X,
        FS_J1939_CODE_ACCEL_Y,
        FS_J1939_CODE_ACCEL_Z,
        FS_J1939_CODE_GYRO_X,
        FS_J1939_CODE_GYRO_Y,
        FS_J1939_CODE_GYRO_Z,
        FS_J1939_STATUS_REG,
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

#ifdef __cplusplus
}
#endif

#endif

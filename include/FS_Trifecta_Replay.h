/// Driver for the Trifecta series of IMU/AHRS/INS devices
/// Copyright 2026 4rge.ai and/or Triangle Man LLC
/// Usage and redistribution of this code is permitted
/// but this notice must be retained in all copies of the code.

/// THIS SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
/// INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE,
/// AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
/// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#ifndef TRIFECTA_REPLAY_H
#define TRIFECTA_REPLAY_H

#include "FS_Trifecta_Defs.h"
#include "FS_Trifecta_Interfaces.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define FS_REPLAY_MAX_COLUMNS 64

    typedef enum fs_field_id
    {
        FIELD_NONE = 0,

        FIELD_TYPE,
        FIELD_TIME,

        FIELD_AX0,
        FIELD_AY0,
        FIELD_AZ0,
        FIELD_GX0,
        FIELD_GY0,
        FIELD_GZ0,

        FIELD_AX1,
        FIELD_AY1,
        FIELD_AZ1,
        FIELD_GX1,
        FIELD_GY1,
        FIELD_GZ1,

        FIELD_AX2,
        FIELD_AY2,
        FIELD_AZ2,
        FIELD_GX2,
        FIELD_GY2,
        FIELD_GZ2,

        FIELD_Q0,
        FIELD_Q1,
        FIELD_Q2,
        FIELD_Q3,

        FIELD_OMEGAX0,
        FIELD_OMEGY0,
        FIELD_OMEGAZ0,
        FIELD_OMEGAX1,
        FIELD_OMEGY1,
        FIELD_OMEGAZ1,

        FIELD_ACCX,
        FIELD_ACCY,
        FIELD_ACCZ,

        FIELD_VX,
        FIELD_VY,
        FIELD_VZ,

        FIELD_RX,
        FIELD_RY,
        FIELD_RZ,

        FIELD_RESERVED0_1,
        FIELD_RESERVED0_2,
        FIELD_RESERVED0_3,

        FIELD_MAGX,
        FIELD_MAGY,
        FIELD_MAGZ,

        FIELD_TEMP0,
        FIELD_TEMP1,
        FIELD_TEMP2,

        FIELD_DIAG,
        FIELD_BARO,

        FIELD_DEVICEMOTION,

        FIELD_COUNT
    } fs_field_id_t;

    typedef struct fs_replay
    {
        FILE *fp;
        uint64_t *offsets;
        uint32_t num_lines;
        uint32_t num_blocks;
        uint32_t sparse_step;
        int has_index;

        int field_for_column[FS_REPLAY_MAX_COLUMNS]; // maps column index → fs_field_id_t
    } fs_replay_t;

    int fs_replay_parse_header(fs_replay_t *r, const char *line);
    int fs_replay_open(fs_replay_t *r, const char *path, uint32_t sparse_step);
    int fs_replay_read_next(fs_replay_t *r, fs_packet_union_t *out);
    int fs_replay_read_line(fs_replay_t *r, uint32_t line_index, fs_packet_union_t *out);
    void fs_replay_close(fs_replay_t *r);

#ifdef __cplusplus
}
#endif

#endif

/// Driver for the Trifecta series of IMU/AHRS/INS devices
/// Copyright 2024 4rge.ai and/or Triangle Man LLC
/// Usage and redistribution of this code is permitted
/// but this notice must be retained in all copies of the code.

/// THIS SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
/// INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE,
/// AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
/// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#ifndef TRIFECTA_DEVICE_H
#define TRIFECTA_DEVICE_H

#include "FS_Trifecta_Defs.h"
#include "FS_Trifecta_Interfaces.h"

#ifdef __cplusplus
extern "C"
{
#endif

    int fs_handle_received_commands(fs_device_info *device_info, const void *cmd_buf, size_t buf_len);
    int fs_device_parse_packet(fs_device_info *device_handle, const void *rx_buf, ssize_t rx_len, fs_communication_mode source);
    void fs_q_to_euler_angles(float *estRoll, float *estPitch, float *estYaw, float q0, float q1, float q2, float q3, bool degrees);

#ifdef __cplusplus
}
#endif

#endif
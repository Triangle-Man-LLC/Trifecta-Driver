/// Driver for the Trifecta series of IMU/AHRS/INS devices
/// Copyright 2024 4rge.ai and/or Triangle Man LLC
/// Usage and redistribution of this code is permitted
/// but this notice must be retained in all copies of the code.

/// THIS SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
/// INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE,
/// AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
/// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#ifndef TRIFECTA_H
#define TRIFECTA_H

#include "FS_Trifecta_Defs.h"

#ifdef __cplusplus
extern "C"
{
#endif

    int fs_initialize_networked(fs_driver_config *dconfig, const char *device_ip_address);
    int fs_initialize_serial(fs_driver_config *dconfig, int fd);

    int fs_start_stream();
    int fs_stop_stream();

    int fs_read_one_shot();

    int fs_reboot_device();
    int fs_set_communication_mode(fs_communication_mode mode);
    int fs_set_wifi_params(char* ssid, char* password);

    int fs_enable_logging(bool do_enable);

    int fs_get_last_timestamp(uint32_t *time);
    int fs_get_raw_packet(fs_packet_union *packet_buffer);
    int fs_get_orientation(fs_quaternion *orientation_buffer);
    int fs_get_orientation_euler(fs_vector3 *orientation_buffer);
    int fs_get_acceleration(fs_vector3 *acceleration_buffer);
    int fs_get_angular_velocity(fs_vector3 *angular_velocity_buffer);
    int fs_get_velocity(fs_vector3 *velocity_buffer);
    int fs_get_position(fs_vector3 *position_buffer);

    int fs_set_ahrs_heading(float heading_deg);
    int fs_set_ins_position(fs_vector3 *position);

    int fs_closedown();

#ifdef __cplusplus
}
#endif

#endif

/// Driver for the Trifecta series of IMU/AHRS/INS devices
/// Copyright 2024 4rge.ai and/or Triangle Man LLC
/// Usage and redistribution of this code is permitted
/// but this notice must be retained in all copies of the code.

/// THIS SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
/// INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE,
/// AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
/// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#ifndef TRIFECTA_NETWORKED_H
#define TRIFECTA_NETWORKED_H

#include "FS_Trifecta_Defs.h"
#include "FS_Trifecta_Interfaces.h"

#include "FS_Trifecta_Device.h"

#ifdef __cplusplus
extern "C"
{
#endif

    int fs_network_set_driver_config(fs_driver_config *driver_config);
    int fs_network_send_message(fs_device_info *device_handle, char *message, size_t len);
    int fs_network_start(const char *ip_addr, fs_device_info *device_handle);
    int fs_network_start_device_stream(fs_device_info *device_handle);
    int fs_network_stop_device_stream(fs_device_info *device_handle);
    int fs_network_read_one_shot(fs_device_info *device_handle);
    int fs_network_exit(fs_device_info *device_handle);
    int fs_network_device_restart(fs_device_info *device_handle);
    int fs_network_set_device_operating_mode(fs_device_info *device_handle, fs_communication_mode mode);
    int fs_network_set_network_params(fs_device_info *device_handle, char *ssid, char *password);

#ifdef __cplusplus
}
#endif

#endif

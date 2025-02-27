/// Driver for the Trifecta series of IMU/AHRS/INS devices
/// Copyright 2024 4rge.ai and/or Triangle Man LLC
/// Usage and redistribution of this code is permitted
/// but this notice must be retained in all copies of the code.

/// THIS SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
/// INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE,
/// AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
/// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#ifndef TRIFECTA_INTERFACES_H
#define TRIFECTA_INTERFACES_H

#include "FS_Trifecta_Defs.h"

#ifdef __cplusplus
extern "C"
{
#endif

    // -- Platform-specific methods --
    // -- Be sure to implement these when porting to a new platform --

    int fs_init_network_tcp_driver(fs_device_info *device_handle);
    int fs_init_network_udp_driver(fs_device_info *device_handle);
    int fs_init_serial_driver(fs_device_info *device_handle);

    int fs_thread_start(void(thread_func)(void *), void *params, bool *thread_running_flag, size_t stack_size, int priority, int core_affinity);
    int fs_thread_exit();

    ssize_t fs_transmit_networked_tcp(fs_device_info *device_handle, void *tx_buffer, size_t length_bytes, int timeout_micros);
    ssize_t fs_transmit_networked_udp(fs_device_info *device_handle, void *tx_buffer, size_t length_bytes, int timeout_micros);
    ssize_t fs_transmit_serial(fs_device_info *device_handle, void *tx_buffer, size_t length_bytes, int timeout_micros);

    ssize_t fs_receive_networked_tcp(fs_device_info *device_handle, void *rx_buffer, size_t length_bytes, int timeout_micros);
    ssize_t fs_receive_networked_udp(fs_device_info *device_handle, void *rx_buffer, size_t length_bytes, int timeout_micros);
    ssize_t fs_receive_serial(fs_device_info *device_handle, void *rx_buffer, size_t length_bytes, int timeout_micros);

    int fs_shutdown_network_tcp_driver(fs_device_info *device_handle);
    int fs_shutdown_network_udp_driver(fs_device_info *device_handle);
    int fs_shutdown_serial_driver(fs_device_info *device_handle);

    int fs_log_output(const char *format, ...);
    int fs_toggle_logging(bool do_log);
    
    int fs_delay(int millis);
    int fs_delay_for(uint32_t *current_time, int millis);
    int fs_get_current_time(uint32_t *current_time);

#ifdef __cplusplus
}
#endif

#endif
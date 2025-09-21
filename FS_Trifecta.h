/// Driver for the Trifecta series of IMU/AHRS/INS devices
/// Copyright 2025 4rge.ai and/or Triangle Man LLC
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


    /// @section Device handle allocation/deallocation

    /// @brief Allocate a new device handle for use externally.
    /// @return A default-initialized device handle.
    FS_API fs_device_info_t *fs_export_allocate_device();

    /// @brief Deallocate the exported device handle.
    /// This should only be used for purposes of external bindings (e.g. C# or Python)
    /// For native C/C++ uses, it is much more preferable to statically allocate.
    /// @param device The DYNAMICALLY ALLOCATED device handle
    /// @return None
    FS_API void fs_export_free_device(fs_device_info_t *device);


    /// @section Device initialization functions

    /// @brief Sets the connection configuration parameters for the device.
    /// Defaults to FS_DRIVER_CONFIG_DEFAULT if you do not call this function.
    /// @param device_handle Device handle
    /// @param device_handle Device handle
    /// @param driver_config Pointer to the device configuration definition.
    /// @return 0 on success.
    FS_API int fs_set_driver_parameters(fs_device_info_t *device_handle, fs_driver_config_t *driver_config);

    /// @brief Start the device in networked mode, this will attempt to connect to the device at given IP address.
    /// @param device_handle Device handle
    /// @param device_handle Device handle
    /// @param device_ip_address String representation of the device IP address. If set to "\0", the driver will auto-scan ports (on supported platforms).
    /// @return 0 on success.
    FS_API int fs_initialize_networked(fs_device_info_t *device_handle, const char *device_ip_address);

    /// @brief Start the device in wired serial mode, this will attempt to connect to the device at the given port.
    /// If the device_handle has a name set, it will only connect to a device with that name.
    /// @param device_handle Device handle
    /// @param device_handle Device handle
    /// @param fd File descriptor or USB/UART port number. If set to -1, the driver will auto-scan ports (on supported platforms).
    /// @return 0 on success.
    FS_API int fs_initialize_serial(fs_device_info_t *device_handle, int fd, fs_communication_mode_t serial_mode);

    /// @brief Deallocate the resources for the indicated device.
    /// Note that if you have obtained the handle through dynamic allocation, you will still need to free
    /// @param device_handle Device handle
    /// @return 0 on success.
    FS_API int fs_closedown(fs_device_info_t *device_handle);

    
    /// @section Stream control

    /// @brief Deallocate the resources for the indicated device.
    /// Note that if you have obtained the handle through dynamic allocation, you will still need to free
    /// @param device_handle Device handle
    /// @return 0 on success.
    FS_API int fs_closedown(fs_device_info_t *device_handle);

    
    /// @section Stream control

    /// @brief Request the device to asynchronously stream data. This data will arrive at the native speed (e.g. 200 Hz on Trifecta-K).
    /// Due to the high data rate, this may not work well on smaller host platforms, with large numbers of devices, or at slow baud rates.
    /// For resource constrained situations, it is recommended to periodically call fs_read_one_shot() instead.
    /// @param device_handle Device handle
    /// @return 0 on success.
    FS_API int fs_start_stream(fs_device_info_t *device_handle);

    /// @brief Stop the asynchronous data stream from the indicated device.
    /// @param device_handle Device handle
    /// @return 0 on success.
    FS_API int fs_stop_stream(fs_device_info_t *device_handle);

    /// @brief Request a single reading from the device. The updated reading will arrive within 5 ms.
    /// @param device_handle Device handle
    /// @return 0 on success.
    FS_API int fs_read_one_shot(fs_device_info_t *device_handle);

    /// @brief Trigger a device restart. Usually, there are not many situations where you would need to call this.
    /// @param device_handle Device handle
    /// @return 0 on success.
    FS_API int fs_reboot_device(fs_device_info_t *device_handle);


    /// @section Data export/viewing

    /// @brief Retrieve the internal timestamp of the last received transmission from the device.
    /// @param device_handle Device handle
    /// @param device_handle Device handle
    /// @param time Pointer to the buffer for storing the timestamp.
    /// @return 0 on success.
    FS_API int fs_get_last_timestamp(fs_device_info_t *device_handle, uint32_t *time);

    /// @brief Retrieve the latest device data packet.
    /// @param device_handle Device handle
    /// @param packet_buffer Pointer to the packet buffer.
    /// @return 0 on success.
    FS_API int fs_get_raw_packet(fs_device_info_t *device_handle, fs_packet_union_t *packet_buffer);

    /// @brief Retrieve the current size of the packet queue.
    /// @param device_handle Device handle
    /// @param packet_buffer Pointer to the packet buffer.
    /// @return 0 on success.
    FS_API int fs_get_raw_packet_queue_size(fs_device_info_t *device_handle);

    /// @brief Retrieve the indicated packet number from the queue.
    /// @param device_handle Device handle
    /// @param packet_buffer Pointer to the packet buffer.
    /// @param pos The position in queue to obtain, starting from earliest first.
    /// @return 0 on success.
    FS_API int fs_get_raw_packet_from_queue(fs_device_info_t *device_handle, fs_packet_union_t *packet_buffer, int pos);

    /// @brief Retrieve the latest device orientation (quaternion).
    /// @param device_handle Device handle
    /// @param orientation_buffer Pointer to the orientation buffer.
    /// @return 0 on success.
    FS_API int fs_get_orientation(fs_device_info_t *device_handle, fs_quaternion_t *orientation_buffer);

    /// @brief Retrieve the latest device orientation (euler angles)
    /// @param device_handle Device handle
    /// @param orientation_buffer Pointer to the orientation buffer
    /// @param degrees TRUE to output degrees, FALSE to output radians
    /// @return 0 on success.
    FS_API int fs_get_orientation_euler(fs_device_info_t *device_info, fs_vector3_t *orientation_buffer, bool degrees);

    /// @brief Retrieve the latest device acceleration.
    /// @param device_handle Device handle
    /// @param acceleration_buffer Pointer to the acceleration buffer.
    /// @return 0 on success.
    FS_API int fs_get_acceleration(fs_device_info_t *device_handle, fs_vector3_t *acceleration_buffer);

    /// @brief Retrieve the latest device angular velocity.
    /// @param device_handle Device handle
    /// @param angular_velocity_buffer Pointer to the angular velocity buffer.
    /// @return 0 on success.
    FS_API int fs_get_angular_velocity(fs_device_info_t *device_handle, fs_vector3_t *angular_velocity_buffer);

    /// @brief Retrieve the latest measured device velocity.
    /// @param device_handle Device handle
    /// @param velocity_buffer Pointer to the velocity buffer.
    /// @return 0 on success.
    FS_API int fs_get_velocity(fs_device_info_t *device_handle, fs_vector3_t *velocity_buffer);

    /// @brief Retrieve the latest measured device movement state.
    /// @param device_handle Device handle
    /// @param device_state_buffer Pointer to the buffer for storing the state.
    /// FS_RUN_STATUS_RUNNING when moving, FS_RUN_STATUS_IDLE when stationary.
    /// @return 0 on success.
    FS_API int fs_get_movement_state(fs_device_info_t *device_handle, fs_run_status_t *device_state_buffer);

    /// @brief Retrieve the latest device position.
    /// Note that for non-GNSS stabilized systems, there is very little meaning to this value.
    /// @param device_handle Device handle
    /// @param position_buffer Pointer to the position buffer.
    /// @return 0 on success.
    FS_API int fs_get_position(fs_device_info_t *device_handle, fs_vector3_t *position_buffer);
    

    /// @section Device configuration methods
    
    /// @brief Manually set the AHRS yaw angle. Coupled with fs_set_ins_position(), it is possible to perform INS alignment.
    /// However, this procedure is usually performed automatically when connected with a compatible GNSS.
    /// @param device_handle Device handle
    /// @param heading_deg The desired angle.
    /// @return 0 on success.
    FS_API int fs_set_ahrs_heading(fs_device_info_t *device_handle, float heading_deg);

    /// @brief Manually set the INS position, this is typically used to update the device position from a
    /// GNSS system. 
    /// @param device_handle Device handle
    /// @param position The position to set to (for now, this function only re-sets position to zero).
    /// @return 0 on success.
    FS_API int fs_set_ins_position(fs_device_info_t *device_handle, fs_vector3_t *position);

    /// @brief Set all enabled communication modes of the device.
    /// @param device_handle Device handle
    /// @param modes The device operating modes, should be an OR flag of fs_communication_mode_t
    /// @return 0 on success.
    FS_API int fs_set_communication_mode(fs_device_info_t *device_handle, int modes);

    /// @brief 
    /// @param device_handle 
    /// @param ssid 
    /// @param pw 
    /// @param access_point 
    /// @return 0 on success. 
    FS_API int fs_set_network_parameters(fs_device_info_t *device_handle, const char ssid[32], const char pw[64], bool access_point);

    /// @brief Device UDP port is typically controlled in the backend, but can be changed if necessary.
    /// @param device_handle Device handle (Wi-Fi mode should be enabled)
    /// @param port Between 1024-65535
    /// @return 0 on success.
    FS_API int fs_set_network_udp_port(fs_device_info_t *device_handle, int port);

    /// @brief 
    /// @param device_handle 
    /// @param baudrate 
    /// @return 0 on success. 
    FS_API int fs_set_serial_uart_baudrate(fs_device_info_t *device_handle, int baudrate);

    /// @brief 
    /// @param device_handle 
    /// @param device_config_info 
    /// @return 0 on success. 
    FS_API int fs_get_device_operating_state(fs_device_info_t *device_handle, fs_device_params_t *device_params_info);


    /// @section Debug utilities - This should typically not be used at all.

    /// @brief Toggle logging (logging is disabled by default because it is a severe latency penalty).
    /// You should never enable it unless debugging some issue.
    /// @param do_enable TRUE to turn logging on.
    /// @return
    FS_API int fs_enable_logging(bool do_enable);
    

#ifdef __cplusplus
}
#endif

#endif

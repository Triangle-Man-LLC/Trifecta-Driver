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

    /// -- Platform-specific methods --
    /// -- Be sure to implement these when porting to a new platform --
    /// @brief Initializes a TCP network driver for the given device.
    /// @param device_handle Pointer to the device information structure.
    /// @return 0 on success, or a negative error code on failure.
    int fs_init_network_tcp_driver(fs_device_info *device_handle);

    /// @brief Initializes a UDP network driver for the given device.
    /// @param device_handle Pointer to the device information structure.
    /// @return 0 on success, or a negative error code on failure.
    int fs_init_network_udp_driver(fs_device_info *device_handle);

    /// @brief Initializes a serial communication driver for the given device.
    /// @param device_handle Pointer to the device information structure.
    /// @return 0 on success, or a negative error code on failure.
    int fs_init_serial_driver(fs_device_info *device_handle);

    /// @brief Starts a new thread for executing a specific function.
    /// @param thread_func Pointer to the thread's main function.
    /// @param params Parameters to be passed to the thread function.
    /// @param run_status_flag Pointer to a flag indicating the thread's run status.
    /// @param thread_handle Pointer to store the thread handle.
    /// @param stack_size Stack size for the thread (bytes).
    /// @param priority Priority of the thread.
    /// @param core_affinity Core affinity for the thread (-1 for no preference).
    /// @return 0 on success, or a negative error code on failure.
    int fs_thread_start(void(thread_func)(void *), void *params, fs_run_status *thread_running_flag, size_t stack_size, int priority, int core_affinity);
    
    /// @brief Exits the currently running thread.
    /// @param thread_handle Pointer to the thread handle. 
    /// Some platforms (e.g. FreeRTOS) allow forceful deletion of the thread using the handle, 
    /// while most POSIX and std::thread implementations do not. If unused, use NULL.
    /// @return 0 on success, or a negative error code on failure.
    int fs_thread_exit(void *thread_handle);

    /// @brief Sends data via TCP to the specified device.
    /// @param device_handle Pointer to the device information structure.
    /// @param tx_buffer Pointer to the data buffer to transmit.
    /// @param length_bytes Number of bytes to transmit.
    /// @param timeout_micros Timeout for the operation in microseconds.
    /// @return Number of bytes sent on success, or a negative error code on failure.
    ssize_t fs_transmit_networked_tcp(fs_device_info *device_handle, void *tx_buffer, size_t length_bytes, int timeout_micros);

    /// @brief Sends data via UDP to the specified device.
    /// @param device_handle Pointer to the device information structure.
    /// @param tx_buffer Pointer to the data buffer to transmit.
    /// @param length_bytes Number of bytes to transmit.
    /// @param timeout_micros Timeout for the operation in microseconds.
    /// @return Number of bytes sent on success, or a negative error code on failure.
    ssize_t fs_transmit_networked_udp(fs_device_info *device_handle, void *tx_buffer, size_t length_bytes, int timeout_micros);

    /// @brief Sends data via serial communication to the specified device.
    /// @param device_handle Pointer to the device information structure.
    /// @param tx_buffer Pointer to the data buffer to transmit.
    /// @param length_bytes Number of bytes to transmit.
    /// @param timeout_micros Timeout for the operation in microseconds.
    /// @return Number of bytes sent on success, or a negative error code on failure.
    ssize_t fs_transmit_serial(fs_device_info *device_handle, void *tx_buffer, size_t length_bytes, int timeout_micros);

    /// @brief Receives data via TCP from the specified device.
    /// @param device_list Pointer to the list of connected devices.
    /// @param device_handle Pointer to the device that received the message.
    /// @param rx_buffer Pointer to the buffer to store received data.
    /// @param length_bytes Number of bytes to receive.
    /// @param timeout_micros Timeout for the operation in microseconds.
    /// @return Number of bytes received on success, or a negative error code on failure.
    ssize_t fs_receive_networked_tcp(const fs_device_info device_list[FS_MAX_NUMBER_DEVICES], fs_device_info *device_handle, void *rx_buffer, size_t length_bytes, int timeout_micros);

    /// @brief Receives data via UDP from the specified device.
    /// @param device_list Pointer to the list of connected devices.
    /// @param device_handle Pointer to the device that received the message.
    /// @param rx_buffer Pointer to the buffer to store received data.
    /// @param length_bytes Number of bytes to receive.
    /// @param timeout_micros Timeout for the operation in microseconds.
    /// @return Number of bytes received on success, or a negative error code on failure.
    ssize_t fs_receive_networked_udp(const fs_device_info device_list[FS_MAX_NUMBER_DEVICES], fs_device_info *device_handle, void *rx_buffer, size_t length_bytes, int timeout_micros);

    /// @brief Receives data via serial communication from the specified device.
    /// @param device_handle Pointer to the device information structure.
    /// @param rx_buffer Pointer to the buffer to store received data.
    /// @param length_bytes Number of bytes to receive.
    /// @param timeout_micros Timeout for the operation in microseconds.
    /// @return Number of bytes received on success, or a negative error code on failure.
    ssize_t fs_receive_serial(fs_device_info *device_handle, void *rx_buffer, size_t length_bytes, int timeout_micros);

    /// @brief Shuts down the TCP network driver for the specified device.
    /// @param device_handle Pointer to the device information structure.
    /// @return 0 on success, or a negative error code on failure.
    int fs_shutdown_network_tcp_driver(fs_device_info *device_handle);

    /// @brief Shuts down the UDP network driver for the specified device.
    /// @param device_handle Pointer to the device information structure.
    /// @return 0 on success, or a negative error code on failure.
    int fs_shutdown_network_udp_driver(fs_device_info *device_handle);

    /// @brief Shuts down the serial communication driver for the specified device.
    /// @param device_handle Pointer to the device information structure.
    /// @return 0 on success, or a negative error code on failure.
    int fs_shutdown_serial_driver(fs_device_info *device_handle);

    /// @brief Logs a formatted message to the output stream.
    /// @param format Format string for the log message.
    /// @param ... Additional arguments for the format string.
    /// @return 0 on success, or a negative error code on failure.
    int fs_log_output(const char *format, ...);

    /// @brief Enables or disables logging.
    /// @param do_log Set to true to enable logging, false to disable it.
    /// @return 0 on success, or a negative error code on failure.
    int fs_toggle_logging(bool do_log);

    /// @brief Delays execution for a specified number of milliseconds.
    /// @param millis Number of milliseconds to delay.
    /// @return 0 on success, or a negative error code on failure.
    int fs_delay(int millis);

    /// @brief Delays execution until a specified time has elapsed.
    /// @param current_time Pointer to the current time value.
    /// @param millis Number of milliseconds to delay.
    /// @return 0 on success, or a negative error code on failure.
    int fs_delay_for(uint32_t *current_time, int millis);

    /// @brief Retrieves the current system time in milliseconds.
    /// @param current_time Pointer to store the current time.
    /// @return 0 on success, or a negative error code on failure.
    int fs_get_current_time(uint32_t *current_time);

#ifdef __cplusplus
}
#endif

#endif
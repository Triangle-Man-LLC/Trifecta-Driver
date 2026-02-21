/// Driver for the Trifecta series of IMU/AHRS/INS devices
/// Copyright 2025 4rge.ai and/or Triangle Man LLC
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
    int fs_init_network_tcp_driver(fs_device_info_t *device_handle);

    /// @brief Initializes a UDP network driver for the given device.
    /// @param device_handle Pointer to the device information structure.
    /// @return 0 on success, or a negative error code on failure.
    int fs_init_network_udp_driver(fs_device_info_t *device_handle);

    /// @brief Initializes a serial communication driver for the given device.
    /// @param device_handle Pointer to the device information structure.
    /// @return 0 on success, or a negative error code on failure.
    int fs_init_serial_driver(fs_device_info_t *device_handle);

    /// @brief Whether interrupt-driven UART etc. is supported by the platform.
    /// Many RTOSes support this, but Linux does not, etc.
    /// @return OR FLAG of serial interfaces which support interrupts.
    /// If the return is FS_COMMUNICATION_MODE_UNINITIALIZED, then no interrupt-driven serial is allowed.
    int fs_platform_supported_serial_interrupts();

    /// @brief Start serial in interrupt mode on platforms that support it.
    /// This enables more precise and low latency serial reads than polling.
    /// It works by using the DRDY notification of the GPIO.
    /// @param device_handle
    /// @return 0 on success, -1 on fail (e.g. not supported on platform)
    int fs_init_serial_interrupts(fs_device_info_t *device_handle);

    /// @brief Wait for the next serial interrupt on the device handle.
    /// This will yield the task until the interrupt has occurred.
    /// @param device_handle
    /// @return 0 on success, -1 on fail (e.g. not supported on platform)
    int fs_wait_until_next_serial_interrupt(fs_device_info_t *device_handle);

    /// @brief Platform-specific start thread given a function handle.
    /// @param thread_func Pointer to the thread function handle.
    /// @param params Parameters to pass to the thread function.
    /// @param thread_running_flag Pointer to the flag used to indicate thread status.
    /// @param thread_handle_ptr Pointer to the task handle.
    /// @param stack_size Size of the stack allocated for the thread.
    /// @param priority Priority level of the thread.
    /// @param core_affinity -1 for indifference, else preferred core number
    /// @return Status of the thread creation (0 for success, -1 for failure).
    int fs_thread_start(fs_thread_func_t (*thread_func)(void *), void *params, fs_run_status_t *thread_running_flag, fs_thread_t *thread_handle, size_t stack_size, int priority, int core_affinity);

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
    ssize_t fs_transmit_networked_tcp(fs_device_info_t *device_handle, void *tx_buffer, size_t length_bytes, int timeout_micros);

    /// @brief Sends data via UDP to the specified device.
    /// @param device_handle Pointer to the device information structure.
    /// @param tx_buffer Pointer to the data buffer to transmit.
    /// @param length_bytes Number of bytes to transmit.
    /// @param timeout_micros Timeout for the operation in microseconds.
    /// @return Number of bytes sent on success, or a negative error code on failure.
    ssize_t fs_transmit_networked_udp(fs_device_info_t *device_handle, void *tx_buffer, size_t length_bytes, int timeout_micros);

    /// @brief Sends data via serial communication to the specified device.
    /// @param device_handle Pointer to the device information structure.
    /// @param tx_buffer Pointer to the data buffer to transmit.
    /// @param length_bytes Number of bytes to transmit.
    /// @param timeout_micros Timeout for the operation in microseconds.
    /// @return Number of bytes sent on success, or a negative error code on failure.
    ssize_t fs_transmit_serial(fs_device_info_t *device_handle, void *tx_buffer, size_t length_bytes, int timeout_micros);

    /// @brief Receives data via TCP from the specified device.
    /// @param device_list Pointer to the list of connected devices.
    /// @param device_handle Pointer to the device that received the message.
    /// @param rx_buffer Pointer to the buffer to store received data.
    /// @param length_bytes Number of bytes to receive.
    /// @param timeout_micros Timeout for the operation in microseconds.
    /// @return Number of bytes received on success, or a negative error code on failure.
    ssize_t fs_receive_networked_tcp(fs_device_info_t *device_handle, void *rx_buffer, size_t length_bytes, int timeout_micros);

    /// @brief Receives data via UDP from the specified device.
    /// @param device_list Pointer to the list of connected devices.
    /// @param device_handle Pointer to the device that received the message.
    /// @param rx_buffer Pointer to the buffer to store received data.
    /// @param length_bytes Number of bytes to receive.
    /// @param timeout_micros Timeout for the operation in microseconds.
    /// @return Number of bytes received on success, or a negative error code on failure.
    ssize_t fs_receive_networked_udp(fs_device_info_t *device_handle, void *rx_buffer, size_t length_bytes, int timeout_micros);

    /// @brief Receives data via serial communication from the specified device.
    /// @param device_handle Pointer to the device information structure.
    /// @param rx_buffer Pointer to the buffer to store received data.
    /// @param length_bytes Number of bytes to receive.
    /// @param timeout_micros Timeout for the operation in microseconds.
    /// @return Number of bytes received on success, or a negative error code on failure.
    ssize_t fs_receive_serial(fs_device_info_t *device_handle, void *rx_buffer, size_t length_bytes, int timeout_micros);

    /// @brief Shuts down the TCP network driver for the specified device.
    /// @param device_handle Pointer to the device information structure.
    /// @return 0 on success, or a negative error code on failure.
    int fs_shutdown_network_tcp_driver(fs_device_info_t *device_handle);

    /// @brief Shuts down the UDP network driver for the specified device.
    /// @param device_handle Pointer to the device information structure.
    /// @return 0 on success, or a negative error code on failure.
    int fs_shutdown_network_udp_driver(fs_device_info_t *device_handle);

    /// @brief Shuts down the serial communication driver for the specified device.
    /// @param device_handle Pointer to the device information structure.
    /// @return 0 on success, or a negative error code on failure.
    int fs_shutdown_serial_driver(fs_device_info_t *device_handle);

    /// @brief Attempts to reconnect the TCP connection for the specified device.
    /// @param device_handle Pointer to the device information structure.
    /// @return 0 on success, or a negative error code on failure.
    int fs_attempt_reconnect_network_tcp(fs_device_info_t *device_handle);

    /// @brief Attempts to reconnect the UDP connection for the specified device.
    /// @param device_handle Pointer to the device information structure.
    /// @return 0 on success, or a negative error code on failure.
    int fs_attempt_reconnect_network_udp(fs_device_info_t *device_handle);

    /// @brief Attempts to reconnect the serial communication for the specified device.
    /// @param device_handle Pointer to the device information structure.
    /// @return 0 on success, or a negative error code on failure.
    int fs_attempt_reconnect_serial(fs_device_info_t *device_handle);

    /// @brief Logs a formatted message to the output stream.
    /// @param format Format string for the log message.
    /// @param ... Additional arguments for the format string.
    /// @return 0 on success, or a negative error code on failure.
    int fs_log_output(const char *format, ...);

    /// @brief Logs a formatted message to the output stream,
    /// even if logging is disabled.
    /// @param format Format string for the log message.
    /// @param ... Additional arguments for the format string.
    /// @return 0 on success, or a negative error code on failure.
    int fs_log_critical(const char *format, ...);

    /// @brief Enables or disables logging.
    /// @param do_log Set to true to enable logging, false to disable it.
    /// @return 0 on success, or a negative error code on failure.
    int fs_toggle_logging(bool do_log);

    /// @brief Redirect logs to the indicated path.
    /// Only some platforms support this. (E.g. a filesystem needed.)
    /// @param context The path to store logs into.
    /// @return 0 on success, or a negative error code on failure.
    int fs_set_log_location(const char *directory);

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
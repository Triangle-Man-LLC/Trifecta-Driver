/// Driver for the Trifecta series of IMU/AHRS/INS devices
/// Copyright 2024 4rge.ai and/or Triangle Man LLC
/// Usage and redistribution of this code is permitted
/// but this notice must be retained in all copies of the code.

/// THIS SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
/// INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE,
/// AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
/// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include <lwip/netdb.h>

#include "FS_Trifecta_Interfaces.h"

// Platform-specific: Functions for initializing communication drivers on target platform

int fs_logging_level = 0; // Logging level - 0 = OFF, 1 = ON

/// @brief Start the network TCP driver.
/// @param device_handle Pointer to the device information structure
/// @return 0 on success, -1 on failure
int fs_init_network_tcp_driver(fs_device_info *device_handle)
{
    if (device_handle == NULL || device_handle->ip_addr[0] == '\0')
    {
        fs_log_output("[Trifecta] Error: Invalid device handle or IP address!\n");
        return -1;
    }

    // Convert IP address string to binary form
    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(device_handle->ip_port);

    if (inet_pton(AF_INET, device_handle->ip_addr, &server_addr.sin_addr) <= 0)
    {
        fs_log_output("[Trifecta] Error: Invalid IP address format!\n");
        return -1;
    }

    // Create TCP socket
    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0)
    {
        fs_log_output("[Trifecta] Error: Could not create TCP socket!\n");
        return -1;
    }

    // Connect to the device
    if (connect(sockfd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
    {
        fs_log_output("[Trifecta] Error: Could not connect to device!\n");
        close(sockfd);
        return -1;
    }

    device_handle->tcp_sock = sockfd;
    return 0;
}

/// @brief Start the network UDP driver.
/// @param device_handle Pointer to the device information structure
/// @return 0 on success, -1 on failure
int fs_init_network_udp_driver(fs_device_info *device_handle)
{
    if (device_handle == NULL || device_handle->ip_addr[0] == '\0')
    {
        fs_log_output("[Trifecta] Error: Invalid device handle or IP address!\n");
        return -1;
    }

    // Convert IP address string to binary form
    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(device_handle->ip_port);

    if (inet_pton(AF_INET, device_handle->ip_addr, &server_addr.sin_addr) <= 0)
    {
        fs_log_output("[Trifecta] Error: Invalid IP address format!\n");
        return -1;
    }

    // Create UDP socket
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0)
    {
        fs_log_output("[Trifecta] Error: Could not create UDP socket!\n");
        return -1;
    }

    device_handle->udp_sock = sockfd;
    return 0;
}

/// @brief Start the network serial driver.
/// @param device_handle
/// @return
int fs_init_serial_driver(fs_device_info *device_handle)
{
    // On FreeRTOS/microcontroller systems, the serial port is usually a fixed number, so port scanning will not be done
    // Only check to ensure that the serial port was previously set up
    if (device_handle->serial_port < 0)
    {
        fs_log_output("[Trifecta] Serial port number cannot be less than zero!");
        return -1;
    }
    return 0;
}

/// @brief Platform-specific start thread given a function handle.
/// @param thread_func Pointer to the thread function handle.
/// @param params Parameters to pass to the thread function.
/// @param thread_running_flag Pointer to the flag used to indicate thread status.
/// @param stack_size Size of the stack allocated for the thread.
/// @param priority Priority level of the thread.
/// @param core_affinity -1 for indifference, else preferred core number
/// @return Status of the thread creation (0 for success, -1 for failure).
/// @brief Platform-specific start thread given a function handle.
/// @param thread_func Pointer to the thread function handle.
/// @param params Parameters to pass to the thread function.
/// @param thread_running_flag Pointer to the flag used to indicate thread status.
/// @param stack_size Size of the stack allocated for the thread.
/// @param priority Priority level of the thread.
/// @param core_affinity -1 for indifference, else preferred core number
/// @return Status of the thread creation (0 for success, -1 for failure).
int fs_thread_start(void(thread_func)(void *), void *params, bool *thread_running_flag, size_t stack_size, int priority, int core_affinity)
{
    if (thread_func == NULL || thread_running_flag == NULL)
    {
        fs_log_output("[Trifecta] Error: Invalid thread function or running flag!\n");
        return -1;
    }

    else if(thread_running_flag != NULL && *thread_running_flag != 0)
    {
        fs_log_output("[Trifecta] Warning: Thread start aborted, thread was already running!\n");
        return 0; // Thread already running
    }

    // Ensure the flag is set to indicate the thread is running
    *thread_running_flag = true;

    TaskHandle_t task_handle = NULL;
    BaseType_t result;

    // Create the FreeRTOS task
    if (core_affinity < 0)
    {
        result = xTaskCreate(thread_func, "ThreadTask", stack_size, params, priority, &task_handle);
    }
    else
    {
        result = xTaskCreatePinnedToCore(thread_func, "ThreadTask", stack_size, params, priority, &task_handle, core_affinity);
    }

    if (result != pdPASS)
    {
        fs_log_output("[Trifecta] Error: Task creation failed!\n");
        *thread_running_flag = 0;
        return -1;
    }

    return 0;
}

/// @brief Some platforms (e.g. FreeRTOS) require thread exit to be properly handled.
/// This function should implement that behavior.
/// @return Should always return 0...
int fs_thread_exit(void *thread_handle)
{
    if (thread_handle == NULL)
    {
        vTaskDelete(NULL);
        return 0;
    }
    vTaskDelete(*(TaskHandle_t *)thread_handle);
    return 0; // Success
}

/// @brief Transmit data over a networked TCP connection
/// @param device_handle Pointer to the device information structure
/// @param tx_buffer Pointer to the transmit data buffer
/// @param length_bytes The size of the tx_buffer
/// @param timeout_micros The max amount of time to wait (microseconds)
/// @return -1 if failed, else number of bytes written
ssize_t fs_transmit_networked_tcp(fs_device_info *device_handle, void *tx_buffer, size_t length_bytes, int timeout_micros)
{
    if (device_handle == NULL)
    {
        fs_log_output("[Trifecta] Error: Device handle is NULL!");
        return -1;
    }

    if (device_handle->communication_mode != FS_COMMUNICATION_MODE_TCP_UDP)
    {
        fs_log_output("[Trifecta] Error: Invalid communication mode! Expected COMMUNICATION_MODE_TCP_UDP.");
        return -1;
    }

    if (device_handle->tcp_sock < 0)
    {
        fs_log_output("[Trifecta] Error: Invalid TCP socket!");
        return -1;
    }

    if (tx_buffer == NULL)
    {
        fs_log_output("[Trifecta] Error: Transmit buffer is NULL!");
        return -1;
    }

    // Set the send timeout
    struct timeval timeout;
    timeout.tv_sec = timeout_micros / 1000000;
    timeout.tv_usec = timeout_micros % 1000000;
    if (setsockopt(device_handle->tcp_sock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout)) < 0)
    {
        fs_log_output("[Trifecta] Error: Could not set send timeout!");
        return -1;
    }

    int written = send(device_handle->tcp_sock, tx_buffer, length_bytes, 0);

    if (written < 0)
    {
        fs_log_output("[Trifecta] Error: Sending data over TCP failed!");
    }

    return written;
}

/// @brief Transmit data over a networked UDP connection
/// @param device_handle Pointer to the device information structure
/// @param tx_buffer Pointer to the transmit data buffer
/// @param length_bytes The size of the tx_buffer
/// @param timeout_micros The max amount of time to wait (microseconds)
/// @return -1 if failed, else number of bytes written
ssize_t fs_transmit_networked_udp(fs_device_info *device_handle, void *tx_buffer, size_t length_bytes, int timeout_micros)
{
    if (device_handle == NULL)
    {
        fs_log_output("[Trifecta] Error: Device handle is NULL!");
        return -1;
    }

    if (device_handle->communication_mode != FS_COMMUNICATION_MODE_TCP_UDP)
    {
        fs_log_output("[Trifecta] Error: Invalid communication mode! Expected COMMUNICATION_MODE_TCP_UDP.");
        return -1;
    }

    if (device_handle->udp_sock < 0)
    {
        fs_log_output("[Trifecta] Error: Invalid UDP socket!");
        return -1;
    }

    if (tx_buffer == NULL)
    {
        fs_log_output("[Trifecta] Error: Transmit buffer is NULL!");
        return -1;
    }

    // Set the send timeout
    struct timeval timeout;
    timeout.tv_sec = timeout_micros / 1000000;
    timeout.tv_usec = timeout_micros % 1000000;
    if (setsockopt(device_handle->udp_sock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout)) < 0)
    {
        fs_log_output("[Trifecta] Error: Could not set send timeout!");
        return -1;
    }

    int written = send(device_handle->udp_sock, tx_buffer, length_bytes, 0);

    if (written < 0)
    {
        fs_log_output("[Trifecta] Error: Sending data over UDP failed!");
    }

    return written;
}

/// @brief Transmit data over serial communication
/// @param device_handle Pointer to the device information structure
/// @param tx_buffer Pointer to the transmit data buffer
/// @param length_bytes The size of the tx_buffer
/// @param timeout_micros The max amount of time to wait (microseconds)
/// @return -1 if failed, else number of bytes written
ssize_t fs_transmit_serial(fs_device_info *device_handle, void *tx_buffer, size_t length_bytes, int timeout_micros)
{
    if (device_handle == NULL)
    {
        fs_log_output("[Trifecta] Error: Device handle is NULL!");
        return -1;
    }

    if (device_handle->communication_mode != FS_COMMUNICATION_MODE_SERIAL)
    {
        fs_log_output("[Trifecta] Error: Invalid communication mode! Expected COMMUNICATION_MODE_SERIAL.");
        return -1;
    }

    if (device_handle->serial_port < 0)
    {
        fs_log_output("[Trifecta] Error: Invalid serial port!");
        return -1;
    }

    if (tx_buffer == NULL)
    {
        fs_log_output("[Trifecta] Error: Transmit buffer is NULL!");
        return -1;
    }

    ssize_t actual_len = uart_write_bytes(device_handle->serial_port, (char *)tx_buffer, length_bytes);
    if (actual_len != length_bytes)
    {
        fs_log_output("[Trifecta] Error: Writing data over serial failed! Expected length: %ld, actual: %ld", length_bytes, actual_len);
        return -1;
    }

    fs_log_output("[Trifecta] Serial transmit to port %d - Length %ld - data %s", device_handle->serial_port, actual_len, tx_buffer);

    return actual_len;
}

/// @brief Receive data over a networked TCP connection
/// @param device_handle Pointer to the device information structure
/// @param rx_buffer Pointer to the receive data buffer
/// @param length_bytes The max size of the rx_buffer
/// @param timeout_micros The max amount of time to wait (microseconds)
/// @return -1 if failed, else number of bytes received
ssize_t fs_receive_networked_tcp(fs_device_info *device_handle, void *rx_buffer, size_t length_bytes, int timeout_micros)
{
    if (device_handle == NULL)
    {
        fs_log_output("[Trifecta] Error: Device handle is NULL!");
        return -1;
    }

    if (device_handle->communication_mode != FS_COMMUNICATION_MODE_TCP_UDP)
    {
        fs_log_output("[Trifecta] Error: Invalid communication mode! Expected COMMUNICATION_MODE_TCP_UDP.");
        return -1;
    }

    if (device_handle->tcp_sock < 0)
    {
        fs_log_output("[Trifecta] Error: Invalid TCP socket!");
        return -1;
    }

    if (rx_buffer == NULL)
    {
        fs_log_output("[Trifecta] Error: Receive buffer is NULL!");
        return -1;
    }

    // Set the receive timeout
    struct timeval timeout;
    timeout.tv_sec = timeout_micros / 1000000;
    timeout.tv_usec = timeout_micros % 1000000;
    if (setsockopt(device_handle->tcp_sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0)
    {
        fs_log_output("[Trifecta] Error: Could not set receive timeout (TCP)!");
        return -1;
    }

    ssize_t recv_len = recv(device_handle->tcp_sock, rx_buffer, length_bytes, 0);

    if (recv_len < 0)
    {
        fs_log_output("[Trifecta] Error: Receiving data over TCP failed!");
    }

    return recv_len;
}

/// @brief Receive data over a networked UDP connection
/// @param device_handle Pointer to the device information structure
/// @param rx_buffer Pointer to the receive data buffer
/// @param length_bytes The max size of the rx_buffer
/// @param timeout_micros The max amount of time to wait (microseconds)
/// @return -1 if failed, else number of bytes received
ssize_t fs_receive_networked_udp(fs_device_info *device_handle, void *rx_buffer, size_t length_bytes, int timeout_micros)
{
    if (device_handle == NULL)
    {
        fs_log_output("[Trifecta] Error: Device handle is NULL!");
        return -1;
    }

    if (device_handle->communication_mode != FS_COMMUNICATION_MODE_TCP_UDP)
    {
        fs_log_output("[Trifecta] Error: Invalid communication mode! Expected COMMUNICATION_MODE_TCP_UDP.");
        return -1;
    }

    if (device_handle->udp_sock < 0)
    {
        fs_log_output("[Trifecta] Error: Invalid UDP socket!");
        return -1;
    }

    if (rx_buffer == NULL)
    {
        fs_log_output("[Trifecta] Error: Receive buffer is NULL!");
        return -1;
    }

    // Set the receive timeout
    struct timeval timeout;
    timeout.tv_sec = timeout_micros / 1000000;
    timeout.tv_usec = timeout_micros % 1000000;
    if (setsockopt(device_handle->udp_sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0)
    {
        fs_log_output("[Trifecta] Error: Could not set receive timeout (UDP)!");
        return -1;
    }

    ssize_t recv_len = recv(device_handle->udp_sock, rx_buffer, length_bytes, 0);

    if (recv_len < 0)
    {
        fs_log_output("[Trifecta] Error: Receiving data over UDP failed!");
    }

    return recv_len;
}

/// @brief Receive data over serial communication
/// @param device_handle Pointer to the device information structure
/// @param rx_buffer Pointer to the receive data buffer (this buffer is cleared on read)
/// @param length_bytes The max size of the rx_buffer
/// @param timeout_micros The max amount of time to wait (microseconds)
/// @return -1 if failed, else number of bytes received
ssize_t fs_receive_serial(fs_device_info *device_handle, void *rx_buffer, size_t length_bytes, int timeout_micros)
{
    if (device_handle == NULL)
    {
        fs_log_output("[Trifecta] Error: Device handle is NULL!");
        return -1;
    }

    if (device_handle->communication_mode != FS_COMMUNICATION_MODE_SERIAL)
    {
        fs_log_output("[Trifecta] Error: Invalid communication mode! Expected COMMUNICATION_MODE_SERIAL.");
        return -1;
    }

    if (device_handle->serial_port < 0)
    {
        fs_log_output("[Trifecta] Error: Invalid serial port!");
        return -1;
    }

    if (rx_buffer == NULL)
    {
        fs_log_output("[Trifecta] Error: Receive buffer is NULL!");
        return -1;
    }
    // Clear only the specified buffer size
    memset(&rx_buffer, 0, length_bytes);

    size_t buffer_data_len = 0;
    if (uart_get_buffered_data_len(device_handle->serial_port, (size_t *)&buffer_data_len) != 0)
    {
        fs_log_output("[Trifecta] Error: Could not get buffered data length!");
        return -1; // In this case, no data in buffer, so do nothing.
    }

    if (buffer_data_len > length_bytes)
    {
        fs_log_output("[Trifecta] Warning: Data length %ld exceeds buffer size %ld. Truncating data.",
                      buffer_data_len, length_bytes);
        buffer_data_len = length_bytes; // Read only up to buffer size
    }

    ssize_t rx_len = uart_read_bytes(device_handle->serial_port, rx_buffer, buffer_data_len, timeout_micros / 1000);
    // ssize_t rx_len = uart_read_bytes(device_handle->serial_port, rx_buffer, FS_MAX_DATA_LENGTH, timeout_micros / 1000);

    if (rx_len > 0)
    {
        fs_log_output("[Trifecta] Read data from port %d - length %d!", device_handle->serial_port, rx_len);
    }
    else if (rx_len < 0)
    {
        fs_log_output("[Trifecta] Error: Reading data over serial failed!");
        uart_flush(device_handle->serial_port);
        return -1;
    }

    return rx_len;
}

/// @brief Shutdown the network TCP driver.
/// @param device_handle Pointer to the device information structure.
/// @return 0 if successful, -1 if failed.
int fs_shutdown_network_tcp_driver(fs_device_info *device_handle)
{
    if (close(device_handle->tcp_sock) != 0)
    {
        fs_log_output("[Trifecta] Warning: Failed to close TCP socket (socket: %d)!", device_handle->tcp_sock);
        device_handle->tcp_sock = -1;
        return -1;
    }
    device_handle->tcp_sock = -1;
    return 0;
}

/// @brief Shutdown the network UDP driver.
/// @param device_handle Pointer to the device information structure.
/// @return 0 if successful, -1 if failed.
int fs_shutdown_network_udp_driver(fs_device_info *device_handle)
{
    if (close(device_handle->udp_sock) != 0)
    {
        fs_log_output("[Trifecta] Warning: Failed to close UDP socket (socket: %d)!", device_handle->udp_sock);
        device_handle->udp_sock = -1;
        return -1;
    }
    device_handle->udp_sock = -1;
    return 0;
}

/// @brief Shutdown the serial driver.
/// @param device_handle Pointer to the device information structure.
/// @return 0 if successful, -1 if failed.
int fs_shutdown_serial_driver(fs_device_info *device_handle)
{
    if (uart_driver_delete(device_handle->serial_port) != 0)
    {
        fs_log_output("[Trifecta] Warning: Failed to delete UART driver (serial port: %d)!", device_handle->serial_port);
        device_handle->serial_port = -1;
        return -1;
    }
    device_handle->serial_port = -1;
    return 0;
}

/// @brief Logs output with formatting.
/// @param format Format string.
/// @param ... Additional arguments.
/// @return Number of characters printed.
int fs_log_output(const char *format, ...)
{
    int chars_printed = 0;

    if (fs_logging_level > 0)
    {
        va_list args;
        va_start(args, format);

        // Print formatted string
        chars_printed = vprintf(format, args);

        // Check if the last character is a newline
        if (format[chars_printed - 1] != '\n')
        {
            printf("\n");
            chars_printed++;
        }

        va_end(args);
    }

    return chars_printed;
}

/// @brief Toggle logging (you may want to turn it off in some systems to avoid flooding the serial output)
/// @param do_log TRUE to turn log on, FALSE to turn log off
/// @return 1 if logging turned on, 0 if logging turned off
int fs_toggle_logging(bool do_log)
{
    fs_logging_level = do_log ? 1 : 0;
    return fs_logging_level;
}

/// @brief Delay by at least this amount of time
/// @param millis Number of milliseconds to delay
/// @return The number of ticks the delay lasted
int fs_delay(int millis)
{
    vTaskDelay(pdMS_TO_TICKS(millis));
    return pdMS_TO_TICKS(millis);
}

/// @brief Real-time delay
/// @param current_time Pointer to the current time
/// @param millis The exact amount of time to delay
/// @return The number of ticks the delay lasted
int fs_delay_for(uint32_t *current_time, int millis)
{
    if (*current_time == 0)
    {
        *current_time = xTaskGetTickCount();
    }
    uint32_t initial_time = *current_time;
    vTaskDelayUntil(current_time, pdMS_TO_TICKS(millis));
    uint32_t elapsed_ticks = *current_time - initial_time;
    return (int)elapsed_ticks;
}

/// @brief Get the current system time
/// @param current_time Pointer to the current time
/// @param millis The exact amount of time to delay
/// @return 0 on success
int fs_get_current_time(uint32_t *current_time)
{
    *current_time = xTaskGetTickCount();
    return 0;
}
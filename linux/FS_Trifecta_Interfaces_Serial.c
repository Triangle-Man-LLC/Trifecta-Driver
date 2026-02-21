/// Generic driver for the Trifecta series of IMU/AHRS/INS devices.
/// Copyright 2025 4rge.ai and/or Triangle Man LLC
/// Usage and redistribution of this code is permitted
/// but this notice must be retained in all copies of the code.

/// THIS SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
/// INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE,
/// AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
/// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#define _GNU_SOURCE /* See feature_test_macros(7) */

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>

#include <pthread.h>
#include <sched.h>

#include <string.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/select.h>
#include <sys/ioctl.h>

#include <fcntl.h>
#include <termios.h>
#include <glob.h>

#include "FS_Trifecta_Interfaces.h"

// Platform-specific: Functions for initializing communication drivers on target platform

#define FS_TRIFECTA_SERIAL_BAUDRATE_POSIX B2000000

/// @brief Configure serial port settings
/// @param fd The file descriptor for the serial port
/// @return 0 if successful, -1 if failed
static inline int configure_serial_port(int fd)
{
    struct termios options;

    if (fd < 2)
    {
        fs_log_output("[Trifecta] Cannot configure invalid FD %d! Disallowed: 0, 1, 2.", fd);
    }

    // Get the current options for the port
    if (tcgetattr(fd, &options) != 0)
    {
        fs_log_output("[Trifecta] Error: Failed to get serial port attributes: %s", strerror(errno));
        return -1;
    }

    cfsetispeed(&options, FS_TRIFECTA_SERIAL_BAUDRATE_POSIX);

    // Set 8 data bits, no parity, 1 stop bit (8N1)
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;

    // No hardware flow control
    options.c_cflag &= ~CRTSCTS;

    // No software flow control
    options.c_iflag &= ~(IXON | IXOFF | IXANY);

    // No output post-processing (for supporting binary transmissions)
    options.c_oflag &= ~OPOST;

    /* set input mode (non-canonical, no echo,...) */
    options.c_lflag = 0;
    // options.c_iflag &= ~(ICRNL | INLCR | IGNCR);

    options.c_cc[VTIME] = 10; /* inter-character timer 1 sec */
    options.c_cc[VMIN] = 0;   /* blocking read disabled  */

    tcflush(fd, TCIFLUSH);
    // Set the options for the port
    if (tcsetattr(fd, TCSANOW, &options) != 0)
    {
        fs_log_output("[Trifecta] Error: Failed to set serial port attributes: %s", strerror(errno));
        return -1;
    }

    return 0;
}

/// @brief Start the network serial driver.
/// @param device_handle Pointer to the device information structure
/// @return 0 if successful, -1 if failed
int fs_init_serial_driver(fs_device_info_t *device_handle)
{
    if (device_handle == NULL)
    {
        fs_log_critical("[Trifecta] Could not start serial driver: Device handle is NULL!");
        return -1;
    }

    if (device_handle->device_params.status == FS_RUN_STATUS_RUNNING)
    {
        fs_log_critical("[Trifecta] Could not start serial driver: Device is already initialized!");
        return -1;
    }

    size_t expected_path_len = fs_safe_strnlen((const char *)device_handle->device_params.serial_port,
                                               sizeof(device_handle->device_params.serial_path));
    if (expected_path_len == 0 || expected_path_len >= sizeof(device_handle->device_params.serial_path))
    {
        fs_log_critical("[Trifecta] Path length invalid: %d (must be within range (0, 127]!", expected_path_len);
        return -1;
    }

    fs_safe_strncpy(device_handle->device_params.serial_path,
                    (const char *)device_handle->device_params.serial_port,
                    sizeof(device_handle->device_params.serial_path));

    // Linux-specific: Store the serial path, and then also the file descriptor as well.
    device_handle->device_params.serial_port = open(device_handle->device_params.serial_path, O_RDWR | O_NOCTTY);

    if (device_handle->device_params.serial_port < 2)
    {
        fs_log_critical("[Trifecta] Could not start serial driver: open() returned an invalid FD (%d)", device_handle->device_params.serial_port);
        device_handle->device_params.serial_port = -1;
        return -1;
    }

    // Configure the serial port
    if (configure_serial_port(device_handle->device_params.serial_port) != 0)
    {
        fs_log_critical("[Trifecta] Could not start serial driver: Failed to configure port: %d (Errno: %d)", device_handle->device_params.serial_port, errno);
        close(device_handle->device_params.serial_port);
        return -1;
    }
    return 0;
}

/// @brief Whether interrupt-driven UART etc. is supported by the platform.
/// Many RTOSes support this, but Linux does not, etc.
/// @return OR FLAG of serial interfaces which support interrupts.
/// If the return is FS_COMMUNICATION_MODE_UNINITIALIZED, then no interrupt-driven serial is allowed.
int fs_platform_supported_serial_interrupts()
{
    return FS_COMMUNICATION_MODE_UNINITIALIZED; // Not supported!
}

/// @brief Start serial in interrupt mode on platforms that support it.
/// This enables more precise and low latency serial reads than polling.
/// @param device_handle
/// @param status_flag
/// @return 0 on success, -1 on fail (e.g. not supported on platform)
int fs_init_serial_interrupts(fs_device_info_t *device_handle)
{
    return -1; // Not supported!
}

/// @brief Wait for the next serial interrupt on the device handle.
/// This will yield the task until the interrupt has occurred.
/// @param device_handle
/// @return 0 on success, -1 on fail (e.g. not supported on platform)
int fs_wait_until_next_serial_interrupt(fs_device_info_t *device_handle)
{
    return -1; // Not supported!
}


/// @brief Transmit data over serial communication
/// @param device_handle Pointer to the device information structure
/// @param tx_buffer Pointer to the transmit data buffer
/// @param length_bytes The size of the tx_buffer
/// @param timeout_micros The max amount of time to wait (microseconds)
/// @return -1 if failed, else number of bytes written
ssize_t fs_transmit_serial(fs_device_info_t *device_handle, void *tx_buffer, size_t length_bytes, int timeout_micros)
{
    if (device_handle == NULL)
    {
        fs_log_output("[Trifecta] Error: Device handle is NULL!");
        return -1;
    }

    if (device_handle->device_params.communication_mode != FS_COMMUNICATION_MODE_UART && device_handle->device_params.communication_mode != FS_COMMUNICATION_MODE_USB_CDC)
    {
        fs_log_output("[Trifecta] Error: Invalid communication mode! Expected FS_COMMUNICATION_MODE_SERIAL.");
        return -1;
    }

    if (device_handle->device_params.serial_port < 0)
    {
        fs_log_output("[Trifecta] Error: Invalid serial port!");
        return -1;
    }

    if (tx_buffer == NULL)
    {
        fs_log_output("[Trifecta] Error: Transmit buffer is NULL!");
        return -1;
    }

    // Set up the timeout using select
    struct timeval timeout;
    timeout.tv_sec = timeout_micros / 1000000;
    timeout.tv_usec = timeout_micros % 1000000;

    fd_set write_fds;
    FD_ZERO(&write_fds);
    FD_SET(device_handle->device_params.serial_port, &write_fds);

    int select_result = select(device_handle->device_params.serial_port + 1, NULL, &write_fds, NULL, &timeout);
    if (select_result == -1)
    {
        fs_log_output("[Trifecta] Error: select() failed! Error: %s", strerror(errno));
        return -1;
    }
    else if (select_result == 0)
    {
        fs_log_output("[Trifecta] Error: Write timeout reached!");
        return -1;
    }

    ssize_t actual_len = write(device_handle->device_params.serial_port, tx_buffer, length_bytes);
    if (actual_len == -1)
    {
        fs_log_output("[Trifecta] Error: Writing data over serial failed! Error: %s", strerror(errno));
        return -1;
    }

    fs_log_output("[Trifecta] Serial transmit to port %d - Length %ld", device_handle->device_params.serial_port, (long)actual_len);

    return actual_len;
}

/// @brief Receive data over serial communication
/// @param device_handle Pointer to the device information structure
/// @param rx_buffer Pointer to the receive data buffer
/// @param length_bytes The max size of the rx_buffer
/// @param timeout_micros The max amount of time to wait (microseconds)
/// @return -1 if failed, else number of bytes received
ssize_t fs_receive_serial(fs_device_info_t *device_handle, void *rx_buffer, size_t length_bytes, int timeout_micros)
{
    if (device_handle == NULL)
    {
        fs_log_output("[Trifecta] Error: Device handle is NULL!");
        return -1;
    }

    if (device_handle->device_params.communication_mode != FS_COMMUNICATION_MODE_UART && device_handle->device_params.communication_mode != FS_COMMUNICATION_MODE_USB_CDC)
    {
        fs_log_output("[Trifecta] Error: Invalid communication mode! Expected FS_COMMUNICATION_MODE_SERIAL.");
        return -1;
    }

    if (device_handle->device_params.serial_port < 0)
    {
        fs_log_output("[Trifecta] Error: Invalid serial port!");
        return -1;
    }

    if (rx_buffer == NULL)
    {
        fs_log_output("[Trifecta] Error: Receive buffer is NULL!");
        return -1;
    }

    // Set up the timeout using select
    struct timeval timeout;
    timeout.tv_sec = timeout_micros / 1000000;
    timeout.tv_usec = timeout_micros % 1000000;

    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(device_handle->device_params.serial_port, &read_fds);

    int select_result = select(device_handle->device_params.serial_port + 1, &read_fds, NULL, NULL, &timeout);
    if (select_result == -1)
    {
        fs_log_critical("[Trifecta] Error: select() failed! Error: %s", strerror(errno));
        return -1;
    }
    else if (select_result == 0)
    {
        fs_log_output("[Trifecta] Error: Read timeout reached!");
        return 0;
    }

    // Check how much data is available
    int buffer_data_len = 0;
    if (ioctl(device_handle->device_params.serial_port, FIONREAD, &buffer_data_len) < 0)
    {
        fs_log_output("[Trifecta] Error: Could not get buffered data length! Error: %s", strerror(errno));
        return -1;
    }

    if (buffer_data_len > length_bytes)
    {
        fs_log_output("[Trifecta] Error: Data of length %d would overflow buffer size %ld!", buffer_data_len, length_bytes);
        tcflush(device_handle->device_params.serial_port, TCIFLUSH); // Flush the input buffer
        return -1;
    }

    ssize_t rx_len = read(device_handle->device_params.serial_port, rx_buffer, buffer_data_len);
    if (rx_len < 0)
    {
        fs_log_output("[Trifecta] Error: Reading data over serial failed! Error: %s", strerror(errno));
        tcflush(device_handle->device_params.serial_port, TCIFLUSH); // Flush the input buffer
        return -1;
    }
    else if (rx_len > 0)
    {
        fs_log_output("[Trifecta] Read data from port %d - length %zd!", device_handle->device_params.serial_port, rx_len);
        // tcflush(device_handle->device_params.serial_port, TCIFLUSH); // Flush the input buffer
    }

    return rx_len;
}

/// @brief Shutdown the serial driver.
/// @param device_handle Pointer to the device information structure.
/// @return 0 if successful, -1 if failed.
int fs_shutdown_serial_driver(fs_device_info_t *device_handle)
{
    if (device_handle == NULL || device_handle->device_params.serial_port < 0)
    {
        fs_log_output("[Trifecta] Warning: Invalid device handle or serial port!");
        return -1;
    }

    if (close(device_handle->device_params.serial_port) != 0)
    {
        fs_log_output("[Trifecta] Warning: Failed to close serial port (port: %d)! Error: %s", device_handle->device_params.serial_port, strerror(errno));
        device_handle->device_params.serial_port = -1;
        return -1;
    }
    device_handle->device_params.serial_port = -1;
    return 0;
}

/// @brief Attempts to reconnect the serial communication for the specified device.
/// @param device_handle Pointer to the device information structure.
/// @return 0 on success, or a negative error code on failure.
int fs_attempt_reconnect_serial(fs_device_info_t *device_handle)
{
    if (!device_handle)
    {
        return -1;
    }

    if (device_handle->device_params.serial_port > 2)
        close(device_handle->device_params.serial_port);

    device_handle->device_params.serial_port = open(device_handle->device_params.serial_path, O_RDWR | O_NOCTTY | O_NONBLOCK);

    if (device_handle->device_params.serial_port <= 2)
    {
        fs_log_critical("[Trifecta] Could not start serial driver: open() returned an invalid FD (%d)", device_handle->device_params.serial_port);
        device_handle->device_params.serial_port = -1;
        return -1;
    }

    // Configure the serial port
    if (configure_serial_port(device_handle->device_params.serial_port) != 0)
    {
        fs_log_critical("[Trifecta] Could not start serial driver: Failed to configure port: %d (Errno: %d)", device_handle->device_params.serial_port, errno);
        close(device_handle->device_params.serial_port);
        return -1;
    }
    return 0;
}
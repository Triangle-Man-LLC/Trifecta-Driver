/// Generic driver for the Trifecta series of IMU/AHRS/INS devices.
/// Copyright 2024 4rge.ai and/or Triangle Man LLC
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

int fs_logging_level = 1; // Logging level - 0 = OFF, 1 = ON

/// @brief Start the network TCP driver.
/// @param device_handle Pointer to the device information structure
/// @return 0 on success, -1 on failure
int fs_init_network_tcp_driver(fs_device_info_t *device_handle)
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
int fs_init_network_udp_driver(fs_device_info_t *device_handle)
{
    if (device_handle == NULL || device_handle->ip_addr[0] == '\0')
    {
        fs_log_output("[Trifecta] Error: Invalid device handle or IP address!\n");
        return -1;
    }

    // Close existing socket if it's already open
    if (device_handle->udp_sock >= 0)
    {
        close(device_handle->udp_sock);
        device_handle->udp_sock = -1;
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

    // Set SO_REUSEADDR to allow multiple sockets to bind to the same port
    int reuse = 1;
    if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0)
    {
        fs_log_output("[Trifecta] Error: setsockopt SO_REUSEADDR failed!\n");
        close(sockfd);
        return -1;
    }

    // Bind to a local address and port for receiving packets
    struct sockaddr_in local_addr;
    memset(&local_addr, 0, sizeof(local_addr));
    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = htonl(INADDR_ANY); // Listen on all interfaces
    local_addr.sin_port = htons(FS_TRIFECTA_PORT);

    if (bind(sockfd, (struct sockaddr *)&local_addr, sizeof(local_addr)) < 0)
    {
        fs_log_output("[Trifecta] Error: Could not bind UDP socket!\n");
        close(sockfd);
        return -1;
    }

    device_handle->udp_sock = sockfd;
    return 0;
}

/// @brief Configure serial port settings
/// @param fd The file descriptor for the serial port
/// @return 0 if successful, -1 if failed
static int configure_serial_port(int fd)
{
    struct termios options;
    if (fd <= 2)
    {
        // File descriptors 0, 1, 2 are typically STDIN, STDOUT, STDERR
        fs_log_output("[Trifecta] Error: Invalid file descriptor %d!", fd);
        return -1;
    }

    // Get the current options for the port
    if (tcgetattr(fd, &options) != 0)
    {
        fs_log_output("[Trifecta] Error: Failed to get serial port attributes: %s", strerror(errno));
        return -1;
    }

    // Set the baud rate to FS_TRIFECTA_SERIAL_BAUDRATE_POSIX (B921600)
    cfsetispeed(&options, FS_TRIFECTA_SERIAL_BAUDRATE_POSIX);
    cfsetospeed(&options, FS_TRIFECTA_SERIAL_BAUDRATE_POSIX);

    // Enable the receiver and set local mode
    options.c_cflag |= (CLOCAL | CREAD);

    // Set 8 data bits, no parity, 1 stop bit (8N1)
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;

    // No hardware flow control
    options.c_cflag &= ~CRTSCTS;

    // No software flow control
    options.c_iflag &= ~(IXON | IXOFF | IXANY);

    // Raw input mode (no processing)
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(INPCK | ISTRIP | ICRNL | IXON);
    options.c_oflag &= ~OPOST;

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
        fs_log_output("[Trifecta] Device handle is NULL!");
        return -1;
    }

    if (device_handle->serial_port == -1)
    {
        glob_t globbuf;
        int result;

        // Search for all files matching the pattern "/dev/tty*"
        result = glob("/dev/tty*", GLOB_NOSORT, NULL, &globbuf);

        if (result == 0)
        {
            for (size_t i = 0; i < globbuf.gl_pathc; i++)
            {
                char *port_name = globbuf.gl_pathv[i];

                // Open the serial port in non-blocking mode
                int fd = open(port_name, O_RDWR | O_NOCTTY | O_NONBLOCK);
                if (fd == -1)
                {
                    fs_log_output("[Trifecta] Failed to open serial port %s: %s\n", port_name, strerror(errno));
                    continue;
                }

                // Configure the serial port
                if (configure_serial_port(fd) != 0)
                {
                    close(fd);
                    continue;
                }

                // Write "I0;" to the serial port
                const char *init_cmd = "I0;";
                ssize_t written = write(fd, init_cmd, strnlen(init_cmd, 4));
                if (written == -1)
                {
                    fs_log_output("[Trifecta] Failed to write to serial port %s: %s\n", port_name, strerror(errno));
                    close(fd);
                    continue;
                }

                // Wait for 25 ms
                usleep(25000);

                // Read the response
                char response[20];
                ssize_t read_len = read(fd, response, sizeof(response) - 1);
                if (read_len > 0 && response[0] == 'I')
                {
                    response[read_len] = '\0'; // Null-terminate the response
                    char *end_char = strchr(response, ';');
                    if (end_char)
                    {
                        *end_char = '\0';
                        strncpy(device_handle->device_name, response + 1, sizeof(device_handle->device_name) - 1);
                        device_handle->device_name[sizeof(device_handle->device_name) - 1] = '\0';
                        device_handle->serial_port = fd;
                        globfree(&globbuf);
                        return 0;
                    }
                }

                // Close the serial port if the response is not valid
                close(fd);
            }

            fs_log_output("[Trifecta] No valid serial port found!\n");
        }
        else
        {
            fs_log_output("[Trifecta] glob() error: %d\n", result);
        }

        globfree(&globbuf);
        return -1;
    }

    else
    {
        // Open the specified serial port in non-blocking mode
        char port_name[20];
        snprintf(port_name, sizeof(port_name), "/dev/ttyS%d", device_handle->serial_port);
        int fd = open(port_name, O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd == -1)
        {
            fs_log_output("[Trifecta] Failed to open serial port %s: %s", port_name, strerror(errno));
            return -1;
        }

        // Configure the serial port
        if (configure_serial_port(fd) != 0)
        {
            close(fd);
            return -1;
        }

        device_handle->serial_port = fd;
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
int fs_thread_start(void(thread_func)(void *), void *params, fs_run_status_t *thread_running_flag, size_t stack_size, int priority, int core_affinity)
{
    if (thread_func == NULL || thread_running_flag == NULL)
    {
        fs_log_output("[Trifecta] Error: Invalid thread function or running flag!\n");
        return -1;
    }

    pthread_t thread;
    pthread_attr_t attr;
    pthread_attr_init(&attr);

    // Apply system defaults for parameters if their values are < 0
    if (stack_size <= 0)
    {
        stack_size = PTHREAD_STACK_MIN; // Default stack size, platform-defined minimum
    }
    if (priority < 0)
    {
        priority = sched_get_priority_min(SCHED_OTHER); // Default priority, lowest valid
    }
    if (core_affinity < 0)
    {
        core_affinity = -1; // Indifferent to core affinity
    }

    // Set stack size
    if (pthread_attr_setstacksize(&attr, stack_size) != 0)
    {
        fs_log_output("[Trifecta] Warning: Failed to set stack size!\n");
    }

    // Set thread priority if supported
    struct sched_param param;
    param.sched_priority = priority;
    if (pthread_attr_setschedparam(&attr, &param) != 0)
    {
        fs_log_output("[Trifecta] Warning: Failed to set thread priority!\n");
    }

    // Set CPU core affinity if specified and supported
    if (core_affinity >= 0)
    {
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(core_affinity, &cpuset);
        int affinity_result = pthread_attr_setaffinity_np(&attr, sizeof(cpu_set_t), &cpuset);
        if (affinity_result != 0)
        {
            fs_log_output("[Trifecta] Warning: Failed to set thread affinity!\n");
        }
    }

    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

    // Create the thread
    *thread_running_flag = FS_RUN_STATUS_RUNNING;
    int result = pthread_create(&thread, &attr, thread_func, params);
    pthread_attr_destroy(&attr);

    if (result != 0)
    {
        fs_log_output("[Trifecta] Error: Thread creation failed: errno %d!\n", errno);
        *thread_running_flag = FS_RUN_STATUS_ERROR;
        return -1;
    }

    fs_log_output("[Trifecta] Thread created successfully.\n");
    return 0;
}

/// @brief Some platforms (e.g. FreeRTOS) require thread exit to be properly handled.
/// This function should implement that behavior.
/// @param thread_handle On Linux systems, this has no impact.
/// @return Should always return 0...
int fs_thread_exit(void *thread_handle)
{
    pthread_exit(NULL);
    return 0;
}

/// @brief Transmit data over a networked TCP connection
/// @param device_handle Pointer to the device information structure
/// @param tx_buffer Pointer to the transmit data buffer
/// @param length_bytes The size of the tx_buffer
/// @param timeout_micros The max amount of time to wait (microseconds)
/// @return -1 if failed, else number of bytes written
ssize_t fs_transmit_networked_tcp(fs_device_info_t *device_handle, void *tx_buffer, size_t length_bytes, int timeout_micros)
{
    if (device_handle == NULL)
    {
        fs_log_output("[Trifecta] Error: Device handle is NULL!");
        return -1;
    }

    if (device_handle->communication_mode != FS_COMMUNICATION_MODE_TCP_UDP)
    {
        fs_log_output("[Trifecta] Error: Invalid communication mode! Expected FS_COMMUNICATION_MODE_TCP_UDP.");
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
ssize_t fs_transmit_networked_udp(fs_device_info_t *device_handle, void *tx_buffer, size_t length_bytes, int timeout_micros)
{
    if (device_handle == NULL)
    {
        fs_log_output("[Trifecta] Error: Device handle is NULL!");
        return -1;
    }

    if (device_handle->communication_mode != FS_COMMUNICATION_MODE_TCP_UDP)
    {
        fs_log_output("[Trifecta] Error: Invalid communication mode! Expected FS_COMMUNICATION_MODE_TCP_UDP.");
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
ssize_t fs_transmit_serial(fs_device_info_t *device_handle, void *tx_buffer, size_t length_bytes, int timeout_micros)
{
    if (device_handle == NULL)
    {
        fs_log_output("[Trifecta] Error: Device handle is NULL!");
        return -1;
    }

    if (device_handle->communication_mode != FS_COMMUNICATION_MODE_UART 
        && device_handle->communication_mode != FS_COMMUNICATION_MODE_USB_CDC)
    {
        fs_log_output("[Trifecta] Error: Invalid communication mode! Expected FS_COMMUNICATION_MODE_SERIAL.");
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

    // Set up the timeout using select
    struct timeval timeout;
    timeout.tv_sec = timeout_micros / 1000000;
    timeout.tv_usec = timeout_micros % 1000000;

    fd_set write_fds;
    FD_ZERO(&write_fds);
    FD_SET(device_handle->serial_port, &write_fds);

    int select_result = select(device_handle->serial_port + 1, NULL, &write_fds, NULL, &timeout);
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

    ssize_t actual_len = write(device_handle->serial_port, tx_buffer, length_bytes);
    if (actual_len == -1)
    {
        fs_log_output("[Trifecta] Error: Writing data over serial failed! Error: %s", strerror(errno));
        return -1;
    }

    fs_log_output("[Trifecta] Serial transmit to port %d - Length %ld", device_handle->serial_port, (long)actual_len);

    return actual_len;
}

/// @brief Receive data over a networked TCP connection
/// @param device_handle Pointer to the device information structure
/// @param rx_buffer Pointer to the receive data buffer
/// @param length_bytes The max size of the rx_buffer
/// @param timeout_micros The max amount of time to wait (microseconds)
/// @return -1 if failed, else number of bytes received
ssize_t fs_receive_networked_tcp(fs_device_info_t *device_handle, void *rx_buffer, size_t length_bytes, int timeout_micros)
{
    if (device_handle == NULL)
    {
        fs_log_output("[Trifecta] Error: Device handle is NULL!");
        return -1;
    }

    if (device_handle->communication_mode != FS_COMMUNICATION_MODE_TCP_UDP)
    {
        fs_log_output("[Trifecta] Error: Invalid communication mode! Expected FS_COMMUNICATION_MODE_TCP_UDP.");
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
        fs_log_output("[Trifecta] Error: Receiving data over TCP failed! Error: %s", strerror(errno));
    }

    return recv_len;
}

/// @brief Receive data over a networked UDP connection
/// @param device_handle Pointer to the device information structure
/// @param rx_buffer Pointer to the receive data buffer
/// @param length_bytes The max size of the rx_buffer
/// @param timeout_micros The max amount of time to wait (microseconds)
/// @return -1 if failed, else number of bytes received
ssize_t fs_receive_networked_udp(fs_device_info_t *device_handle, void *rx_buffer, size_t length_bytes, int timeout_micros)
{
    if (device_handle == NULL)
    {
        fs_log_output("[Trifecta] Error: Device handle is NULL!");
        return -1;
    }

    if (device_handle->communication_mode != FS_COMMUNICATION_MODE_TCP_UDP)
    {
        fs_log_output("[Trifecta] Error: Invalid communication mode! Expected FS_COMMUNICATION_MODE_TCP_UDP.");
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
        fs_log_output("[Trifecta] Error: Receiving data over UDP failed! Error: %s", strerror(errno));
    }

    return recv_len;
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

    if (device_handle->communication_mode != FS_COMMUNICATION_MODE_UART 
        && device_handle->communication_mode != FS_COMMUNICATION_MODE_USB_CDC)
    {
        fs_log_output("[Trifecta] Error: Invalid communication mode! Expected FS_COMMUNICATION_MODE_SERIAL.");
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

    // Set up the timeout using select
    struct timeval timeout;
    timeout.tv_sec = timeout_micros / 1000000;
    timeout.tv_usec = timeout_micros % 1000000;

    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(device_handle->serial_port, &read_fds);

    int select_result = select(device_handle->serial_port + 1, &read_fds, NULL, NULL, &timeout);
    if (select_result == -1)
    {
        fs_log_output("[Trifecta] Error: select() failed! Error: %s", strerror(errno));
        return -1;
    }
    else if (select_result == 0)
    {
        fs_log_output("[Trifecta] Error: Read timeout reached!");
        return -1;
    }

    // Check how much data is available
    int buffer_data_len = 0;
    if (ioctl(device_handle->serial_port, FIONREAD, &buffer_data_len) < 0)
    {
        fs_log_output("[Trifecta] Error: Could not get buffered data length! Error: %s", strerror(errno));
        return -1;
    }

    if (buffer_data_len > length_bytes)
    {
        fs_log_output("[Trifecta] Error: Data of length %d would overflow buffer size %ld!", buffer_data_len, length_bytes);
        tcflush(device_handle->serial_port, TCIFLUSH); // Flush the input buffer
        return -1;
    }

    ssize_t rx_len = read(device_handle->serial_port, rx_buffer, buffer_data_len);
    if (rx_len < 0)
    {
        fs_log_output("[Trifecta] Error: Reading data over serial failed! Error: %s", strerror(errno));
        tcflush(device_handle->serial_port, TCIFLUSH); // Flush the input buffer
        return -1;
    }
    else if (rx_len > 0)
    {
        fs_log_output("[Trifecta] Read data from port %d - length %zd!", device_handle->serial_port, rx_len);
        tcflush(device_handle->serial_port, TCIFLUSH); // Flush the input buffer
    }

    return rx_len;
}

/// @brief Shutdown the network TCP driver.
/// @param device_handle Pointer to the device information structure.
/// @return 0 if successful, -1 if failed.
int fs_shutdown_network_tcp_driver(fs_device_info_t *device_handle)
{
    if (device_handle == NULL || device_handle->tcp_sock < 0)
    {
        fs_log_output("[Trifecta] Warning: Invalid device handle or TCP socket!");
        return -1;
    }

    if (close(device_handle->tcp_sock) != 0)
    {
        fs_log_output("[Trifecta] Warning: Failed to close TCP socket (socket: %d)! Error: %s", device_handle->tcp_sock, strerror(errno));
        device_handle->tcp_sock = -1;
        return -1;
    }
    device_handle->tcp_sock = -1;
    return 0;
}

/// @brief Shutdown the network UDP driver.
/// @param device_handle Pointer to the device information structure.
/// @return 0 if successful, -1 if failed.
int fs_shutdown_network_udp_driver(fs_device_info_t *device_handle)
{
    if (device_handle == NULL || device_handle->udp_sock < 0)
    {
        fs_log_output("[Trifecta] Warning: Invalid device handle or UDP socket!");
        return -1;
    }

    if (close(device_handle->udp_sock) != 0)
    {
        fs_log_output("[Trifecta] Warning: Failed to close UDP socket (socket: %d)! Error: %s", device_handle->udp_sock, strerror(errno));
        device_handle->udp_sock = -1;
        return -1;
    }
    device_handle->udp_sock = -1;
    return 0;
}

/// @brief Shutdown the serial driver.
/// @param device_handle Pointer to the device information structure.
/// @return 0 if successful, -1 if failed.
int fs_shutdown_serial_driver(fs_device_info_t *device_handle)
{
    if (device_handle == NULL || device_handle->serial_port < 0)
    {
        fs_log_output("[Trifecta] Warning: Invalid device handle or serial port!");
        return -1;
    }

    if (close(device_handle->serial_port) != 0)
    {
        fs_log_output("[Trifecta] Warning: Failed to close serial port (port: %d)! Error: %s", device_handle->serial_port, strerror(errno));
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
    struct timespec req, rem;
    req.tv_sec = millis / 1000;
    req.tv_nsec = (millis % 1000) * 1000000;

    while (nanosleep(&req, &rem) == -1 && errno == EINTR)
    {
        req = rem;
    }

    return millis;
}

/// @brief Real-time delay
/// @param current_time Pointer to the current time
/// @param millis The exact amount of time to delay
/// @return The number of ticks the delay lasted
int fs_delay_for(uint32_t *current_time, int millis)
{
    if (current_time == NULL)
    {
        return -1;
    }

    struct timespec start, end;
    clock_gettime(CLOCK_MONOTONIC, &start);

    fs_delay(millis);

    clock_gettime(CLOCK_MONOTONIC, &end);

    uint32_t elapsed_ms = (end.tv_sec - start.tv_sec) * 1000 + (end.tv_nsec - start.tv_nsec) / 1000000;

    *current_time += elapsed_ms;
    return elapsed_ms;
}

/// @brief Get the current system time
/// @param current_time Pointer to the current time
/// @return 0 on success
int fs_get_current_time(uint32_t *current_time)
{
    struct timespec ts;
    if (clock_gettime(CLOCK_REALTIME, &ts) == -1)
    {
        return -1;
    }
    *current_time = ts.tv_sec * 1000 + ts.tv_nsec / 1000000; // Convert to milliseconds
    return 0;
}

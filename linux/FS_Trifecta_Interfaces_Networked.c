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

/// @brief Start the network TCP driver.
/// @param device_handle Pointer to the device information structure
/// @return 0 on success, -1 on failure
int fs_init_network_tcp_driver(fs_device_info_t *device_handle)
{
    if (device_handle == NULL || device_handle->device_params.ip_addr[0] == '\0')
    {
        fs_log_output("[Trifecta] Error: Invalid device handle or IP address!\n");
        return -1;
    }

    // Convert IP address string to binary form
    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(device_handle->device_params.tcp_port);

    if (inet_pton(AF_INET, device_handle->device_params.ip_addr, &server_addr.sin_addr) <= 0)
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

    device_handle->device_params.tcp_sock = sockfd;
    return 0;
}

/// @brief Start the network UDP driver.
/// @param device_handle Pointer to the device information structure
/// @return 0 on success, -1 on failure
int fs_init_network_udp_driver(fs_device_info_t *device_handle)
{
    if (device_handle == NULL || device_handle->device_params.ip_addr[0] == '\0')
    {
        fs_log_output("[Trifecta] Error: Invalid device handle or IP address!\n");
        return -1;
    }

    // Close existing socket if it's already open
    if (device_handle->device_params.udp_sock >= 0)
    {
        close(device_handle->device_params.udp_sock);
        device_handle->device_params.udp_sock = -1;
    }

    // Convert IP address string to binary form
    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(device_handle->device_params.udp_port);
    if (inet_pton(AF_INET, device_handle->device_params.ip_addr, &server_addr.sin_addr) <= 0)
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

    device_handle->device_params.udp_sock = sockfd;
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

    if (device_handle->device_params.communication_mode != FS_COMMUNICATION_MODE_TCP_UDP)
    {
        fs_log_output("[Trifecta] Error: Invalid communication mode! Expected FS_COMMUNICATION_MODE_TCP_UDP.");
        return -1;
    }

    if (device_handle->device_params.tcp_sock < 0)
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
    if (setsockopt(device_handle->device_params.tcp_sock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout)) < 0)
    {
        fs_log_output("[Trifecta] Error: Could not set send timeout!");
        return -1;
    }

    int written = send(device_handle->device_params.tcp_sock, tx_buffer, length_bytes, 0);

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

    if (device_handle->device_params.communication_mode != FS_COMMUNICATION_MODE_TCP_UDP)
    {
        fs_log_output("[Trifecta] Error: Invalid communication mode! Expected FS_COMMUNICATION_MODE_TCP_UDP.");
        return -1;
    }

    if (device_handle->device_params.udp_sock < 0)
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
    if (setsockopt(device_handle->device_params.udp_sock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout)) < 0)
    {
        fs_log_output("[Trifecta] Error: Could not set send timeout!");
        return -1;
    }

    int written = send(device_handle->device_params.udp_sock, tx_buffer, length_bytes, 0);

    if (written < 0)
    {
        fs_log_output("[Trifecta] Error: Sending data over UDP failed!");
    }

    return written;
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

    if (device_handle->device_params.communication_mode != FS_COMMUNICATION_MODE_TCP_UDP)
    {
        fs_log_output("[Trifecta] Error: Invalid communication mode! Expected FS_COMMUNICATION_MODE_TCP_UDP.");
        return -1;
    }

    if (device_handle->device_params.tcp_sock < 0)
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
    if (setsockopt(device_handle->device_params.tcp_sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0)
    {
        fs_log_output("[Trifecta] Error: Could not set receive timeout (TCP)!");
        return -1;
    }

    ssize_t recv_len = recv(device_handle->device_params.tcp_sock, rx_buffer, length_bytes, 0);

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

    if (device_handle->device_params.communication_mode != FS_COMMUNICATION_MODE_TCP_UDP)
    {
        fs_log_output("[Trifecta] Error: Invalid communication mode! Expected FS_COMMUNICATION_MODE_TCP_UDP.");
        return -1;
    }

    if (device_handle->device_params.udp_sock < 0)
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
    if (setsockopt(device_handle->device_params.udp_sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0)
    {
        fs_log_output("[Trifecta] Error: Could not set receive timeout (UDP)!");
        return -1;
    }

    ssize_t recv_len = recv(device_handle->device_params.udp_sock, rx_buffer, length_bytes, 0);

    if (recv_len < 0)
    {
        fs_log_output("[Trifecta] Error: Receiving data over UDP failed! Error: %s", strerror(errno));
    }

    return recv_len;
}

/// @brief Shutdown the network TCP driver.
/// @param device_handle Pointer to the device information structure.
/// @return 0 if successful, -1 if failed.
int fs_shutdown_network_tcp_driver(fs_device_info_t *device_handle)
{
    if (device_handle == NULL || device_handle->device_params.tcp_sock < 0)
    {
        fs_log_output("[Trifecta] Warning: Invalid device handle or TCP socket!");
        return -1;
    }

    if (close(device_handle->device_params.tcp_sock) != 0)
    {
        fs_log_output("[Trifecta] Warning: Failed to close TCP socket (socket: %d)! Error: %s", device_handle->device_params.tcp_sock, strerror(errno));
        device_handle->device_params.tcp_sock = -1;
        return -1;
    }
    device_handle->device_params.tcp_sock = -1;
    return 0;
}

/// @brief Shutdown the network UDP driver.
/// @param device_handle Pointer to the device information structure.
/// @return 0 if successful, -1 if failed.
int fs_shutdown_network_udp_driver(fs_device_info_t *device_handle)
{
    if (device_handle == NULL || device_handle->device_params.udp_sock < 0)
    {
        fs_log_output("[Trifecta] Warning: Invalid device handle or UDP socket!");
        return -1;
    }

    if (close(device_handle->device_params.udp_sock) != 0)
    {
        fs_log_output("[Trifecta] Warning: Failed to close UDP socket (socket: %d)! Error: %s", device_handle->device_params.udp_sock, strerror(errno));
        device_handle->device_params.udp_sock = -1;
        return -1;
    }
    device_handle->device_params.udp_sock = -1;
    return 0;
}

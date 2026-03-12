/// Generic driver for the Trifecta series of IMU/AHRS/INS devices.
/// Copyright 2025 4rge.ai and/or Triangle Man LLC
/// Usage and redistribution of this code is permitted
/// but this notice must be retained in all copies of the code.

/// THIS SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
/// INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE,
/// AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
/// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <tchar.h>
#include <time.h>
#include <string.h>

#define WIN32_LEAN_AND_MEAN
#define _WINSOCKAPI_ // Prevent inclusion of winsock.h by windows.h

#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <process.h>
#include <io.h>
#include <fcntl.h>
#include <tchar.h>
#include <conio.h>

#include "FS_Trifecta_Interfaces.h"

// Platform-specific: Functions for initializing communication drivers on target platform
#pragma comment(lib, "ws2_32.lib")

/// @brief Start the network TCP driver.
/// @param device_handle Pointer to the device information structure
/// @return 0 on success, -1 on failure
int fs_init_network_tcp_driver(fs_device_info_t *device_handle)
{
    if (device_handle == NULL || device_handle->device_params.ip_addr[0] == '\0')
    {
        fs_log_output("[Trifecta-Interface] Error: Invalid device handle or IP address!\n");
        return -1;
    }

    // Initialize Winsock
    WSADATA wsaData;
    int wsa_result = WSAStartup(MAKEWORD(2, 2), &wsaData);
    if (wsa_result != 0)
    {
        fs_log_output("[Trifecta-Interface] Error: WSAStartup failed with code %d\n", wsa_result);
        return -1;
    }

    // Convert IP address string to binary form
    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(device_handle->device_params.tcp_port);
    if (inet_pton(AF_INET, device_handle->device_params.ip_addr, &server_addr.sin_addr) <= 0)
    {
        fs_log_output("[Trifecta-Interface] Error: Invalid IP address format! IP address was: %s\n", device_handle->device_params.ip_addr);
        WSACleanup();
        return -1;
    }

    // Create TCP socket
    SOCKET sockfd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sockfd == INVALID_SOCKET)
    {
        fs_log_output("[Trifecta-Interface] Error: Could not create TCP socket! Code: %d\n", WSAGetLastError());
        WSACleanup();
        return -1;
    }

    // Connect to the device
    if (connect(sockfd, (struct sockaddr *)&server_addr, sizeof(server_addr)) == SOCKET_ERROR)
    {
        fs_log_output("[Trifecta-Interface] Error: Could not connect to device! Code: %d\n", WSAGetLastError());
        closesocket(sockfd);
        WSACleanup();
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
        fs_log_output("[Trifecta-Interface] Error: Invalid device handle or IP address!\n");
        return -1;
    }

    // Close existing socket if it's already open
    if (device_handle->device_params.udp_sock != INVALID_SOCKET)
    {
        closesocket(device_handle->device_params.udp_sock);
        device_handle->device_params.udp_sock = INVALID_SOCKET;
    }

    // Initialize Winsock
    WSADATA wsaData;
    int wsa_result = WSAStartup(MAKEWORD(2, 2), &wsaData);
    if (wsa_result != 0)
    {
        fs_log_output("[Trifecta-Interface] Error: WSAStartup failed with code %d\n", wsa_result);
        return -1;
    }

    // Convert IP address string to binary form
    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(device_handle->device_params.udp_port);
    if (inet_pton(AF_INET, device_handle->device_params.ip_addr, &server_addr.sin_addr) <= 0)
    {
        fs_log_output("[Trifecta-Interface] Error: Invalid IP address format!\n");
        WSACleanup();
        return -1;
    }

    // Create UDP socket
    SOCKET sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sockfd == INVALID_SOCKET)
    {
        fs_log_output("[Trifecta-Interface] Error: Could not create UDP socket! Code: %d\n", WSAGetLastError());
        WSACleanup();
        return -1;
    }

    // Set SO_REUSEADDR
    BOOL reuse = TRUE;
    if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, (const char *)&reuse, sizeof(reuse)) == SOCKET_ERROR)
    {
        fs_log_output("[Trifecta-Interface] Error: setsockopt SO_REUSEADDR failed! Code: %d\n", WSAGetLastError());
        closesocket(sockfd);
        WSACleanup();
        return -1;
    }

    // Bind to local address
    struct sockaddr_in local_addr;
    memset(&local_addr, 0, sizeof(local_addr));
    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    local_addr.sin_port = htons(device_handle->device_params.udp_port);

    if (bind(sockfd, (struct sockaddr *)&local_addr, sizeof(local_addr)) == SOCKET_ERROR)
    {
        fs_log_output("[Trifecta-Interface] Error: Could not bind UDP socket! Code: %d\n", WSAGetLastError());
        closesocket(sockfd);
        WSACleanup();
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
ssize_t fs_transmit_networked_tcp(fs_device_info_t *device_handle,
                                  void *tx_buffer,
                                  size_t length_bytes,
                                  int timeout_micros)
{
    if (!device_handle ||
        device_handle->device_params.communication_mode != FS_COMMUNICATION_MODE_TCP_UDP ||
        device_handle->device_params.tcp_sock == INVALID_SOCKET ||
        !tx_buffer)
        return -1;

    WSAPOLLFD pfd;
    pfd.fd = device_handle->device_params.tcp_sock;
    pfd.events = POLLOUT;
    pfd.revents = 0;

    int timeout_ms = timeout_micros / 1000;
    int r = WSAPoll(&pfd, 1, timeout_ms);

    if (r <= 0)
        return -1;  // timeout or error

    if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL))
        return -1;  // connection died

    int written = send(device_handle->device_params.tcp_sock,
                       (const char *)tx_buffer,
                       (int)length_bytes,
                       0);

    return (written == SOCKET_ERROR) ? -1 : written;
}

/// @brief Transmit data over a networked UDP connection
/// @param device_handle Pointer to the device information structure
/// @param tx_buffer Pointer to the transmit data buffer
/// @param length_bytes The size of the tx_buffer
/// @param timeout_micros The max amount of time to wait (microseconds)
/// @return -1 if failed, else number of bytes written
ssize_t fs_transmit_networked_udp(fs_device_info_t *device_handle,
                                  void *tx_buffer,
                                  size_t length_bytes,
                                  int timeout_micros)
{
    if (!device_handle ||
        device_handle->device_params.communication_mode != FS_COMMUNICATION_MODE_TCP_UDP ||
        device_handle->device_params.udp_sock == INVALID_SOCKET ||
        !tx_buffer)
        return -1;

    // Optional poll for symmetry (not required for UDP)
    WSAPOLLFD pfd;
    pfd.fd = device_handle->device_params.udp_sock;
    pfd.events = POLLOUT;
    pfd.revents = 0;

    int timeout_ms = timeout_micros / 1000;
    int r = WSAPoll(&pfd, 1, timeout_ms);

    if (r <= 0)
        return -1;

    if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL))
        return -1;

    int written = send(device_handle->device_params.udp_sock,
                       (const char *)tx_buffer,
                       (int)length_bytes,
                       0);

    return (written == SOCKET_ERROR) ? -1 : written;
}

/// @brief Receive data over a networked TCP connection
/// @param device_handle Pointer to the device information structure
/// @param rx_buffer Pointer to the receive data buffer
/// @param length_bytes The max size of the rx_buffer
/// @param timeout_micros The max amount of time to wait (microseconds)
/// @return -1 if failed, else number of bytes received
ssize_t fs_receive_networked_tcp(fs_device_info_t *device_handle,
                                 void *rx_buffer,
                                 size_t length_bytes,
                                 int timeout_micros)
{
    if (!device_handle ||
        device_handle->device_params.communication_mode != FS_COMMUNICATION_MODE_TCP_UDP ||
        device_handle->device_params.tcp_sock == INVALID_SOCKET ||
        !rx_buffer)
        return -1;

    WSAPOLLFD pfd;
    pfd.fd = device_handle->device_params.tcp_sock;
    pfd.events = POLLIN;
    pfd.revents = 0;

    int timeout_ms = timeout_micros / 1000;
    int r = WSAPoll(&pfd, 1, timeout_ms);

    if (r <= 0)
        return -1;  // timeout or error

    if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL))
        return -1;  // connection closed/reset

    int recv_len = recv(device_handle->device_params.tcp_sock,
                        (char *)rx_buffer,
                        (int)length_bytes,
                        0);

    return (recv_len == SOCKET_ERROR) ? -1 : recv_len;
}


/// @brief Receive data over a networked UDP connection
/// @param device_handle Pointer to the device information structure
/// @param rx_buffer Pointer to the receive data buffer
/// @param length_bytes The max size of the rx_buffer
/// @param timeout_micros The max amount of time to wait (microseconds)
/// @return -1 if failed, else number of bytes received
ssize_t fs_receive_networked_udp(fs_device_info_t *device_handle,
                                 void *rx_buffer,
                                 size_t length_bytes,
                                 int timeout_micros)
{
    if (!device_handle ||
        (device_handle->device_params.communication_mode != FS_COMMUNICATION_MODE_TCP_UDP &&
         device_handle->device_params.communication_mode != FS_COMMUNICATION_MODE_TCP_UDP_AP) ||
        device_handle->device_params.udp_sock == INVALID_SOCKET ||
        !rx_buffer)
        return -1;

    WSAPOLLFD pfd;
    pfd.fd = device_handle->device_params.udp_sock;
    pfd.events = POLLIN;
    pfd.revents = 0;

    int timeout_ms = timeout_micros / 1000;
    int r = WSAPoll(&pfd, 1, timeout_ms);

    if (r <= 0)
        return -1;

    if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL))
        return -1;

    int recv_len = recv(device_handle->device_params.udp_sock,
                        (char *)rx_buffer,
                        (int)length_bytes,
                        0);

    return (recv_len == SOCKET_ERROR) ? -1 : recv_len;
}


/// @brief Shutdown the network TCP driver.
/// @param device_handle Pointer to the device information structure.
/// @return 0 if successful, -1 if failed.
int fs_shutdown_network_tcp_driver(fs_device_info_t *device_handle)
{
    if (device_handle == NULL || device_handle->device_params.tcp_sock == INVALID_SOCKET)
    {
        fs_log_output("[Trifecta-Interface] Warning: Invalid device handle or TCP socket!");
        return -1;
    }

    if (closesocket(device_handle->device_params.tcp_sock) == SOCKET_ERROR)
    {
        fs_log_output("[Trifecta-Interface] Warning: Failed to close TCP socket (socket: %d)! Code: %d", (int)device_handle->device_params.tcp_sock, WSAGetLastError());
        device_handle->device_params.tcp_sock = INVALID_SOCKET;
        return -1;
    }

    device_handle->device_params.tcp_sock = INVALID_SOCKET;
    return 0;
}

/// @brief Shutdown the network UDP driver.
/// @param device_handle Pointer to the device information structure.
/// @return 0 if successful, -1 if failed.
int fs_shutdown_network_udp_driver(fs_device_info_t *device_handle)
{
    if (device_handle == NULL || device_handle->device_params.udp_sock == INVALID_SOCKET)
    {
        fs_log_output("[Trifecta-Interface] Warning: Invalid device handle or UDP socket!");
        return -1;
    }

    if (closesocket(device_handle->device_params.udp_sock) == SOCKET_ERROR)
    {
        fs_log_output("[Trifecta-Interface] Warning: Failed to close UDP socket (socket: %d)! Code: %d", (int)device_handle->device_params.udp_sock, WSAGetLastError());
        device_handle->device_params.udp_sock = INVALID_SOCKET;
        return -1;
    }

    device_handle->device_params.udp_sock = INVALID_SOCKET;
    return 0;
}

/// @brief Attempts to reconnect the TCP connection for the specified device.
/// @param device_handle Pointer to the device information structure.
/// @return 0 on success, or a negative error code on failure.
int fs_attempt_reconnect_network_tcp(fs_device_info_t *device_handle)
{
    if (!device_handle)
        return -1;

    // Close old socket if valid
    SOCKET old = device_handle->device_params.tcp_sock;
    if (old != INVALID_SOCKET)
        closesocket(old);

    // Create new socket
    SOCKET sockfd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sockfd == INVALID_SOCKET)
        return -2;

    // Build server address
    struct sockaddr_in server_addr = {0};
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(device_handle->device_params.tcp_port);

    if (inet_pton(AF_INET,
                  device_handle->device_params.ip_addr,
                  &server_addr.sin_addr) <= 0)
    {
        closesocket(sockfd);
        return -3;
    }

    // Reconnect
    if (connect(sockfd, (struct sockaddr *)&server_addr, sizeof(server_addr)) == SOCKET_ERROR)
    {
        closesocket(sockfd);
        return -4;
    }

    // Success: store new socket
    device_handle->device_params.tcp_sock = sockfd;
    return 0;
}

/// @brief Attempts to reconnect the UDP connection for the specified device.
/// @param device_handle Pointer to the device information structure.
/// @return 0 on success, or a negative error code on failure.
int fs_attempt_reconnect_network_udp(fs_device_info_t *device_handle)
{
    if (!device_handle)
        return -1;

    // Close old socket
    SOCKET old = device_handle->device_params.udp_sock;
    if (old != INVALID_SOCKET)
        closesocket(old);

    // Create new UDP socket
    SOCKET sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sockfd == INVALID_SOCKET)
        return -2;

    // Bind to same local port
    struct sockaddr_in local_addr = {0};
    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    local_addr.sin_port = htons(device_handle->device_params.udp_port);

    if (bind(sockfd, (struct sockaddr *)&local_addr, sizeof(local_addr)) == SOCKET_ERROR)
    {
        closesocket(sockfd);
        return -3;
    }

    device_handle->device_params.udp_sock = sockfd;
    return 0;
}

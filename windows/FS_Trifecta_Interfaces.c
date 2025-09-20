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
#define FS_TRIFECTA_SERIAL_BAUDRATE_WINDOWS 2000000

int fs_logging_level = 1; // Logging level - 0 = OFF, 1 = ON

/// @brief Helper to set device serial field from Windows-specific HANDLE
/// @param dev Device pointer
/// @param h HANDLE (Windows fd-like equivalent)
static inline void fs_set_serial_handle(fs_device_info_t *dev, HANDLE h)
{
    dev->serial_port = (int)(intptr_t)h;
}

/// @brief Helper to get Windows serial handle from device
/// @param dev Device pointer
/// @return HANDLE (Windows fd-like equivalent)
static inline HANDLE fs_get_serial_handle(const fs_device_info_t *dev)
{
    return (HANDLE)(intptr_t)(dev->serial_port);
}

/// @brief Whether interrupt-driven UART etc. is supported by the platform.
/// Many RTOSes support this, but Linux does not, etc.
/// @return OR FLAG of serial interfaces which support interrupts.
/// If the return is FS_COMMUNICATION_MODE_UNINITIALIZED, then no interrupt-driven serial is allowed.
int fs_platform_supported_serial_interrupts()
{
    return FS_COMMUNICATION_MODE_UNINITIALIZED;
}

/// @brief Start serial in interrupt mode on platforms that support it.
/// This enables more precise and low latency serial reads than polling.
/// @param device_handle
/// @param status_flag
/// @return 0 on success, -1 on fail (e.g. not supported on platform)
int fs_init_serial_interrupts(fs_device_info_t *device_handle, fs_run_status_t *status_flag)
{
    return -1;
}

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

    // Initialize Winsock
    WSADATA wsaData;
    int wsa_result = WSAStartup(MAKEWORD(2, 2), &wsaData);
    if (wsa_result != 0)
    {
        fs_log_output("[Trifecta] Error: WSAStartup failed with code %d\n", wsa_result);
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
        WSACleanup();
        return -1;
    }

    // Create TCP socket
    SOCKET sockfd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sockfd == INVALID_SOCKET)
    {
        fs_log_output("[Trifecta] Error: Could not create TCP socket! Code: %d\n", WSAGetLastError());
        WSACleanup();
        return -1;
    }

    // Connect to the device
    if (connect(sockfd, (struct sockaddr *)&server_addr, sizeof(server_addr)) == SOCKET_ERROR)
    {
        fs_log_output("[Trifecta] Error: Could not connect to device! Code: %d\n", WSAGetLastError());
        closesocket(sockfd);
        WSACleanup();
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
    if (device_handle->udp_sock != INVALID_SOCKET)
    {
        closesocket(device_handle->udp_sock);
        device_handle->udp_sock = INVALID_SOCKET;
    }

    // Initialize Winsock
    WSADATA wsaData;
    int wsa_result = WSAStartup(MAKEWORD(2, 2), &wsaData);
    if (wsa_result != 0)
    {
        fs_log_output("[Trifecta] Error: WSAStartup failed with code %d\n", wsa_result);
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
        WSACleanup();
        return -1;
    }

    // Create UDP socket
    SOCKET sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sockfd == INVALID_SOCKET)
    {
        fs_log_output("[Trifecta] Error: Could not create UDP socket! Code: %d\n", WSAGetLastError());
        WSACleanup();
        return -1;
    }

    // Set SO_REUSEADDR
    BOOL reuse = TRUE;
    if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, (const char *)&reuse, sizeof(reuse)) == SOCKET_ERROR)
    {
        fs_log_output("[Trifecta] Error: setsockopt SO_REUSEADDR failed! Code: %d\n", WSAGetLastError());
        closesocket(sockfd);
        WSACleanup();
        return -1;
    }

    // Bind to local address
    struct sockaddr_in local_addr;
    memset(&local_addr, 0, sizeof(local_addr));
    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    local_addr.sin_port = htons(FS_TRIFECTA_PORT);

    if (bind(sockfd, (struct sockaddr *)&local_addr, sizeof(local_addr)) == SOCKET_ERROR)
    {
        fs_log_output("[Trifecta] Error: Could not bind UDP socket! Code: %d\n", WSAGetLastError());
        closesocket(sockfd);
        WSACleanup();
        return -1;
    }

    device_handle->udp_sock = sockfd;
    return 0;
}

/// @brief Configure serial port settings (Windows version)
/// @param hSerial Windows HANDLE to the serial port (stored via fs_set_serial_handle)
/// @return 0 if successful, -1 if failed
static int configure_serial_port(HANDLE hSerial)
{
    if (hSerial == INVALID_HANDLE_VALUE)
    {
        fs_log_output("[Trifecta] Error: Invalid serial handle!");
        return -1;
    }

    DCB dcbSerialParams = {0};
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);

    if (!GetCommState(hSerial, &dcbSerialParams))
    {
        fs_log_output("[Trifecta] Error: Failed to get serial port state: %lu", GetLastError());
        return -1;
    }

    dcbSerialParams.BaudRate = FS_TRIFECTA_SERIAL_BAUDRATE_WINDOWS;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;

    // Disable hardware flow control
    dcbSerialParams.fOutxCtsFlow = FALSE;
    dcbSerialParams.fOutxDsrFlow = FALSE;
    dcbSerialParams.fDtrControl = DTR_CONTROL_DISABLE;
    dcbSerialParams.fRtsControl = RTS_CONTROL_DISABLE;

    // Disable software flow control
    dcbSerialParams.fInX = FALSE;
    dcbSerialParams.fOutX = FALSE;

    // Raw mode: no processing
    dcbSerialParams.fBinary = TRUE;
    dcbSerialParams.fAbortOnError = FALSE;

    if (!SetCommState(hSerial, &dcbSerialParams))
    {
        fs_log_output("[Trifecta] Error: Failed to set serial port state: %lu", GetLastError());
        return -1;
    }

    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadIntervalTimeout = MAXDWORD;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.ReadTotalTimeoutConstant = 0;
    timeouts.WriteTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant = 0;

    if (!SetCommTimeouts(hSerial, &timeouts))
    {
        fs_log_output("[Trifecta] Error: Failed to set serial timeouts: %lu", GetLastError());
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
        // Scan COM1 to COM32
        for (int i = 1; i <= 32; ++i)
        {
            TCHAR port_name[16];
            snprintf(port_name, sizeof(port_name), "\\\\.\\COM%d", i);

            HANDLE hSerial = CreateFileA(
                port_name,
                GENERIC_READ | GENERIC_WRITE,
                0,
                NULL,
                OPEN_EXISTING,
                0,
                NULL);

            if (hSerial == INVALID_HANDLE_VALUE)
            {
                continue;
            }

            if (configure_serial_port(hSerial) != 0)
            {
                CloseHandle(hSerial);
                continue;
            }

            const char *init_cmd = "I0;";
            DWORD bytes_written;
            if (!WriteFile(hSerial, init_cmd, (DWORD)strlen(init_cmd), &bytes_written, NULL))
            {
                fs_log_output("[Trifecta] Failed to write to %s", port_name);
                CloseHandle(hSerial);
                continue;
            }

            Sleep(25); // 25 ms delay

            char response[20] = {0};
            DWORD bytes_read;
            if (ReadFile(hSerial, response, sizeof(response) - 1, &bytes_read, NULL) && bytes_read > 0 && response[0] == 'I')
            {
                response[bytes_read] = '\0';
                char *end_char = strchr(response, ';');
                if (end_char)
                {
                    *end_char = '\0';
                    strncpy(device_handle->device_name, response + 1, sizeof(device_handle->device_name) - 1);
                    device_handle->device_name[sizeof(device_handle->device_name) - 1] = '\0';
                    fs_set_serial_handle(device_handle, hSerial);
                    return 0;
                }
            }

            CloseHandle(hSerial);
        }

        fs_log_output("[Trifecta] No valid COM port found!");
        return -1;
    }
    else
    {
        // Use specified COM port
        TCHAR port_name[16];
        snprintf(port_name, sizeof(port_name), "\\\\.\\COM%d", (int)device_handle->serial_port);

        HANDLE hSerial = CreateFileA(
            port_name,
            GENERIC_READ | GENERIC_WRITE,
            0,
            NULL,
            OPEN_EXISTING,
            0,
            NULL);

        if (hSerial == INVALID_HANDLE_VALUE)
        {
            fs_log_output("[Trifecta] Failed to open serial port %s", port_name);
            return -1;
        }

        if (configure_serial_port(hSerial) != 0)
        {
            CloseHandle(hSerial);
            return -1;
        }

        fs_set_serial_handle(device_handle, hSerial);
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
int fs_thread_start(void (*thread_func)(void *), void *params, fs_run_status_t *thread_running_flag, size_t stack_size, int priority, int core_affinity)
{
    if (thread_func == NULL || thread_running_flag == NULL)
    {
        fs_log_output("[Trifecta] Error: Invalid thread function or running flag!\n");
        return -1;
    }

    *thread_running_flag = FS_RUN_STATUS_RUNNING;

    HANDLE thread_handle = (HANDLE)_beginthreadex(
        NULL, // Security attributes
        (unsigned)stack_size,
        (unsigned(__stdcall *)(void *))thread_func,
        params,
        0,   // Run immediately
        NULL // Optionally capture thread ID
    );

    if (thread_handle == 0)
    {
        fs_log_output("[Trifecta] Error: Thread creation failed: errno %d!\n", errno);
        *thread_running_flag = FS_RUN_STATUS_ERROR;
        return -1;
    }

    // Set thread priority
    if (priority >= 0)
    {
        if (!SetThreadPriority(thread_handle, priority))
        {
            fs_log_output("[Trifecta] Warning: Failed to set thread priority: %lu\n", GetLastError());
        }
    }

    // Set core affinity
    if (core_affinity >= 0)
    {
        DWORD_PTR affinity_mask = 1ULL << core_affinity;
        if (SetThreadAffinityMask(thread_handle, affinity_mask) == 0)
        {
            fs_log_output("[Trifecta] Warning: Failed to set thread affinity: %lu\n", GetLastError());
        }
    }

    CloseHandle(thread_handle); // Detach thread

    fs_log_output("[Trifecta] Thread created successfully.\n");
    return 0;
}

/// @brief Some platforms (e.g. FreeRTOS) require thread exit to be properly handled.
/// This function should implement that behavior.
/// @param thread_handle On Linux systems, this has no impact.
/// @return Should always return 0...
int fs_thread_exit(void *thread_handle)
{
    ExitThread(0);
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

    if (device_handle->tcp_sock == INVALID_SOCKET)
    {
        fs_log_output("[Trifecta] Error: Invalid TCP socket!");
        return -1;
    }

    if (tx_buffer == NULL)
    {
        fs_log_output("[Trifecta] Error: Transmit buffer is NULL!");
        return -1;
    }

    // Set the send timeout (in milliseconds for Windows)
    DWORD timeout_ms = (DWORD)(timeout_micros / 1000);
    if (setsockopt(device_handle->tcp_sock, SOL_SOCKET, SO_SNDTIMEO, (const char *)&timeout_ms, sizeof(timeout_ms)) == SOCKET_ERROR)
    {
        fs_log_output("[Trifecta] Error: Could not set send timeout! Code: %d\n", WSAGetLastError());
        return -1;
    }

    int written = send(device_handle->tcp_sock, (const char *)tx_buffer, (int)length_bytes, 0);

    if (written == SOCKET_ERROR)
    {
        fs_log_output("[Trifecta] Error: Sending data over TCP failed! Code: %d\n", WSAGetLastError());
        return -1;
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

    if (device_handle->udp_sock == INVALID_SOCKET)
    {
        fs_log_output("[Trifecta] Error: Invalid UDP socket!");
        return -1;
    }

    if (tx_buffer == NULL)
    {
        fs_log_output("[Trifecta] Error: Transmit buffer is NULL!");
        return -1;
    }

    // Set the send timeout (in milliseconds for Windows)
    DWORD timeout_ms = (DWORD)(timeout_micros / 1000);
    if (setsockopt(device_handle->udp_sock, SOL_SOCKET, SO_SNDTIMEO, (const char *)&timeout_ms, sizeof(timeout_ms)) == SOCKET_ERROR)
    {
        fs_log_output("[Trifecta] Error: Could not set send timeout! Code: %d\n", WSAGetLastError());
        return -1;
    }

    int written = send(device_handle->udp_sock, (const char *)tx_buffer, (int)length_bytes, 0);

    if (written == SOCKET_ERROR)
    {
        fs_log_output("[Trifecta] Error: Sending data over UDP failed! Code: %d\n", WSAGetLastError());
        return -1;
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

    if (device_handle->communication_mode != FS_COMMUNICATION_MODE_UART &&
        device_handle->communication_mode != FS_COMMUNICATION_MODE_USB_CDC)
    {
        fs_log_output("[Trifecta] Error: Invalid communication mode! Expected FS_COMMUNICATION_MODE_SERIAL.");
        return -1;
    }

    HANDLE hSerial = fs_get_serial_handle(device_handle);
    if (hSerial == INVALID_HANDLE_VALUE || hSerial == NULL)
    {
        fs_log_output("[Trifecta] Error: Invalid serial handle!");
        return -1;
    }

    if (tx_buffer == NULL)
    {
        fs_log_output("[Trifecta] Error: Transmit buffer is NULL!");
        return -1;
    }

    // Configure write timeout
    COMMTIMEOUTS timeouts;
    if (!GetCommTimeouts(hSerial, &timeouts))
    {
        fs_log_output("[Trifecta] Error: Failed to get serial timeouts: %lu", GetLastError());
        return -1;
    }

    timeouts.WriteTotalTimeoutConstant = timeout_micros / 1000;
    timeouts.WriteTotalTimeoutMultiplier = 0;

    if (!SetCommTimeouts(hSerial, &timeouts))
    {
        fs_log_output("[Trifecta] Error: Failed to set serial timeouts: %lu", GetLastError());
        return -1;
    }

    DWORD bytes_written = 0;
    BOOL result = WriteFile(hSerial, tx_buffer, (DWORD)length_bytes, &bytes_written, NULL);
    if (!result)
    {
        fs_log_output("[Trifecta] Error: Writing data over serial failed: %lu", GetLastError());
        return -1;
    }

    fs_log_output("[Trifecta] Serial transmit to HANDLE %p - Length %lu", hSerial, bytes_written);
    return (ssize_t)bytes_written;
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

    if (device_handle->tcp_sock == INVALID_SOCKET)
    {
        fs_log_output("[Trifecta] Error: Invalid TCP socket!");
        return -1;
    }

    if (rx_buffer == NULL)
    {
        fs_log_output("[Trifecta] Error: Receive buffer is NULL!");
        return -1;
    }

    // Set the receive timeout (in milliseconds for Windows)
    DWORD timeout_ms = (DWORD)(timeout_micros / 1000);
    if (setsockopt(device_handle->tcp_sock, SOL_SOCKET, SO_RCVTIMEO, (const char *)&timeout_ms, sizeof(timeout_ms)) == SOCKET_ERROR)
    {
        fs_log_output("[Trifecta] Error: Could not set receive timeout (TCP)! Code: %d\n", WSAGetLastError());
        return -1;
    }

    int recv_len = recv(device_handle->tcp_sock, (char *)rx_buffer, (int)length_bytes, 0);

    if (recv_len == SOCKET_ERROR)
    {
        fs_log_output("[Trifecta] Error: Receiving data over TCP failed! Code: %d\n", WSAGetLastError());
        return -1;
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

    if (device_handle->udp_sock == INVALID_SOCKET)
    {
        fs_log_output("[Trifecta] Error: Invalid UDP socket!");
        return -1;
    }

    if (rx_buffer == NULL)
    {
        fs_log_output("[Trifecta] Error: Receive buffer is NULL!");
        return -1;
    }

    // Set the receive timeout (in milliseconds for Windows)
    DWORD timeout_ms = (DWORD)(timeout_micros / 1000);
    if (setsockopt(device_handle->udp_sock, SOL_SOCKET, SO_RCVTIMEO, (const char *)&timeout_ms, sizeof(timeout_ms)) == SOCKET_ERROR)
    {
        fs_log_output("[Trifecta] Error: Could not set receive timeout (UDP)! Code: %d\n", WSAGetLastError());
        return -1;
    }

    int recv_len = recv(device_handle->udp_sock, (char *)rx_buffer, (int)length_bytes, 0);

    if (recv_len == SOCKET_ERROR)
    {
        fs_log_output("[Trifecta] Error: Receiving data over UDP failed! Code: %d\n", WSAGetLastError());
        return -1;
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

    if (device_handle->communication_mode != FS_COMMUNICATION_MODE_UART &&
        device_handle->communication_mode != FS_COMMUNICATION_MODE_USB_CDC)
    {
        fs_log_output("[Trifecta] Error: Invalid communication mode! Expected FS_COMMUNICATION_MODE_SERIAL.");
        return -1;
    }

    HANDLE hSerial = fs_get_serial_handle(device_handle);
    if (hSerial == INVALID_HANDLE_VALUE || hSerial == NULL)
    {
        fs_log_output("[Trifecta] Error: Invalid serial handle!");
        return -1;
    }

    if (rx_buffer == NULL)
    {
        fs_log_output("[Trifecta] Error: Receive buffer is NULL!");
        return -1;
    }

    // Configure read timeout
    COMMTIMEOUTS timeouts;
    if (!GetCommTimeouts(hSerial, &timeouts))
    {
        fs_log_output("[Trifecta] Error: Failed to get serial timeouts: %lu", GetLastError());
        return -1;
    }

    timeouts.ReadIntervalTimeout = MAXDWORD;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.ReadTotalTimeoutConstant = timeout_micros / 1000;

    if (!SetCommTimeouts(hSerial, &timeouts))
    {
        fs_log_output("[Trifecta] Error: Failed to set serial timeouts: %lu", GetLastError());
        return -1;
    }

    DWORD bytes_read = 0;
    BOOL result = ReadFile(hSerial, rx_buffer, (DWORD)length_bytes, &bytes_read, NULL);
    if (!result)
    {
        fs_log_output("[Trifecta] Error: Reading data over serial failed: %lu", GetLastError());
        return -1;
    }

    if (bytes_read > 0)
    {
        fs_log_output("[Trifecta] Read data from HANDLE %p - length %lu", hSerial, bytes_read);
    }

    return (ssize_t)bytes_read;
}

/// @brief Shutdown the network TCP driver.
/// @param device_handle Pointer to the device information structure.
/// @return 0 if successful, -1 if failed.
int fs_shutdown_network_tcp_driver(fs_device_info_t *device_handle)
{
    if (device_handle == NULL || device_handle->tcp_sock == INVALID_SOCKET)
    {
        fs_log_output("[Trifecta] Warning: Invalid device handle or TCP socket!");
        return -1;
    }

    if (closesocket(device_handle->tcp_sock) == SOCKET_ERROR)
    {
        fs_log_output("[Trifecta] Warning: Failed to close TCP socket (socket: %d)! Code: %d", (int)device_handle->tcp_sock, WSAGetLastError());
        device_handle->tcp_sock = INVALID_SOCKET;
        return -1;
    }

    device_handle->tcp_sock = INVALID_SOCKET;
    return 0;
}

/// @brief Shutdown the network UDP driver.
/// @param device_handle Pointer to the device information structure.
/// @return 0 if successful, -1 if failed.
int fs_shutdown_network_udp_driver(fs_device_info_t *device_handle)
{
    if (device_handle == NULL || device_handle->udp_sock == INVALID_SOCKET)
    {
        fs_log_output("[Trifecta] Warning: Invalid device handle or UDP socket!");
        return -1;
    }

    if (closesocket(device_handle->udp_sock) == SOCKET_ERROR)
    {
        fs_log_output("[Trifecta] Warning: Failed to close UDP socket (socket: %d)! Code: %d", (int)device_handle->udp_sock, WSAGetLastError());
        device_handle->udp_sock = INVALID_SOCKET;
        return -1;
    }

    device_handle->udp_sock = INVALID_SOCKET;
    return 0;
}

/// @brief Shutdown the serial driver.
/// @param device_handle Pointer to the device information structure.
/// @return 0 if successful, -1 if failed.
int fs_shutdown_serial_driver(fs_device_info_t *device_handle)
{
    if (device_handle == NULL)
    {
        fs_log_output("[Trifecta] Warning: Device handle is NULL!");
        return -1;
    }

    HANDLE hSerial = fs_get_serial_handle(device_handle);
    if (hSerial == NULL || hSerial == INVALID_HANDLE_VALUE)
    {
        fs_log_output("[Trifecta] Warning: Invalid serial handle!");
        device_handle->serial_port = -1;
        return -1;
    }

    if (!CloseHandle(hSerial))
    {
        fs_log_output("[Trifecta] Warning: Failed to close serial handle %p! Error: %lu", hSerial, GetLastError());
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
    Sleep(millis);
    return millis;
}

/// @brief Real-time delay
/// @param current_time Pointer to the current time
/// @param millis The exact amount of time to delay
/// @return The number of ticks the delay lasted
/// @brief Real-time delay (Windows version)
/// @param current_time Pointer to the current time
/// @param millis The exact amount of time to delay
/// @return The number of ticks the delay lasted
int fs_delay_for(uint32_t *current_time, int millis)
{
    if (current_time == NULL)
    {
        return -1;
    }

    LARGE_INTEGER freq, start, end;
    QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&start);

    fs_delay(millis);

    QueryPerformanceCounter(&end);

    uint32_t elapsed_ms = (uint32_t)((end.QuadPart - start.QuadPart) * 1000 / freq.QuadPart);
    *current_time += elapsed_ms;
    return elapsed_ms;
}

/// @brief Get the current system time
/// @param current_time Pointer to the current time
/// @return 0 on success
int fs_get_current_time(uint32_t *current_time)
{
    if (current_time == NULL)
    {
        return -1;
    }

    FILETIME ft;
    GetSystemTimeAsFileTime(&ft);

    // Convert FILETIME (100-ns intervals since Jan 1, 1601) to milliseconds since Unix epoch
    ULARGE_INTEGER time;
    time.LowPart = ft.dwLowDateTime;
    time.HighPart = ft.dwHighDateTime;

    uint64_t ms_since_1601 = time.QuadPart / 10000;
    uint64_t ms_since_1970 = ms_since_1601 - 11644473600000ULL;

    *current_time = (uint32_t)(ms_since_1970 & 0xFFFFFFFF);
    return 0;
}
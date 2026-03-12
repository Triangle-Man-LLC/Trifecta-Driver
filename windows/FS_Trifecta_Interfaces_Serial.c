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

#include <windows.h>
#include <process.h>
#include <io.h>
#include <fcntl.h>
#include <tchar.h>
#include <conio.h>

#include "FS_Trifecta_Interfaces.h"

#define FS_TRIFECTA_SERIAL_BAUDRATE_WINDOWS 2000000

/// @brief Helper to set device serial field from Windows-specific HANDLE
/// @param dev Device pointer
/// @param h HANDLE (Windows fd-like equivalent)
static inline void fs_set_serial_handle(fs_device_info_t *dev, HANDLE h)
{
    dev->device_params.serial_port = (fs_serial_handle_t)h;
}

/// @brief Helper to get Windows serial handle from device
/// @param dev Device pointer
/// @return HANDLE (Windows fd-like equivalent)
static inline HANDLE fs_get_serial_handle(const fs_device_info_t *dev)
{
    return (HANDLE)(dev->device_params.serial_port);
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

int fs_wait_until_next_serial_interrupt(fs_device_info_t *device_handle)
{
    return -1; // Not supported on Windows
}

/// @brief Configure serial port settings (Windows version)
/// @param hSerial Windows HANDLE to the serial port (stored via fs_set_serial_handle)
/// @return 0 if successful, -1 if failed
static int configure_serial_port(HANDLE hSerial)
{
    if (hSerial == INVALID_HANDLE_VALUE)
    {
        fs_log_output("[Trifecta-Interface] Error: Invalid serial handle!");
        return -1;
    }

    // Clear any stale data from previous sessions or reconnects
    PurgeComm(hSerial, PURGE_RXCLEAR | PURGE_TXCLEAR);
    ClearCommError(hSerial, NULL, NULL);

    // Optional: increase driver-level buffers (Windows defaults are small)
    SetupComm(hSerial, 4096, 4096);

    DCB dcb = {0};
    dcb.DCBlength = sizeof(DCB);

    if (!GetCommState(hSerial, &dcb))
    {
        fs_log_output("[Trifecta-Interface] Error: Failed to get serial port state: %lu", GetLastError());
        return -1;
    }

    // Core serial settings
    dcb.BaudRate = FS_TRIFECTA_SERIAL_BAUDRATE_WINDOWS;
    dcb.ByteSize = 8;
    dcb.StopBits = ONESTOPBIT;
    dcb.Parity = NOPARITY;

    // Disable hardware flow control fully
    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fDtrControl = DTR_CONTROL_DISABLE;
    dcb.fRtsControl = RTS_CONTROL_DISABLE;
    dcb.fDsrSensitivity = FALSE;

    // Disable software flow control
    dcb.fInX = FALSE;
    dcb.fOutX = FALSE;

    // Raw binary mode (no character translation)
    dcb.fBinary = TRUE;
    dcb.fAbortOnError = FALSE;
    dcb.fErrorChar = FALSE;
    dcb.fNull = FALSE;
    dcb.EofChar = 0;
    dcb.EvtChar = 0;

    if (!SetCommState(hSerial, &dcb))
    {
        fs_log_output("[Trifecta-Interface] Error: Failed to set serial port state: %lu", GetLastError());
        return -1;
    }

    // Non-blocking, Linux-like timeout behavior
    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadIntervalTimeout = MAXDWORD;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.ReadTotalTimeoutConstant = 0;
    timeouts.WriteTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant = 0;

    if (!SetCommTimeouts(hSerial, &timeouts))
    {
        fs_log_output("[Trifecta-Interface] Error: Failed to set serial timeouts: %lu", GetLastError());
        return -1;
    }

    return 0;
}

/// @brief Start the network serial driver.
/// @param device_handle Pointer to the device information structure
/// @return 0 if successful, -1 if failed
int fs_init_serial_driver(fs_device_info_t *device_handle)
{
    if (!device_handle)
    {
        fs_log_output("[Trifecta-Interface] Device handle is NULL!");
        return -1;
    }

    // serial_port now holds a char* like "COM3"
    const char *com_string = (const char *)device_handle->device_params.serial_port;
    if (!com_string || com_string[0] == '\0')
    {
        fs_log_output("[Trifecta-Interface] Invalid COM port string!");
        return -1;
    }

    // Build and store the Windows device path
    snprintf(device_handle->device_params.serial_path,
             sizeof(device_handle->device_params.serial_path),
             "\\\\.\\%s",
             com_string);

    const char *port_name = device_handle->device_params.serial_path;

    HANDLE hSerial = CreateFileA(
        port_name,
        GENERIC_READ | GENERIC_WRITE,
        0,
        NULL,
        OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL,
        NULL);

    if (hSerial == INVALID_HANDLE_VALUE)
    {
        fs_log_output("[Trifecta-Interface] Failed to open serial port %s (Err=%lu)",
                      port_name, GetLastError());
        return -3;
    }

    if (configure_serial_port(hSerial) != 0)
    {
        CloseHandle(hSerial);
        return -4;
    }

    // After init, serial_port now holds the HANDLE (just like Linux stores the FD)
    fs_set_serial_handle(device_handle, hSerial);

    return 0;
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
        fs_log_output("[Trifecta-Interface] Error: Device handle is NULL!");
        return -1;
    }

    if (device_handle->device_params.communication_mode != FS_COMMUNICATION_MODE_UART &&
        device_handle->device_params.communication_mode != FS_COMMUNICATION_MODE_USB_CDC)
    {
        fs_log_output("[Trifecta-Interface] Error: Invalid communication mode! Expected FS_COMMUNICATION_MODE_SERIAL.");
        return -1;
    }

    HANDLE hSerial = fs_get_serial_handle(device_handle);
    if (hSerial == INVALID_HANDLE_VALUE || hSerial == NULL)
    {
        fs_log_output("[Trifecta-Interface] Error: Invalid serial handle!");
        return -1;
    }

    if (tx_buffer == NULL)
    {
        fs_log_output("[Trifecta-Interface] Error: Transmit buffer is NULL!");
        return -1;
    }

    // Configure write timeout
    COMMTIMEOUTS timeouts;
    if (!GetCommTimeouts(hSerial, &timeouts))
    {
        fs_log_output("[Trifecta-Interface] Error: Failed to get serial timeouts: %lu", GetLastError());
        return -1;
    }

    timeouts.WriteTotalTimeoutConstant = timeout_micros / 1000;
    timeouts.WriteTotalTimeoutMultiplier = 0;

    if (!SetCommTimeouts(hSerial, &timeouts))
    {
        fs_log_output("[Trifecta-Interface] Error: Failed to set serial timeouts: %lu", GetLastError());
        return -1;
    }

    DWORD bytes_written = 0;
    BOOL result = WriteFile(hSerial, tx_buffer, (DWORD)length_bytes, &bytes_written, NULL);
    if (!result)
    {
        fs_log_output("[Trifecta-Interface] Error: Writing data over serial failed: %lu", GetLastError());
        return -1;
    }

    fs_log_output("[Trifecta-Interface] Serial transmit to HANDLE %p - Length %lu, Data: %s", hSerial, bytes_written, tx_buffer);
    return (ssize_t)bytes_written;
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
        fs_log_output("[Trifecta-Interface] Error: Device handle is NULL!");
        return -1;
    }

    if (device_handle->device_params.communication_mode != FS_COMMUNICATION_MODE_UART &&
        device_handle->device_params.communication_mode != FS_COMMUNICATION_MODE_USB_CDC)
    {
        fs_log_output("[Trifecta-Interface] Error: Invalid communication mode! Expected FS_COMMUNICATION_MODE_SERIAL.");
        return -1;
    }

    HANDLE hSerial = fs_get_serial_handle(device_handle);
    if (hSerial == INVALID_HANDLE_VALUE || hSerial == NULL)
    {
        fs_log_output("[Trifecta-Interface] Error: Invalid serial handle!");
        return -1;
    }

    if (rx_buffer == NULL)
    {
        fs_log_output("[Trifecta-Interface] Error: Receive buffer is NULL!");
        return -1;
    }

    // Configure read timeout
    COMMTIMEOUTS timeouts;
    if (!GetCommTimeouts(hSerial, &timeouts))
    {
        fs_log_output("[Trifecta-Interface] Error: Failed to get serial timeouts: %lu", GetLastError());
        return -1;
    }

    timeouts.ReadIntervalTimeout = MAXDWORD;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.ReadTotalTimeoutConstant = timeout_micros / 1000;

    if (!SetCommTimeouts(hSerial, &timeouts))
    {
        fs_log_output("[Trifecta-Interface] Error: Failed to set serial timeouts: %lu", GetLastError());
        return -1;
    }

    DWORD bytes_read = 0;
    BOOL result = ReadFile(hSerial, rx_buffer, (DWORD)length_bytes, &bytes_read, NULL);
    if (!result)
    {
        fs_log_output("[Trifecta-Interface] Error: Reading data over serial failed: %lu", GetLastError());
        return -1;
    }

    if (bytes_read > 0)
    {
        fs_log_output("[Trifecta-Interface] Read data from HANDLE %p - length %lu", hSerial, bytes_read);
    }

    return (ssize_t)bytes_read;
}

/// @brief Shutdown the serial driver.
/// @param device_handle Pointer to the device information structure.
/// @return 0 if successful, -1 if failed.
int fs_shutdown_serial_driver(fs_device_info_t *device_handle)
{
    if (!device_handle)
        return -1;

    HANDLE hSerial = fs_get_serial_handle(device_handle);
    if (!hSerial || hSerial == INVALID_HANDLE_VALUE)
    {
        device_handle->device_params.serial_port = (fs_serial_handle_t)INVALID_HANDLE_VALUE;
        return -1;
    }

    // Stop background thread first
    device_handle->device_params.status = FS_RUN_STATUS_IDLE;

    // Flush buffers
    PurgeComm(hSerial, PURGE_RXCLEAR | PURGE_TXCLEAR);
    FlushFileBuffers(hSerial);

    // Close handle
    if (!CloseHandle(hSerial))
    {
        fs_log_output("[Trifecta-Interface] Warning: Failed to close serial handle %p! Error: %lu",
                      hSerial, GetLastError());
    }

    // Clear stored handle
    device_handle->device_params.serial_port = (fs_serial_handle_t)INVALID_HANDLE_VALUE;
    device_handle->device_params.status = FS_RUN_STATUS_IDLE;
    return 0;
}


/// @brief Attempts to reconnect the serial communication for the specified device.
/// @param device_handle Pointer to the device information structure.
/// @return 0 on success, or a negative error code on failure.
int fs_attempt_reconnect_serial(fs_device_info_t *device_handle)
{

    const char *port_name = device_handle->device_params.serial_path;

    HANDLE old = fs_get_serial_handle(device_handle);
    if (old && old != INVALID_HANDLE_VALUE)
        CloseHandle(old);

    HANDLE hSerial = CreateFileA(
        port_name,
        GENERIC_READ | GENERIC_WRITE,
        0,
        NULL,
        OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL,
        NULL);

    if (hSerial == INVALID_HANDLE_VALUE)
    {
        fs_log_output("[Trifecta-Interface] Failed to open serial port %s (Err=%lu)",
                      port_name, GetLastError());
        return -3;
    }

    if (configure_serial_port(hSerial) != 0)
    {
        CloseHandle(hSerial);
        return -4;
    }

    // After init, serial_port now holds the HANDLE (just like Linux stores the FD)
    fs_set_serial_handle(device_handle, hSerial);

    return 0;
}

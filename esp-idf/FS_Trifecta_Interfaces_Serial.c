/// Driver for the Trifecta series of IMU/AHRS/INS devices
/// Copyright 2025 4rge.ai and/or Triangle Man LLC
/// Usage and redistribution of this code is permitted
/// but this notice must be retained in all copies of the code.

/// THIS SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
/// INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE,
/// AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
/// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include "sdkconfig.h"

#define CDC_AVAILABLE (CONFIG_USB_ENABLED)

#include "driver/uart.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"

#include "FS_Trifecta_Interfaces.h"

#if (CONFIG_USB_ENABLED)
#include "usb/usb_host.h"
#include "usb/cdc_acm_host.h"

#define MAX_TRIFECTA_CDC_DEVICES 4

typedef struct
{
    int id;
    cdc_acm_dev_hdl_t handle;
    bool active;
} fs_cdc_handle_entry_t;

cdc_handle_entry_t cdc_table[MAX_TRIFECTA_CDC_DEVICES];

cdc_acm_dev_hdl_t get_handle_from_id(int id)
{
    for (int i = 0; i < MAX_TRIFECTA_CDC_DEVICES; ++i)
    {
        if (cdc_table[i].id == id && cdc_table[i].active)
        {
            return cdc_table[i].handle;
        }
    }
    return NULL;
}

#endif

/// @brief Start the network serial driver.
/// @param device_handle
/// @return
int fs_init_serial_driver(fs_device_info_t *device_handle)
{
    // On FreeRTOS/microcontroller systems, the serial port is usually a fixed number, so port scanning will not be done
    // Only check to ensure that the serial port was previously set up
    if (device_handle->device_params.serial_port < 0)
    {
        fs_log_output("[Trifecta] Serial port number cannot be less than zero!");
        return -1;
    }
    if (device_handle->device_params.communication_mode == FS_COMMUNICATION_MODE_UART)
    {
        // On ESP-IDF, this is necessary to prevent bytes from being discarded at high baud rates
        ESP_ERROR_CHECK(uart_set_rx_full_threshold(device_handle->device_params.serial_port, 64));
    }
    else if (device_handle->device_params.communication_mode == FS_COMMUNICATION_MODE_USB_CDC)
    {
#if (CDC_AVAILABLE)
        // Exclude device from interrupt-based handling mode
#else
        fs_log_output("[Trifecta] CDC ACM host init failed! Features not enabled on device!");
        return -1;
#endif
    }
    else if (device_handle->device_params.communication_mode == FS_COMMUNICATION_MODE_I2C)
    {
        // Custom setup logic for I2C to be added here, if any exists.
    }
    else if (device_handle->device_params.communication_mode == FS_COMMUNICATION_MODE_SPI)
    {
        // Custom setup logic for SPI to be added here, if any exists.
    }
    return 0;
}

/// @brief Start serial in interrupt mode on platforms that support it.
/// This enables more precise and low latency serial reads than polling.
/// @param device_handle
/// @param status_flag
/// @return 0 on success, -1 on fail (e.g. not supported on platform)
int fs_platform_supported_serial_interrupts()
{
    return FS_COMMUNICATION_MODE_UNINITIALIZED; // TODO:
}

/// @brief Start serial in interrupt mode on platforms that support it.
/// This enables more precise and low latency serial reads than polling.
/// @param device_handle
/// @param status_flag
/// @return 0 on success, -1 on fail (e.g. not supported on platform)
int fs_init_serial_interrupts(fs_device_info_t *device_handle, fs_run_status_t *status_flag)
{
    return -1; // TODO:
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

    if (device_handle->device_params.communication_mode != FS_COMMUNICATION_MODE_UART && device_handle->device_params.communication_mode != FS_COMMUNICATION_MODE_USB_CDC && device_handle->device_params.communication_mode != FS_COMMUNICATION_MODE_I2C && device_handle->device_params.communication_mode != FS_COMMUNICATION_MODE_SPI)
    {
        fs_log_output("[Trifecta] Error: Invalid communication mode! Expected COMMUNICATION_MODE_SERIAL.");
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

    ssize_t actual_len = 0;
    if (device_handle->device_params.communication_mode == FS_COMMUNICATION_MODE_UART)
    {
        actual_len = uart_write_bytes(device_handle->device_params.serial_port, (char *)tx_buffer, length_bytes);
        if (actual_len != length_bytes)
        {
            fs_log_output("[Trifecta] Error: Writing data over serial failed! Expected length: %ld, actual: %ld", length_bytes, actual_len);
            return -1;
        }
    }
    else if (device_handle->device_params.communication_mode == FS_COMMUNICATION_MODE_USB_CDC)
    {
        // TODO: CDC ACM host write
    }
    else if (device_handle->device_params.communication_mode == FS_COMMUNICATION_MODE_I2C)
    {
        // TODO: I2C write function
    }
    else if (device_handle->device_params.communication_mode == FS_COMMUNICATION_MODE_SPI)
    {
        // TODO: SPI write function
    }

    fs_log_output("[Trifecta] Serial transmit to port %d - Length %ld - data %s", device_handle->device_params.serial_port, actual_len, tx_buffer);

    return actual_len;
}

/// @brief Receive data over serial communication
/// @param device_handle Pointer to the device information structure
/// @param rx_buffer Pointer to the receive data buffer (this buffer is cleared on read)
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

    if (device_handle->device_params.communication_mode != FS_COMMUNICATION_MODE_UART && device_handle->device_params.communication_mode != FS_COMMUNICATION_MODE_USB_CDC && device_handle->device_params.communication_mode != FS_COMMUNICATION_MODE_I2C && device_handle->device_params.communication_mode != FS_COMMUNICATION_MODE_SPI)
    {
        fs_log_output("[Trifecta] Error: Invalid communication mode! Expected COMMUNICATION_MODE_SERIAL.");
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
    // Clear only the specified buffer size
    memset(rx_buffer, 0, length_bytes);

    size_t buffer_data_len = 0;
    if (uart_get_buffered_data_len(device_handle->device_params.serial_port, (size_t *)&buffer_data_len) != 0)
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

    ssize_t rx_len = 0;
    if (device_handle->device_params.communication_mode == FS_COMMUNICATION_MODE_UART)
    {
        rx_len = uart_read_bytes(device_handle->device_params.serial_port, rx_buffer, buffer_data_len, timeout_micros / 1000);
        if (rx_len > 0)
        {
            fs_log_output("[Trifecta] Read data from port %d - length %d!", device_handle->device_params.serial_port, rx_len);
        }
        else if (rx_len < 0)
        {
            fs_log_output("[Trifecta] Error: Reading data over serial failed!");
            uart_flush(device_handle->device_params.serial_port);
            return -1;
        }
    }
    else if (device_handle->device_params.communication_mode == FS_COMMUNICATION_MODE_USB_CDC)
    {
        // TODO: CDC ACM host read
    }
    else if (device_handle->device_params.communication_mode == FS_COMMUNICATION_MODE_I2C)
    {
        // TODO: I2C read function
    }
    else if (device_handle->device_params.communication_mode == FS_COMMUNICATION_MODE_SPI)
    {
        // TODO: SPI read function
    }

    return rx_len;
}

/// @brief Shutdown the serial driver.
/// @param device_handle Pointer to the device information structure.
/// @return 0 if successful, -1 if failed.
int fs_shutdown_serial_driver(fs_device_info_t *device_handle)
{
    if (device_handle->device_params.communication_mode == FS_COMMUNICATION_MODE_UART)
    {
        // Edit: On ESP32 platforms, the serial driver is externally managed, do not close manually.
        // if (uart_driver_delete(device_handle->device_params.serial_port) != 0)
        // {
        //     fs_log_output("[Trifecta] Warning: Failed to delete UART driver (serial port: %d)!", device_handle->device_params.serial_port);
        //     device_handle->device_params.serial_port = -1;
        //     return -1;
        // }
    }
    else if (device_handle->device_params.communication_mode == FS_COMMUNICATION_MODE_USB_CDC)
    {
        // TODO: CDC ACM host shutdown
    }
    else if (device_handle->device_params.communication_mode == FS_COMMUNICATION_MODE_I2C)
    {
        // TODO: I2C shutdown function
    }
    else if (device_handle->device_params.communication_mode == FS_COMMUNICATION_MODE_SPI)
    {
        // TODO: SPI shutdown function
    }
    device_handle->device_params.serial_port = -1;
    return 0;
}

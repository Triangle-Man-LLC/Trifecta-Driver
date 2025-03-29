/// Driver for the Trifecta series of IMU/AHRS/INS devices
/// Copyright 2024 4rge.ai and/or Triangle Man LLC
/// Usage and redistribution of this code is permitted
/// but this notice must be retained in all copies of the code.

/// THIS SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
/// INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE,
/// AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
/// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include "FS_Trifecta_Serial.h"

/// @brief Updater thread, there is one of these per device connected.
/// @param params Passes the device handle to the thread.
/// @return
static void fs_serial_update_thread(void *params)
{
    if (params == NULL)
    {
        fs_log_output("[Trifecta] Error: Serial thread params point to an invalid instance of fs_device_info!");
        fs_thread_exit(NULL);
        return;
    }

    fs_device_info *active_device = (fs_device_info *)params;
    const int delay_time_millis = active_device->driver_config.task_wait_ms;
    const int receive_timeout_micros = active_device->driver_config.read_timeout_micros;
    active_device->status = FS_RUN_STATUS_RUNNING;

    uint8_t rx_buffer[FS_MAX_DATA_LENGTH] = {0};

    fs_log_output("[Trifecta] Device %s parameters: Run status: %d, Delay time %d ms, Receive timeout %d us",
                  active_device->device_name, active_device->status, delay_time_millis, receive_timeout_micros);

    size_t last_received_serial = 0;

    while (active_device->status == FS_RUN_STATUS_RUNNING)
    {
        last_received_serial = fs_receive_serial(active_device, rx_buffer, FS_MAX_DATA_LENGTH, receive_timeout_micros);
        while (last_received_serial > 0)
        {
            fs_log_output("[Trifecta] SERIAL:RX LEN %d, DATA: %s", last_received_serial, rx_buffer);
            if (fs_device_parse_packet(active_device, rx_buffer, last_received_serial, FS_COMMUNICATION_MODE_SERIAL) < 0)
            {
                fs_log_output("[Trifecta] WARN: Could not parse data! Is there corruption?");
            }
            last_received_serial = fs_receive_serial(active_device, rx_buffer, FS_MAX_DATA_LENGTH, receive_timeout_micros);
        }
        fs_delay(delay_time_millis);
    }

    fs_thread_exit(NULL);
    return;
}

/// @brief Generic message send over serial.
/// @param device_handle Pointer to the device information structure.
/// @return Status code indicating success or failure.
int fs_serial_send_message(fs_device_info *device_handle, char *message, size_t len)
{
    const int receive_timeout_micros = 1000;
    return (fs_transmit_serial(device_handle, message, len, receive_timeout_micros) > 0) ? 0 : -1;
}

/// @brief Starts the serial communication with the device.
/// @param device_handle Pointer to the device information structure.
/// @return Status code indicating success or failure.
int fs_serial_start(fs_device_info *device_handle)
{
    // Clear the device name
    memset(&device_handle->device_name, 0, sizeof(device_handle->device_name));

    // Initialize the serial driver
    int status = fs_init_serial_driver(device_handle);
    if (status != 0)
    {
        fs_log_output("[Trifecta] Error: Could not start serial driver!\n");
        return status;
    }

    // Define thread parameters
    const int task_priority = device_handle->driver_config.background_task_priority;
    const int core_affinity = device_handle->driver_config.background_task_core_affinity;
    const int task_stack_size = device_handle->driver_config.task_stack_size_bytes;

    // Short delay before starting the thread
    fs_delay(10);

    // Send identification command
    char send_buf[16] = {0};
    snprintf(send_buf, sizeof(send_buf), ";%c%d;", CMD_IDENTIFY, 0);
    size_t send_len = strnlen(send_buf, sizeof(send_buf)) + 1;

    const int receive_timeout_micros = 100000;

    // Retry connection attempts
    const int connection_retries = 10;
    for (int i = 0; i < connection_retries; i++)
    {
        status = (fs_transmit_serial(device_handle, send_buf, send_len, receive_timeout_micros) > 0) ? 0 : -1;
        fs_delay(receive_timeout_micros / 1000);
        if (status != 0)
        {
            fs_log_output("[Trifecta] Error: Could not transmit identification command!");
            continue;
        }

        uint8_t rx_buffer[FS_MAX_CMD_LENGTH] = {0};

        size_t last_received_serial = fs_receive_serial(device_handle, rx_buffer, FS_MAX_CMD_LENGTH, receive_timeout_micros);
        if (last_received_serial > 0)
        {
            fs_log_output("[Trifecta] SERIAL:RX LEN %d, DATA: %s", last_received_serial, rx_buffer);
            if (fs_handle_received_commands(device_handle, rx_buffer, last_received_serial) == 0)
            {
                break;
            }
            last_received_serial = fs_receive_serial(device_handle, rx_buffer, FS_MAX_CMD_LENGTH, receive_timeout_micros);
        }
        fs_delay(receive_timeout_micros / 1000);
    }
    if (strnlen(device_handle->device_name, sizeof(device_handle->device_name)) > 0)
    {
        fs_log_output("[Trifecta] Connected to device! Device name: %s", device_handle->device_name);

        fs_log_output("[Trifecta] Device %s parameters: Run status: %d, Delay time %d ms, Receive timeout %d us",
                      device_handle->device_name, device_handle->status, device_handle->driver_config.task_wait_ms, device_handle->driver_config.read_timeout_micros);

        // Start the serial update thread
        status = fs_thread_start(fs_serial_update_thread, (void *)device_handle, &device_handle->status, task_stack_size, task_priority, core_affinity);
        if (status != 0)
        {
            fs_log_output("[Trifecta] Error: Could not start serial thread!");
            return -1;
        }
        return 0;
    }

    // If no connection was made, log an error
    fs_log_output("[Trifecta] Error: Device was not detected!");
    return -1;
}

/// @brief
/// @param device_handle
/// @return
int fs_serial_start_device_stream(fs_device_info *device_handle)
{
    char send_buf[16] = {0};
    snprintf(send_buf, 16, ";%c%d;", CMD_STREAM, 1);
    size_t send_len = strnlen(send_buf, 16) + 1;
    const int receive_timeout_micros = 1000;
    return (fs_transmit_serial(device_handle, send_buf, send_len, receive_timeout_micros) > 0) ? 0 : -1;
}

/// @brief
/// @param device_handle
/// @return
int fs_serial_stop_device_stream(fs_device_info *device_handle)
{
    char send_buf[16] = {0};
    snprintf(send_buf, 16, ";%c%d;", CMD_STREAM, 0);
    size_t send_len = strnlen(send_buf, 16) + 1;
    const int receive_timeout_micros = 1000;
    return (fs_transmit_serial(device_handle, send_buf, send_len, receive_timeout_micros) > 0) ? 0 : -1;
}

/// @brief
/// @param device_handle
/// @return
int fs_serial_read_one_shot(fs_device_info *device_handle)
{
    char send_buf[16] = {0};
    snprintf(send_buf, 16, ";%c%d;", CMD_STREAM, 2);
    size_t send_len = strnlen(send_buf, 16) + 1;
    const int receive_timeout_micros = 1000;
    return (fs_transmit_serial(device_handle, send_buf, send_len, receive_timeout_micros) > 0) ? 0 : -1;
}

/// @brief
/// @param device_handle
/// @return
int fs_serial_exit(fs_device_info *device_handle)
{
    device_handle->status = FS_RUN_STATUS_IDLE;

    char send_buf[16] = {0};
    snprintf(send_buf, 16, ";%c%d;", CMD_STREAM, 0);
    size_t send_len = strnlen(send_buf, 16) + 1;
    const int receive_timeout_micros = 1000;
    fs_transmit_serial(device_handle, send_buf, send_len, receive_timeout_micros);

    fs_delay(5);
    return fs_shutdown_serial_driver(device_handle);
}

/// @brief
/// @param device_handle
/// @return
int fs_serial_device_restart(fs_device_info *device_handle)
{
    char send_buf[16] = {0};
    snprintf(send_buf, 16, ";%c%d;", CMD_RESTART, 0);
    size_t send_len = strnlen(send_buf, 16) + 1;
    const int receive_timeout_micros = 1000;
    return (fs_transmit_serial(device_handle, send_buf, send_len, receive_timeout_micros) > 0) ? 0 : -1;
}

/// @brief
/// @param device_handle
/// @return
int fs_serial_set_device_operating_mode(fs_device_info *device_handle, fs_communication_mode mode)
{
    char send_buf[16] = {0};
    snprintf(send_buf, 16, ";%c%d;", CMD_IDENTIFY_PARAM_TRANSMIT, mode);
    size_t send_len = strnlen(send_buf, 16) + 1;
    const int receive_timeout_micros = 1000;
    return (fs_transmit_serial(device_handle, send_buf, send_len, receive_timeout_micros) > 0) ? 0 : -1;
}

/// @brief
/// @param device_handle The device handle
/// @param ssid Null-terminated string with SSID
/// @param password Null-terminated string with password
/// @return
int fs_serial_set_network_params(fs_device_info *device_handle, char *ssid, char *password)
{
    char send_buf[128] = {0};
    snprintf(send_buf, 128, ";%c%s;%c%s;", CMD_SET_SSID, ssid, CMD_SET_PASSWORD, ssid);
    size_t send_len = strnlen(send_buf, 16) + 1;
    const int receive_timeout_micros = 1000;
    return (fs_transmit_serial(device_handle, send_buf, send_len, receive_timeout_micros) > 0) ? 0 : -1;
}

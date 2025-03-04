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

static bool serial_running = false;

static fs_driver_config *config = NULL;

/// @brief Updater thread
/// @param params
/// @return
static void fs_serial_update_thread(void *params)
{
    if (params == NULL)
    {
        fs_log_output("[Trifecta] Error: Serial thread params point to an invalid instance of fs_device_info!");
        fs_thread_exit();
        return;
    }

    fs_device_info *active_device = (fs_device_info *)params;
    const int delay_time_millis = config->task_wait_ms;
    const int receive_timeout_micros = config->read_timeout_micros;

    int rx_buffer[FS_MAX_DATA_LENGTH] = {0};
    memset(rx_buffer, 0, FS_MAX_DATA_LENGTH);

    size_t last_received_serial = 0;

    while (serial_running)
    {
        last_received_serial = fs_receive_serial(active_device, rx_buffer, FS_MAX_DATA_LENGTH, receive_timeout_micros);
        while (last_received_serial > 0)
        {
            if (fs_device_parse_packet(active_device, rx_buffer, last_received_serial) < 0)
            {
                fs_log_output("[Trifecta] Could not parse data! Is there corruption?");
            }
            last_received_serial = fs_receive_serial(active_device, rx_buffer, FS_MAX_DATA_LENGTH, receive_timeout_micros); 
        }
        fs_delay(delay_time_millis);
    }
    fs_thread_exit();
    return;
}

int fs_serial_set_driver_config(fs_driver_config *driver_config)
{
    if(driver_config == NULL)
    {
        return -1;
    }
    config = driver_config;
    return 0;
}

/// @brief Generic message send over serial.
/// @param device_handle Pointer to the device information structure.
/// @return Status code indicating success or failure.
int fs_serial_send_message(fs_device_info *device_handle, char* message, size_t len)
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
    memset(device_handle->device_name, 0, sizeof(device_handle->device_name));

    // Initialize the serial driver
    int status = fs_init_serial_driver(device_handle);
    if (status != 0)
    {
        fs_log_output("[Trifecta] Error: Could not start serial driver!\n");
        return status;
    }

    if (config == NULL)
    {
        fs_log_output("[Trifecta] Error: Did not set driver config!\n");
        return status;
    }

    // Define thread parameters
    const int task_priority = config->background_task_priority;
    const int core_affinity = config->background_task_core_affinity;
    const int task_stack_size = config->task_stack_size_bytes;

    // Short delay before starting the thread
    fs_delay(10);

    // Start the serial update thread
    status = fs_thread_start(fs_serial_update_thread, (void *)device_handle, &serial_running, task_stack_size, task_priority, core_affinity);
    if (status != 0)
    {
        fs_log_output("[Trifecta] Error: Could not start serial thread!");
        return -1;
    }

    // Send identification command
    char send_buf[16] = {0};
    snprintf(send_buf, sizeof(send_buf), ";%c%d;", CMD_IDENTIFY, 0);
    size_t send_len = strnlen(send_buf, sizeof(send_buf)) + 1;

    const int receive_timeout_micros = 10000;

    status = (fs_transmit_serial(device_handle, send_buf, send_len, receive_timeout_micros) > 0) ? 0 : -1;
    fs_delay(receive_timeout_micros / 1000);
    status = (fs_transmit_serial(device_handle, send_buf, send_len, receive_timeout_micros) > 0) ? 0 : -1;
    fs_delay(receive_timeout_micros / 1000);
    status = (fs_transmit_serial(device_handle, send_buf, send_len, receive_timeout_micros) > 0) ? 0 : -1;
    fs_delay(receive_timeout_micros / 1000);

    if (status != 0)
    {
        fs_log_output("[Trifecta] Error: Could not transmit identification command!");
        return status;
    }

    // Retry connection attempts
    const int connection_retries = 10;
    for (int i = 0; i < connection_retries; i++)
    {
        fs_delay(receive_timeout_micros / 1000);
        if (strnlen(device_handle->device_name, sizeof(device_handle->device_name)) > 0)
        {
            fs_log_output("[Trifecta] Connected to device! Device name: %s", device_handle->device_name);
            return 0;
        }
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
    serial_running = false;

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

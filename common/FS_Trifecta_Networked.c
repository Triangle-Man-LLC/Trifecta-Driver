/// Driver for the Trifecta series of IMU/AHRS/INS devices
/// Copyright 2024 4rge.ai and/or Triangle Man LLC
/// Usage and redistribution of this code is permitted
/// but this notice must be retained in all copies of the code.

/// THIS SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
/// INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE,
/// AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
/// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include "FS_Trifecta_Networked.h"

static bool networked_running = false;

static fs_driver_config *config = NULL;

/// @brief Updater thread
/// @param params
/// @return
static void fs_network_update_thread(void *params)
{
    if (params == NULL)
    {
        fs_log_output("[Trifecta] Error: Network thread params point to an invalid instance of fs_device_info!");
        fs_thread_exit();
        return;
    }

    fs_device_info *active_device = (fs_device_info *)params;
    const int delay_time_millis = config->task_wait_ms;
    const int receive_timeout_micros = config->read_timeout_micros;

    int rx_buffer[FS_MAX_DATA_LENGTH] = {0};
    memset(rx_buffer, 0, FS_MAX_DATA_LENGTH);

    size_t last_received_tcp = 0;
    size_t last_received_udp = 0;

    while (networked_running)
    {
        last_received_tcp = fs_receive_networked_tcp(active_device, &rx_buffer, FS_MAX_DATA_LENGTH, receive_timeout_micros);
        fs_log_output("[Trifecta] TCP:RX %d", last_received_tcp);
        if (last_received_tcp > 0)
        {
            // TCP only receive command responses, so handle them here
            if (fs_handle_received_commands(active_device, rx_buffer, last_received_tcp) < 0)
            {
                fs_log_output("[Trifecta] Could not parse data! Is there interference?");
            }
        }
        memset(rx_buffer, 0, FS_MAX_DATA_LENGTH);
        last_received_udp = fs_receive_networked_udp(active_device, &rx_buffer, FS_MAX_DATA_LENGTH, receive_timeout_micros);
        fs_log_output("[Trifecta] UDP:RX %d", last_received_udp);
        if (last_received_udp > 0)
        {
            // UDP only receive data packets, so handle them here
            if (fs_device_parse_packet(active_device, rx_buffer, last_received_udp) < 0)
            {
                fs_log_output("[Trifecta] Could not parse data! Is there interference?");
            }
        }
        fs_delay(delay_time_millis);
    }
    fs_thread_exit();
    return;
}

int fs_network_set_driver_config(fs_driver_config *driver_config)
{
    if (driver_config == NULL)
    {
        return -1;
    }
    config = driver_config;
    return 0;
}

/// @brief Generic message send over network (TCP).
/// @param device_handle Pointer to the device information structure.
/// @return Status code indicating success or failure.
int fs_network_send_message(fs_device_info *device_handle, char *message, size_t len)
{
    const int transmit_timeout_micros = 10000;
    return fs_transmit_networked_tcp(device_handle, message, len, transmit_timeout_micros);
}

/// @brief Starts the network connection and associated thread.
/// @param ip_addr The IP address to connect to.
/// @param device_handle Handle to the network device.
/// @return Status of the network start operation (0 for success, -1 for failure).
int fs_network_start(const char *ip_addr, fs_device_info *device_handle)
{
    // Clear the device name and parameters
    memset(device_handle->device_name, 0, sizeof(device_handle->device_name));
    memset(device_handle->ip_addr, 0, sizeof(device_handle->ip_addr));

    // Set target IP address
    strncpy(device_handle->ip_addr, ip_addr, sizeof(device_handle->ip_addr) - 1);
    device_handle->ip_port = FS_TRIFECTA_PORT;

    // Initialize TCP driver
    if (fs_init_network_tcp_driver(device_handle) != 0)
    {
        fs_log_output("[Trifecta] Error: Could not start TCP server!\n");
        device_handle->communication_mode = FS_COMMUNICATION_MODE_UNINITIALIZED;
        return -1;
    }

    // Initialize UDP driver
    if (fs_init_network_udp_driver(device_handle) != 0)
    {
        fs_log_output("[Trifecta] Error: Could not start UDP server!\n");
        device_handle->communication_mode = FS_COMMUNICATION_MODE_UNINITIALIZED;
        return -1;
    }

    if (config == NULL)
    {
        fs_log_output("[Trifecta] Error: Did not set driver config!\n");
        return -1;
    }

    // Define thread parameters
    const int task_priority = config->background_task_priority;
    const int core_affinity = config->background_task_core_affinity;
    const int task_stack_size = config->task_stack_size_bytes;

    int status = fs_thread_start(fs_network_update_thread, (void *)device_handle, &networked_running, task_stack_size, task_priority, core_affinity);

    if (status != 0)
    {
        fs_log_output("[Trifecta] Error: Could not start network thread!\n");
        return -1;
    }

    // Send identification command
    char send_buf[16] = {0};
    snprintf(send_buf, sizeof(send_buf), ";%c%d;", CMD_IDENTIFY, 0);
    size_t send_len = strnlen(send_buf, sizeof(send_buf)) + 1;

    const int receive_timeout_micros = 10000;

    // Retry connection attempts
    const int connection_retries = 10;

    for (int i = 0; i < connection_retries; i++)
    {
        status = (fs_transmit_networked_tcp(device_handle, send_buf, send_len, receive_timeout_micros) > 0) ? 0 : -1;
        fs_delay(receive_timeout_micros / 1000);
        if (strnlen(device_handle->device_name, sizeof(device_handle->device_name)) > 0)
        {
            fs_log_output("[Trifecta] Connected to device! Device name: %s", device_handle->device_name);
            return 0;
        }
    }

    fs_log_output("[Trifecta] Network thread started successfully.\n");
    return 0;
}

/// @brief Starts streaming data from the device over the network.
/// @param device_handle Handle to the network device.
/// @return Status of the command transmission (0 for success, -1 for failure).
int fs_network_start_device_stream(fs_device_info *device_handle)
{
    char send_buf[16] = {0};
    snprintf(send_buf, 16, "%c%d;", CMD_STREAM, 1);
    size_t send_len = strnlen(send_buf, 16);
    int transmit_timeout_micros = 10000;
    return fs_transmit_networked_tcp(device_handle, &send_buf, send_len, transmit_timeout_micros);
}

/// @brief Stops streaming data from the device over the network.
/// @param device_handle Handle to the network device.
/// @return Status of the command transmission (0 for success, -1 for failure).
int fs_network_stop_device_stream(fs_device_info *device_handle)
{
    char send_buf[16] = {0};
    snprintf(send_buf, 16, "%c%d;", CMD_STREAM, 0);
    size_t send_len = strnlen(send_buf, 16);
    int transmit_timeout_micros = 10000;
    return fs_transmit_networked_tcp(device_handle, &send_buf, send_len, transmit_timeout_micros);
}

/// @brief Stops streaming data from the device over the network.
/// @param device_handle Handle to the network device.
/// @return Status of the command transmission (0 for success, -1 for failure).
int fs_network_read_one_shot(fs_device_info *device_handle)
{
    char send_buf[16] = {0};
    snprintf(send_buf, 16, "%c%d;", CMD_STREAM, 2);
    size_t send_len = strnlen(send_buf, 16);
    int transmit_timeout_micros = 10000;
    return fs_transmit_networked_tcp(device_handle, &send_buf, send_len, transmit_timeout_micros);
}

/// @brief Deallocate all allocated network resources.
/// @param device_handle Handle to the network device.
/// @return Status of the network shutdown (0 for success, -1 for failure).
int fs_network_exit(fs_device_info *device_handle)
{
    char send_buf[16] = {0};
    networked_running = false; // Stop network-related activities

    // Prepare the command to stop streaming
    snprintf(send_buf, 16, "%c%d;", CMD_STREAM, 0);
    size_t send_len = strnlen(send_buf, 16);
    int transmit_timeout_micros = 10000;

    // Transmit the stop command over TCP
    fs_transmit_networked_tcp(device_handle, &send_buf, send_len, transmit_timeout_micros);

    // Delay to ensure the command is processed
    fs_delay(5);

    // Shutdown both TCP and UDP drivers, combining their return statuses
    return fs_shutdown_network_tcp_driver(device_handle) | fs_shutdown_network_udp_driver(device_handle);
}

/// @brief
/// @param device_handle
/// @return
int fs_network_device_restart(fs_device_info *device_handle)
{
    char send_buf[16] = {0};
    snprintf(send_buf, 16, ";%c%d;", CMD_RESTART, 0);
    size_t send_len = strnlen(send_buf, 16) + 1;
    const int receive_timeout_micros = 10000;
    return (fs_transmit_networked_tcp(device_handle, send_buf, send_len, receive_timeout_micros) > 0) ? 0 : -1;
}

/// @brief
/// @param device_handle
/// @return
int fs_network_set_device_operating_mode(fs_device_info *device_handle, fs_communication_mode mode)
{
    char send_buf[16] = {0};
    snprintf(send_buf, 16, ";%c%d;", CMD_IDENTIFY_PARAM_TRANSMIT, mode);
    size_t send_len = strnlen(send_buf, 16) + 1;
    const int receive_timeout_micros = 10000;
    return (fs_transmit_networked_tcp(device_handle, send_buf, send_len, receive_timeout_micros) > 0) ? 0 : -1;
}

/// @brief
/// @param device_handle The device handle
/// @param ssid Null-terminated string with SSID
/// @param password Null-terminated string with password
/// @return
int fs_network_set_network_params(fs_device_info *device_handle, char *ssid, char *password)
{
    char send_buf[128] = {0};
    snprintf(send_buf, 128, ";%c%s;%c%s;", CMD_SET_SSID, ssid, CMD_SET_PASSWORD, ssid);
    size_t send_len = strnlen(send_buf, 16) + 1;
    const int receive_timeout_micros = 10000;
    return (fs_transmit_networked_tcp(device_handle, send_buf, send_len, receive_timeout_micros) > 0) ? 0 : -1;
}

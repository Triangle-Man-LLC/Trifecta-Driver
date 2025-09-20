/// Driver for the Trifecta series of IMU/AHRS/INS devices
/// Copyright 2025 4rge.ai and/or Triangle Man LLC
/// Usage and redistribution of this code is permitted
/// but this notice must be retained in all copies of the code.

/// THIS SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
/// INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE,
/// AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
/// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include "FS_Trifecta_Defs.h"
#include "FS_Trifecta_Networked.h"

/// @brief This counter increments when more network devices are added, since it is necessary to bind UDP sockets to unique ports
static int num_network_devices = 0; 

/// @brief Updater thread, there is one of these per device connected.
/// @param params Passes the device handle to the thread.
/// @return
static void fs_network_update_thread(void *params)
{
    if (params == NULL)
    {
        fs_log_output("[Trifecta] Error: Network thread params point to an invalid instance of fs_device_info_t!");
        fs_thread_exit(NULL);
        return;
    }

    fs_device_info_t *active_device = (fs_device_info_t *)params;
    const int delay_time_millis = active_device->driver_config.task_wait_ms;
    const int receive_timeout_micros = active_device->driver_config.read_timeout_micros;
    active_device->status = FS_RUN_STATUS_RUNNING;

    uint8_t rx_buffer[FS_MAX_DATA_LENGTH] = {0};
    memset(rx_buffer, 0, FS_MAX_DATA_LENGTH);
    uint8_t rx_buffer2[FS_MAX_DATA_LENGTH] = {0};
    memset(rx_buffer2, 0, FS_MAX_DATA_LENGTH);

    fs_log_output("[Trifecta] Device %s parameters: Run status: %d, Delay time %d ms, Receive timeout %d us",
                  active_device->device_name, active_device->status, delay_time_millis, receive_timeout_micros);

    ssize_t last_received_tcp = 0;
    ssize_t last_received_udp = 0;

    while (active_device->status == FS_RUN_STATUS_RUNNING)
    {
        last_received_tcp = fs_receive_networked_tcp(active_device, &rx_buffer, FS_MAX_DATA_LENGTH, receive_timeout_micros);
        // fs_log_output("[Trifecta] TCP:RX %d", last_received_tcp);
        if (last_received_tcp > 0)
        {
            // TCP only receive command responses, so handle them here
            if (fs_handle_received_commands(active_device, &rx_buffer, last_received_tcp) < 0)
            {
                fs_log_output("[Trifecta] Could not parse data! Is there interference?");
            }
        }
        memset(rx_buffer, 0, FS_MAX_DATA_LENGTH);
        last_received_udp = fs_receive_networked_udp(active_device, &rx_buffer2, FS_MAX_DATA_LENGTH, receive_timeout_micros);
        // fs_log_output("[Trifecta] UDP:RX %d", last_received_udp);
        if (last_received_udp > 0)
        {
            // UDP only receive data packets, so handle them here
            if (fs_device_parse_packet(active_device, &rx_buffer2, last_received_udp, active_device->communication_mode) < 0)
            {
                fs_log_output("[Trifecta] Could not parse data! Is there interference?");
            }
        }
        memset(rx_buffer2, 0, FS_MAX_DATA_LENGTH);
        fs_delay(delay_time_millis);
    }

    fs_thread_exit(NULL);
    return;
}

/// @brief Generic message send over network (TCP).
/// @param device_handle Pointer to the device information structure.
/// @return Status code indicating success or failure.
int fs_network_send_message(fs_device_info_t *device_handle, char *message, size_t len)
{
    const int transmit_timeout_micros = 10000;
    return fs_transmit_networked_tcp(device_handle, message, len, transmit_timeout_micros);
}

/// @brief Starts the network connection and associated thread.
/// @param ip_addr The IP address to connect to.
/// @param device_handle Handle to the network device.
/// @return Status of the network start operation (0 for success, -1 for failure).
int fs_network_start(const char *ip_addr, fs_device_info_t *device_handle)
{
    // Clear the device name and parameters
    memset(device_handle->device_name, 0, sizeof(device_handle->device_name));
    memset(device_handle->device_params.ip_addr, 0, sizeof(device_handle->device_params.ip_addr));
    // Set target IP address
    fs_safe_strncpy(device_handle->device_params.ip_addr, ip_addr, sizeof(device_handle->device_params.ip_addr) - 1);
    device_handle->device_params.tcp_port = FS_TRIFECTA_PORT; // TCP port always 8888

    // Initialize TCP driver
    if (fs_init_network_tcp_driver(device_handle) != 0)
    {
        fs_log_output("[Trifecta] Error: Could not start TCP server!");
        device_handle->communication_mode = FS_COMMUNICATION_MODE_UNINITIALIZED;
        return -1;
    }

    // Begin UDP after establishing connection
    // Note that UDP port should be incremented to enable support for multiple devices
    device_handle->device_params.udp_port = FS_TRIFECTA_PORT + num_network_devices; // Starting at 8888, then 8889, etc.
    num_network_devices ++;
    if (fs_network_set_host_udp_port(device_handle, device_handle->device_params.udp_port) != 0)
    {
        fs_log_output("[Trifecta] Error: Could not set UDP host port!");
        device_handle->communication_mode = FS_COMMUNICATION_MODE_UNINITIALIZED;
        return -1;
    }

    // Initialize UDP driver
    if (fs_init_network_udp_driver(device_handle) != 0)
    {
        fs_log_output("[Trifecta] Error: Could not start UDP server!");
        device_handle->communication_mode = FS_COMMUNICATION_MODE_UNINITIALIZED;
        return -1;
    }
    
    // Define thread parameters
    const int task_priority = device_handle->driver_config.background_task_priority;
    const int core_affinity = device_handle->driver_config.background_task_core_affinity;
    const int task_stack_size = device_handle->driver_config.task_stack_size_bytes;

    int status = fs_thread_start(fs_network_update_thread, (void *)device_handle, &device_handle->status, task_stack_size, task_priority, core_affinity);

    if (status != 0)
    {
        fs_log_output("[Trifecta] Error: Could not start network thread!\n");
        return -1;
    }

    // Send identification command
    char send_buf[FS_MAX_CMD_LENGTH] = {0};
    snprintf(send_buf, sizeof(send_buf), ";%c%d;", CMD_IDENTIFY, 0);
    size_t send_len = fs_safe_strnlen(send_buf, sizeof(send_buf)) + 1;

    const int receive_timeout_micros = 10000;

    // Retry connection attempts
    const int connection_retries = 10;

    for (int i = 0; i < connection_retries; i++)
    {
        status = (fs_transmit_networked_tcp(device_handle, send_buf, send_len, receive_timeout_micros) > 0) ? 0 : -1;
        fs_delay(receive_timeout_micros / 1000);
        if (fs_safe_strnlen(device_handle->device_name, sizeof(device_handle->device_name)) > 0)
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
int fs_network_start_device_stream(fs_device_info_t *device_handle)
{
    char send_buf[FS_MAX_CMD_LENGTH] = {0};
    snprintf(send_buf, FS_MAX_CMD_LENGTH, "%c%d;", CMD_STREAM, 1);
    size_t send_len = fs_safe_strnlen(send_buf, FS_MAX_CMD_LENGTH);
    int transmit_timeout_micros = 10000;
    return fs_transmit_networked_tcp(device_handle, &send_buf, send_len, transmit_timeout_micros);
}

/// @brief Stops streaming data from the device over the network.
/// @param device_handle Handle to the network device.
/// @return Status of the command transmission (0 for success, -1 for failure).
int fs_network_stop_device_stream(fs_device_info_t *device_handle)
{
    char send_buf[FS_MAX_CMD_LENGTH] = {0};
    snprintf(send_buf, FS_MAX_CMD_LENGTH, "%c%d;", CMD_STREAM, 0);
    size_t send_len = fs_safe_strnlen(send_buf, FS_MAX_CMD_LENGTH);
    int transmit_timeout_micros = 10000;
    return fs_transmit_networked_tcp(device_handle, &send_buf, send_len, transmit_timeout_micros);
}

/// @brief Stops streaming data from the device over the network.
/// @param device_handle Handle to the network device.
/// @return Status of the command transmission (0 for success, -1 for failure).
int fs_network_read_one_shot(fs_device_info_t *device_handle)
{
    char send_buf[FS_MAX_CMD_LENGTH] = {0};
    snprintf(send_buf, FS_MAX_CMD_LENGTH, "%c%d;", CMD_STREAM, 2);
    size_t send_len = fs_safe_strnlen(send_buf, FS_MAX_CMD_LENGTH);
    int transmit_timeout_micros = 10000;
    return fs_transmit_networked_tcp(device_handle, &send_buf, send_len, transmit_timeout_micros);
}

/// @brief Deallocate all allocated network resources.
/// @param device_handle Handle to the network device.
/// @return Status of the network shutdown (0 for success, -1 for failure).
int fs_network_exit(fs_device_info_t *device_handle)
{
    char send_buf[FS_MAX_CMD_LENGTH] = {0};
    device_handle->status = FS_RUN_STATUS_IDLE;

    // Prepare the command to stop streaming
    snprintf(send_buf, FS_MAX_CMD_LENGTH, "%c%d;", CMD_STREAM, 0);
    size_t send_len = fs_safe_strnlen(send_buf, FS_MAX_CMD_LENGTH);
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
int fs_network_device_restart(fs_device_info_t *device_handle)
{
    char send_buf[FS_MAX_CMD_LENGTH] = {0};
    snprintf(send_buf, FS_MAX_CMD_LENGTH, ";%c%d;", CMD_RESTART, 0);
    size_t send_len = fs_safe_strnlen(send_buf, FS_MAX_CMD_LENGTH) + 1;
    const int receive_timeout_micros = 10000;
    return (fs_transmit_networked_tcp(device_handle, send_buf, send_len, receive_timeout_micros) > 0) ? 0 : -1;
}

/// @brief
/// @param device_handle
/// @return
int fs_network_set_device_operating_mode(fs_device_info_t *device_handle, fs_communication_mode_t mode)
{
    char send_buf[FS_MAX_CMD_LENGTH] = {0};
    snprintf(send_buf, FS_MAX_CMD_LENGTH, ";%c%d;", CMD_IDENTIFY_PARAM_TRANSMIT, mode);
    size_t send_len = fs_safe_strnlen(send_buf, FS_MAX_CMD_LENGTH) + 1;
    const int receive_timeout_micros = 10000;
    return (fs_transmit_networked_tcp(device_handle, send_buf, send_len, receive_timeout_micros) > 0) ? 0 : -1;
}

/// @brief
/// @param device_handle The device handle
/// @param ssid Null-terminated string with SSID
/// @param password Null-terminated string with password
/// @return
int fs_network_set_network_params(fs_device_info_t *device_handle, char *ssid, char *password)
{
    char send_buf[128] = {0};
    snprintf(send_buf, 128, ";%c%s;%c%s;", CMD_SET_SSID, ssid, CMD_SET_PASSWORD, password);
    size_t send_len = fs_safe_strnlen(send_buf, FS_MAX_CMD_LENGTH) + 1;
    const int receive_timeout_micros = 10000;
    return (fs_transmit_networked_tcp(device_handle, send_buf, send_len, receive_timeout_micros) > 0) ? 0 : -1;
}

/// @brief 
/// @param device_handle 
/// @return 
int fs_network_set_host_udp_port(fs_device_info_t *device_handle, int udp_port)
{
    char send_buf[128] = {0};
    if (udp_port < 1024 || udp_port > 65535)
    {
        fs_log_output("[Trifecta] %d is an invalid UDP port!", udp_port);
        return -1;
    }
    snprintf(send_buf, 128, ";%c%d;", CMD_SET_LISTENING_PORT, udp_port);
    size_t send_len = fs_safe_strnlen(send_buf, FS_MAX_CMD_LENGTH) + 1;
    const int receive_timeout_micros = 10000;
    return (fs_transmit_networked_tcp(device_handle, send_buf, send_len, receive_timeout_micros) > 0) ? 0 : -1;
}
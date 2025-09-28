/// Driver for the Trifecta series of IMU/AHRS/INS devices
/// Copyright 2025 4rge.ai and/or Triangle Man LLC
/// Usage and redistribution of this code is permitted
/// but this notice must be retained in all copies of the code.

/// THIS SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
/// INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE,
/// AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
/// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#ifndef TRIFECTA_DEFS_I_H
#define TRIFECTA_DEFS_I_H

#if defined(ESP_PLATFORM)
#define FS_DRIVER_CONFIG_DEFAULT {      \
    .use_serial_interrupt_mode = false, \
    .background_task_priority = 6,      \
    .background_task_core_affinity = 1, \
    .read_timeout_micros = 1000,        \
    .task_wait_ms = 3,                  \
    .task_stack_size_bytes = 4096,      \
}
#elif defined(__linux__) || defined(_WIN32)
#define FS_DRIVER_CONFIG_DEFAULT {       \
    .use_serial_interrupt_mode = false,  \
    .background_task_priority = -1,      \
    .background_task_core_affinity = -1, \
    .read_timeout_micros = 1000,         \
    .task_wait_ms = 5,                   \
    .task_stack_size_bytes = 4096,       \
}
#elif defined(STM32F4) || defined(STM32F7) || defined(STM32H7)
#define FS_DRIVER_CONFIG_DEFAULT {      \
    .use_serial_interrupt_mode = false, \
    .background_task_priority = 6,      \
    .background_task_core_affinity = 1, \
    .read_timeout_micros = 1000,        \
    .task_wait_ms = 5,                  \
    .task_stack_size_bytes = 4096,      \
}
#else
#define FS_DRIVER_CONFIG_DEFAULT {      \
    .use_serial_interrupt_mode = false, \
    .background_task_priority = 6,      \
    .background_task_core_affinity = 1, \
    .read_timeout_micros = 1000,        \
    .task_wait_ms = 3,                  \
    .task_stack_size_bytes = 4096,      \
}
#endif

#define FS_DEVICE_PARAMS_BLANK                                     \
    ((fs_device_params_t){                                         \
        .communication_mode = FS_COMMUNICATION_MODE_UNINITIALIZED, \
        .status = FS_RUN_STATUS_IDLE,                              \
        .all_enabled_interfaces = 0,                               \
        .ip_addr = {0},                                            \
        .ssid = {0},                                               \
        .ssid_ap = {0},                                            \
        .pw_ap = {0},                                              \
        .tcp_port = 8888,                                          \
        .udp_port = 0,                                             \
        .tcp_sock = -1,                                            \
        .udp_sock = -1,                                            \
        .serial_port = -1,                                         \
        .baudrate = 0,                                             \
        .ping = 0,                                                 \
        .hp_timestamp_micros = 0,                                  \
    })

#define FS_DEVICE_DESCRIPTOR_BLANK                \
    ((fs_device_descriptor_t){.device_id = 0,     \
                              .device_name = {0}, \
                              .device_fw = {0},   \
                              .device_desc = {0}, \
                              .device_sn = {0},   \
                              .device_model = {0}})

#define FS_DEVICE_INFO_UNINITIALIZED ((fs_device_info_t){                                              \
    .device_descriptor = FS_DEVICE_DESCRIPTOR_BLANK,                                                   \
    .device_params = FS_DEVICE_PARAMS_BLANK,                                                           \
    .driver_config = FS_DRIVER_CONFIG_DEFAULT,                                                         \
    .last_received_packet = (fs_packet_union_t){{{0}}},                                                \
    .data_buffer = (fs_bytes_ringbuffer_t){.buffer = {0}, .head = 0, .tail = 0, .count = 0},           \
    .packet_buf_queue = (fs_packet_ringbuffer_t){.buffer = {{{0}}}, .head = 0, .tail = 0, .count = 0}, \
    .command_queue = (fs_command_ringbuffer_t){.buffer = {{0}}, .head = 0, .tail = 0, .count = 0}})
#endif
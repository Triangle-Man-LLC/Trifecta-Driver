/// Driver for the Trifecta series of IMU/AHRS/INS devices
/// Copyright 2024 4rge.ai and/or Triangle Man LLC
/// Usage and redistribution of this code is permitted
/// but this notice must be retained in all copies of the code.

/// THIS SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
/// INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE,
/// AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
/// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#ifndef TRIFECTA_DEFS_H
#define TRIFECTA_DEFS_H

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>

#include "FS_Trifecta_Defs_Packets.h"
#include "FS_Trifecta_Defs_Communication.h"
#include "FS_Trifecta_Defs_Ringbuffer.h"

#define FS_PI 3.14159265358979f

#define FS_MAX_NUMBER_DEVICES 16

#define FS_GYRO_SCALER_DPS 500
#define FS_ACCEL_SCALER_Gs 4

#define DEGREES_TO_RADIANS 0.0174533f

#define FS_MAX_DEVICE_NUMBER 1

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum fs_device_id
    {
        FS_DEVICE_ID_UNKNOWN = 0,
        FS_DEVICE_ID_TK = 1,  // Trifecta-K (IMU)
        FS_DEVICE_ID_TM = 2,  // Trifecta-M (GNSS/INS)
        FS_DEVICE_ID_STV = 2, // Super Trifecta
    } fs_device_id_t;

    typedef enum fs_run_status
    {
        FS_RUN_STATUS_ERROR = -1,
        FS_RUN_STATUS_IDLE = 0,
        FS_RUN_STATUS_RUNNING = 1,
    } fs_run_status_t;

    typedef struct fs_driver_config
    {
        bool use_serial_interrupt_mode;    // If TRUE, and the platform supports it, then use serial interrupt mode intead
        int background_task_priority;      // Priority of the background task for obtaining updates from the device (leave as -1 if no preference)
        int background_task_core_affinity; // Core to pin the background task to (leave as -1 if no preference)
        int read_timeout_micros;           // How long to wait (microseconds) to read data
        int task_wait_ms;                  // How long to wait in between updates of the background task
        int task_stack_size_bytes;         // How much to allocate to the background task
    } fs_driver_config_t;

/// TODO:
#define FS_DRIVER_CONFIG_DEFAULT {      \
    .use_serial_interrupt_mode = false, \
    .background_task_priority = 6,      \
    .background_task_core_affinity = 1, \
    .read_timeout_micros = 1000,        \
    .task_wait_ms = 3,                  \
    .task_stack_size_bytes = 4096,      \
}

/// TODO:
#define FS_DRIVER_CONFIG_DEFAULT_ESP32 { \
    .background_task_priority = 6,       \
    .background_task_core_affinity = 1,  \
    .read_timeout_micros = 1000,         \
    .task_wait_ms = 3,                   \
    .task_stack_size_bytes = 4096,       \
}

/// TODO:
#define FS_DRIVER_CONFIG_DEFAULT_LINUX { \
    .background_task_priority = -1,      \
    .background_task_core_affinity = -1, \
    .read_timeout_micros = 1000,         \
    .task_wait_ms = 5,                   \
    .task_stack_size_bytes = 4096,       \
}

/// TODO:
#define FS_DRIVER_CONFIG_DEFAULT_STM32 { \
    .background_task_priority = 6,       \
    .background_task_core_affinity = 1,  \
    .read_timeout_micros = 1000,         \
    .task_wait_ms = 5,                   \
    .task_stack_size_bytes = 4096,       \
}

    FS_RINGBUFFER_DECLARE(fs_packet_union_t, fs_packet_ringbuffer_t, FS_MAX_PACKET_QUEUE_LENGTH);
    FS_RINGBUFFER_DECLARE(fs_command_info_t, fs_command_ringbuffer_t, FS_MAX_CMD_QUEUE_LENGTH);
    FS_RINGBUFFER_DECLARE(uint8_t, fs_bytes_ringbuffer_t, FS_MAX_DATA_LENGTH);

    /// @brief Device information container
    typedef struct fs_device_info
    {
        char device_name[32];                       // Unique identifier for device name
        fs_device_id_t device_id;                   // Unique identifier for the device type
        fs_communication_mode_t communication_mode; // Communication mode
        int32_t ping;                               // Time since last received communication from device

        fs_driver_config_t driver_config; // Device driver configuration (each device has its own thread spawned unless interrupt mode is active)
        uint64_t hp_timestamp_micros;     // If serial interrupt mode is enabled, this enables accurate timestamping of most recent packet.

        fs_run_status_t status; // 0 = UNINITIALIZED/STOPPED, 1 = RUNNING, -1 = ERROR
        char ip_addr[39];       // If networking is used, this is the corresponding IP address string
        int ip_port;            // Always 8888
        int tcp_sock;           // TCP socket number
        int udp_sock;           // UDP socket number

        int serial_port;                         // If serial communication of some kind is used, this is the corresponding port
        int32_t baudrate;                        // If serial communication is used, baud rate. NOTE: USB-CDC ignores this completely
        fs_bytes_ringbuffer_t data_buffer;       // Received data of the device, used primarily for serial reads
        fs_packet_ringbuffer_t packet_buf_queue; // Packet queue buffer for the device (read-only)
        fs_command_ringbuffer_t command_queue;   // Command buffer for the device (read-only)
    } fs_device_info_t;

#define FS_DEVICE_INFO_UNINITIALIZED {                                                        \
    .device_name = {0},                                        /* device_name */              \
    .device_id = 0,                                            /* device_id */                \
    .communication_mode = FS_COMMUNICATION_MODE_UNINITIALIZED, /* communication_mode */       \
    .ping = 9999,                                              /* ping */                     \
    .driver_config = FS_DRIVER_CONFIG_DEFAULT,                 /* driver_config */            \
    .hp_timestamp_micros = 0,                                  /* High precision timestamp */ \
    .status = FS_RUN_STATUS_IDLE,                              /* status */                   \
    .ip_addr = {0},                                            /* ip_addr */                  \
    .ip_port = 8888,                                           /* ip_port */                  \
    .tcp_sock = -1,                                            /* tcp_sock */                 \
    .udp_sock = -1,                                            /* udp_sock */                 \
    .serial_port = -1,                                         /* serial_port */              \
    .baudrate = 0,                                             /* baudrate */                 \
    .data_buffer = {                                           /* data_buffer */              \
                    .buffer = {0},                                                            \
                    .head = 0,                                                                \
                    .tail = 0,                                                                \
                    .count = 0},                                                              \
    .packet_buf_queue = {/* packet_buf_queue */                                               \
                         .buffer = {0},                                                       \
                         .head = 0,                                                           \
                         .tail = 0,                                                           \
                         .count = 0},                                                         \
    .command_queue = {/* command_queue */                                                     \
                      .buffer = {0},                                                          \
                      .head = 0,                                                              \
                      .tail = 0,                                                              \
                      .count = 0}}

#ifdef __cplusplus
}
#endif

#endif
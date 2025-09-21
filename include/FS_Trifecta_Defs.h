/// Driver for the Trifecta series of IMU/AHRS/INS devices
/// Copyright 2025 4rge.ai and/or Triangle Man LLC
/// Copyright 2025 4rge.ai and/or Triangle Man LLC
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
#include <stdlib.h>
#include <stdbool.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>

#if defined(_WIN32) || defined(_WIN64)
#include <BaseTsd.h>
typedef SSIZE_T ssize_t;
#include <BaseTsd.h>
typedef SSIZE_T ssize_t;

#ifdef FS_DRIVER_EXPORTS
#define FS_API __declspec(dllexport)
#else
#define FS_API __declspec(dllimport)
#endif
#ifdef FS_DRIVER_EXPORTS
#define FS_API __declspec(dllexport)
#else
#define FS_API __declspec(dllimport)
#endif

#elif defined(__GNUC__)
#define FS_API __attribute__((visibility("default")))
#define FS_API __attribute__((visibility("default")))

#else
#define FS_API
#define FS_API
#endif

#include "FS_Trifecta_Defs_Packets.h"
#include "FS_Trifecta_Defs_Communication.h"
#include "FS_Trifecta_Defs_Ringbuffer.h"
#include "FS_Trifecta_Defs_Initializers.h"
#include "FS_Trifecta_Defs_Initializers.h"

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

    typedef struct fs_device_params
    {
        int all_enabled_interfaces; // OR flag of fs_communication_mode_t defining all enabled interfaces, if the device has more than one interface available.
        char ip_addr[39];           // If networking is used, this is the corresponding IP address string
        char ssid[32];
        char ssid_ap[32];
        char pw_ap[64];
        int tcp_port;     // Always 8888
        int udp_port;     // Can be set by user
        int tcp_sock;     // TCP socket number
        int udp_sock;     // UDP socket number
        int serial_port;  // If serial communication of some kind is used, this is the corresponding port
        int32_t baudrate; // If serial communication is used, baud rate. NOTE: USB-CDC ignores this completely
    } fs_device_params_t;

    FS_RINGBUFFER_DECLARE(fs_packet_union_t, fs_packet_ringbuffer_t, FS_MAX_PACKET_QUEUE_LENGTH);
    FS_RINGBUFFER_DECLARE(fs_command_info_t, fs_command_ringbuffer_t, FS_MAX_CMD_QUEUE_LENGTH);
    FS_RINGBUFFER_DECLARE(uint8_t, fs_bytes_ringbuffer_t, FS_MAX_DATA_LENGTH);

    /// @brief Device information container
    typedef struct fs_device_info
    {
        char device_name[32];  //
        char device_fw[32];    //
        char device_desc[64];  //
        char device_sn[32];    //
        char device_model[32]; //

        fs_device_id_t device_id;                   // Unique identifier for the device type
        fs_communication_mode_t communication_mode; // Selected communication mode (how this driver is interfacing with the device)
        fs_run_status_t status;                     // 0 = UNINITIALIZED/STOPPED, 1 = RUNNING, -1 = ERROR
        int32_t ping;                               // Time since last received communication from device

        fs_device_params_t device_params; // Parameters, such as serial baudrate, Wi-Fi SSID, etc.
        fs_driver_config_t driver_config; // Device driver configuration (each device has its own thread spawned unless interrupt mode is active)
        uint64_t hp_timestamp_micros;     // If serial interrupt mode is enabled, this enables accurate timestamping of most recent packet.

        fs_packet_union_t last_received_packet; // The most recent packet from the device

        fs_bytes_ringbuffer_t data_buffer;       // Received data of the device, used primarily for serial reads
        fs_packet_ringbuffer_t packet_buf_queue; // Packet queue buffer for the device (read-only)
        fs_command_ringbuffer_t command_queue;   // Command buffer for the device (read-only)
    } fs_device_info_t;

    static inline size_t fs_safe_strnlen(const char *s, size_t maxlen)
    {
        size_t i = 0;
        while (i < maxlen && s[i] != '\0')
            ++i;
        return i;
    }

    static inline void fs_safe_strncpy(char *dest, const char *src, size_t maxlen)
    {
        if (maxlen == 0 || dest == NULL || src == NULL)
            return;

        size_t i = 0;
        while (i < maxlen - 1 && src[i] != '\0')
        {
            dest[i] = src[i];
            ++i;
        }
        dest[i] = '\0';
    }

#ifdef __cplusplus
}
#endif

#endif
/// Driver for the Trifecta series of IMU/AHRS/INS devices
/// Copyright 2024 4rge.ai and/or Triangle Man LLC
/// Usage and redistribution of this code is permitted
/// but this notice must be retained in all copies of the code.

/// THIS SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
/// INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE,
/// AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
/// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

// #include "FreeRTOS.h" // FreeRTOS headers
// #include "task.h"
// #include "queue.h"
// #include "semphr.h"
// #include "timers.h"
#include "cmsis_os.h"

// Include the HAL library header based on the target STM32 series
#if defined(STM32F0xx)
#include "stm32f0xx_hal.h"
#elif defined(STM32F1xx)
#include "stm32f1xx_hal.h"
#elif defined(STM32F2xx)
#include "stm32f2xx_hal.h"
#elif defined(STM32F3xx)
#include "stm32f3xx_hal.h"
#elif defined(STM32F4xx)
#include "stm32f4xx_hal.h"
#elif defined(STM32F7xx)
#include "stm32f7xx_hal.h"
#elif defined(STM32H7xx)
#include "stm32h7xx_hal.h"
#elif defined(STM32L0xx)
#include "stm32l0xx_hal.h"
#elif defined(STM32L1xx)
#include "stm32l1xx_hal.h"
#elif defined(STM32L4xx)
#include "stm32l4xx_hal.h"
#elif defined(STM32L5xx)
#include "stm32l5xx_hal.h"
#else
#error "Unsupported STM32 series"
#endif

#include "FS_Trifecta_Interfaces.h"

// Platform-specific: Functions for initializing communication drivers on target platform

int fs_logging_level = 0; // Logging level - 0 = OFF, 1 = ON

/// @brief Start the network TCP driver.
/// @param device_handle Pointer to the device information structure
/// @return 0 on success, -1 on failure
int fs_init_network_tcp_driver(fs_device_info *device_handle)
{
    fs_log_output("[Trifecta] Error: STM32 does not support Wifi!\n");
    return -1;
}

/// @brief Start the network UDP driver.
/// @param device_handle Pointer to the device information structure
/// @return 0 on success, -1 on failure
int fs_init_network_udp_driver(fs_device_info *device_handle)
{
    fs_log_output("[Trifecta] Error: STM32 does not support Wifi!\n");
    return -1;
}

/// @brief Start the network serial driver.
/// @param device_handle
/// @return
int fs_init_serial_driver(fs_device_info *device_handle)
{
    // On FreeRTOS/microcontroller systems, the serial port is usually a fixed number, so port scanning will not be done
    // Only check to ensure that the serial port was previously set up
    if (device_handle->serial_port < 0)
    {
        fs_log_output("[Trifecta] Serial port number cannot be less than zero!");
        return -1;
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
int fs_thread_start(void(thread_func)(void *), void *params, bool *thread_running_flag, size_t stack_size, int priority, int core_affinity)
{
    if (thread_func == NULL || thread_running_flag == NULL)
    {
        fs_log_output("[Trifecta] Error: Invalid thread function or running flag!\n");
        return -1;
    }

    else if(thread_running_flag != NULL && *thread_running_flag != 0)
    {
        fs_log_output("[Trifecta] Warning: Thread start aborted, thread was already running!\n");
        return 0; // Thread already running
    }

    // Ensure the flag is set to indicate the thread is running
    *thread_running_flag = true;

    osThreadAttr_t thread_attr = {
        .name = "ThreadTask",
        .priority = (osPriority_t) priority,
        .stack_size = stack_size
    };

    if (core_affinity < 0)
    {
        thread_attr.attr_bits = 0;
    }
    else
    {
        fs_log_output("[Trifecta] Warning: STM32 does not support multi-core operation in this manner!\n");
    }

    osThreadId_t task_handle = osThreadNew(thread_func, params, &thread_attr);

    if (task_handle == NULL)
    {
        fs_log_output("[Trifecta] Error: Task creation failed!\n");
        *thread_running_flag = 0;
        return -1;
    }

    return 0;
}

/// @brief Some platforms (e.g. FreeRTOS) require thread exit to be properly handled.
/// This function should implement that behavior.
/// @return Should always return 0...
int fs_thread_exit()
{
    vTaskDelete(NULL);
    return 0;
}

/// @brief Transmit data over a networked TCP connection
/// @param device_handle Pointer to the device information structure
/// @param tx_buffer Pointer to the transmit data buffer
/// @param length_bytes The size of the tx_buffer
/// @param timeout_micros The max amount of time to wait (microseconds)
/// @return -1 if failed, else number of bytes written
ssize_t fs_transmit_networked_tcp(fs_device_info *device_handle, void *tx_buffer, size_t length_bytes, int timeout_micros)
{
    fs_log_output("[Trifecta] Error: STM32 does not support Wifi!\n");
    return -1;
}

/// @brief Transmit data over a networked UDP connection
/// @param device_handle Pointer to the device information structure
/// @param tx_buffer Pointer to the transmit data buffer
/// @param length_bytes The size of the tx_buffer
/// @param timeout_micros The max amount of time to wait (microseconds)
/// @return -1 if failed, else number of bytes written
ssize_t fs_transmit_networked_udp(fs_device_info *device_handle, void *tx_buffer, size_t length_bytes, int timeout_micros)
{
    fs_log_output("[Trifecta] Error: STM32 does not support Wifi!\n");
    return -1;
}

/// @brief Transmit data over serial communication
/// @param device_handle Pointer to the device information structure
/// @param tx_buffer Pointer to the transmit data buffer
/// @param length_bytes The size of the tx_buffer
/// @param timeout_micros The max amount of time to wait (microseconds)
/// @return -1 if failed, else number of bytes written
ssize_t fs_transmit_serial(fs_device_info *device_handle, void *tx_buffer, size_t length_bytes, int timeout_micros)
{
    if (device_handle == NULL)
    {
        fs_log_output("[Trifecta] Error: Device handle is NULL!");
        return -1;
    }

    if (device_handle->communication_mode != FS_COMMUNICATION_MODE_SERIAL)
    {
        fs_log_output("[Trifecta] Error: Invalid communication mode! Expected COMMUNICATION_MODE_SERIAL.");
        return -1;
    }

    if (device_handle->serial_port < 0)
    {
        fs_log_output("[Trifecta] Error: Invalid serial port!");
        return -1;
    }

    if (tx_buffer == NULL)
    {
        fs_log_output("[Trifecta] Error: Transmit buffer is NULL!");
        return -1;
    }

    // Assuming 'device_handle->serial_port' holds the UART handle (e.g., &huart1 for UART1)
    UART_HandleTypeDef *huart = (UART_HandleTypeDef *)device_handle->serial_port;

    // Transmit data over UART
    HAL_StatusTypeDef status = HAL_UART_Transmit(huart, (uint8_t *)tx_buffer, length_bytes, timeout_micros / 1000);

    if (status != HAL_OK)
    {
        fs_log_output("[Trifecta] Error: Writing data over serial failed!");
        return -1;
    }

    fs_log_output("[Trifecta] Serial transmit to port %d - Length %ld - data %s", (int)huart->Instance, (long)length_bytes, (char *)tx_buffer);

    return (ssize_t)length_bytes;
}

/// @brief Receive data over a networked TCP connection
/// @param device_handle Pointer to the device information structure
/// @param rx_buffer Pointer to the receive data buffer
/// @param length_bytes The max size of the rx_buffer
/// @param timeout_micros The max amount of time to wait (microseconds)
/// @return -1 if failed, else number of bytes received
ssize_t fs_receive_networked_tcp(fs_device_info *device_handle, void *rx_buffer, size_t length_bytes, int timeout_micros)
{
    fs_log_output("[Trifecta] Error: STM32 does not support Wifi!\n");
    return -1;
}

/// @brief Receive data over a networked UDP connection
/// @param device_handle Pointer to the device information structure
/// @param rx_buffer Pointer to the receive data buffer
/// @param length_bytes The max size of the rx_buffer
/// @param timeout_micros The max amount of time to wait (microseconds)
/// @return -1 if failed, else number of bytes received
ssize_t fs_receive_networked_udp(fs_device_info *device_handle, void *rx_buffer, size_t length_bytes, int timeout_micros)
{
    fs_log_output("[Trifecta] Error: STM32 does not support Wifi!\n");
    return -1;
}

/// @brief Receive data over serial communication
/// @param device_handle Pointer to the device information structure
/// @param rx_buffer Pointer to the receive data buffer
/// @param length_bytes The max size of the rx_buffer
/// @param timeout_micros The max amount of time to wait (microseconds)
/// @return -1 if failed, else number of bytes received
ssize_t fs_receive_serial(fs_device_info *device_handle, void *rx_buffer, size_t length_bytes, int timeout_micros)
{
    if (device_handle == NULL)
    {
        fs_log_output("[Trifecta] Error: Device handle is NULL!");
        return -1;
    }

    if (device_handle->communication_mode != FS_COMMUNICATION_MODE_SERIAL)
    {
        fs_log_output("[Trifecta] Error: Invalid communication mode! Expected COMMUNICATION_MODE_SERIAL.");
        return -1;
    }

    if (device_handle->serial_port < 0)
    {
        fs_log_output("[Trifecta] Error: Invalid serial port!");
        return -1;
    }

    if (rx_buffer == NULL)
    {
        fs_log_output("[Trifecta] Error: Receive buffer is NULL!");
        return -1;
    }

    // Assuming 'device_handle->serial_port' holds the UART handle (e.g., &huart1 for UART1)
    UART_HandleTypeDef *huart = (UART_HandleTypeDef *)device_handle->serial_port;

    // Receive data over UART
    HAL_StatusTypeDef status = HAL_UART_Receive(huart, (uint8_t *)rx_buffer, length_bytes, timeout_micros / 1000);

    if (status == HAL_OK)
    {
        ssize_t rx_len = length_bytes; // Number of bytes received
        fs_log_output("[Trifecta] Read data from port %d - length %ld!", (int)huart->Instance, (long)rx_len);
        return rx_len;
    }
    else
    {
        fs_log_output("[Trifecta] Error: Reading data over serial failed!");
        return -1;
    }
}

/// @brief Shutdown the network TCP driver.
/// @param device_handle Pointer to the device information structure.
/// @return 0 if successful, -1 if failed.
int fs_shutdown_network_tcp_driver(fs_device_info *device_handle)
{
    device_handle->tcp_sock = -1;
    return 0;
}

/// @brief Shutdown the network UDP driver.
/// @param device_handle Pointer to the device information structure.
/// @return 0 if successful, -1 if failed.
int fs_shutdown_network_udp_driver(fs_device_info *device_handle)
{
    device_handle->udp_sock = -1;
    return 0;
}

/// @brief Shutdown the serial driver.
/// @param device_handle Pointer to the device information structure.
/// @return 0 if successful, -1 if failed.
int fs_shutdown_serial_driver(fs_device_info *device_handle)
{
    if (device_handle == NULL)
    {
        fs_log_output("[Trifecta] Error: Device handle is NULL!");
        return -1;
    }

    if (device_handle->serial_port < 0)
    {
        fs_log_output("[Trifecta] Error: Invalid serial port!");
        return -1;
    }

    // Assuming 'device_handle->serial_port' holds the UART handle (e.g., &huart1 for UART1)
    UART_HandleTypeDef *huart = (UART_HandleTypeDef *)device_handle->serial_port;

    // Deinitialize the UART
    if (HAL_UART_DeInit(huart) != HAL_OK)
    {
        fs_log_output("[Trifecta] Warning: Failed to deinitialize UART driver (serial port: %d)!", (int)huart->Instance);
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
    osDelay(millis);
    return osKernelSysTick();
}

/// @brief Real-time delay
/// @param current_time Pointer to the current time
/// @param millis The exact amount of time to delay
/// @return The number of ticks the delay lasted
int fs_delay_for(uint32_t *current_time, int millis)
{
    if (*current_time == 0)
    {
        *current_time = osKernelSysTick();
    }
    uint32_t initial_time = *current_time;
    osDelayUntil(current_time, millis);
    uint32_t elapsed_ticks = *current_time - initial_time;
    return (int)elapsed_ticks;
}

/// @brief Get the current system time
/// @param current_time Pointer to the current time
/// @return 0 on success
int fs_get_current_time(uint32_t *current_time)
{
    *current_time = osKernelSysTick();
    return 0;
}
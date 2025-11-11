/// Driver for the Trifecta series of IMU/AHRS/INS devices
/// Copyright 2025 4rge.ai and/or Triangle Man LLC
/// Usage and redistribution of this code is permitted
/// but this notice must be retained in all copies of the code.

/// THIS SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
/// INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE,
/// AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
/// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

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

int fs_init_network_tcp_driver(fs_device_info_t *device_handle)
{
    fs_log_critical("[Trifecta] The STM32F4 platform does not support TCP network connections!");
    return -1;
}

int fs_init_network_udp_driver(fs_device_info_t *device_handle)
{
    fs_log_critical("[Trifecta] The STM32F4 platform does not support UDP network connections!");
    return -1;
}

int fs_init_serial_driver(fs_device_info_t *device_handle)
{
    UART_HandleTypeDef *uart = (UART_HandleTypeDef *)device_handle->device_params.serial_port;

    if (uart == NULL)
    {
        fs_log_critical("[Trifecta] STM32F4 serial handle is NULL!");
        return -1;
    }
    return 0;
}

ssize_t fs_transmit_networked_tcp(fs_device_info_t *device_handle,
                                  void *tx_buffer,
                                  size_t length_bytes,
                                  int timeout_micros)
{
    fs_log_critical("[Trifecta] STM32F4 does not support TCP network transmission!");
    return -1;
}

ssize_t fs_transmit_networked_udp(fs_device_info_t *device_handle,
                                  void *tx_buffer,
                                  size_t length_bytes,
                                  int timeout_micros)
{
    fs_log_critical("[Trifecta] STM32F4 does not support UDP network transmission!");
    return -1;
}

ssize_t fs_transmit_serial(fs_device_info_t *device_handle,
                           void *tx_buffer,
                           size_t length_bytes,
                           int timeout_micros)
{
    UART_HandleTypeDef *uart =
        (UART_HandleTypeDef *)device_handle->device_params.serial_port;

    if (uart == NULL)
    {
        fs_log_error("[Trifecta] Serial transmit failed: UART handle is NULL");
        return -1;
    }

    HAL_StatusTypeDef status =
        HAL_UART_Transmit(uart, (uint8_t *)tx_buffer, length_bytes, timeout_micros / 1000);

    if (status != HAL_OK)
    {
        fs_log_error("[Trifecta] HAL_UART_Transmit failed with status %d", status);
        return -1;
    }

    return (ssize_t)length_bytes;
}

ssize_t fs_receive_networked_tcp(fs_device_info_t *device_handle,
                                 void *rx_buffer,
                                 size_t length_bytes,
                                 int timeout_micros)
{
    fs_log_critical("[Trifecta] STM32F4 does not support TCP network reception!");
    return -1;
}

ssize_t fs_receive_networked_udp(fs_device_info_t *device_handle,
                                 void *rx_buffer,
                                 size_t length_bytes,
                                 int timeout_micros)
{
    fs_log_critical("[Trifecta] STM32F4 does not support UDP network reception!");
    return -1;
}

ssize_t fs_receive_serial(fs_device_info_t *device_handle,
                          void *rx_buffer,
                          size_t length_bytes,
                          int timeout_micros)
{
    UART_HandleTypeDef *uart =
        (UART_HandleTypeDef *)device_handle->device_params.serial_port;

    if (uart == NULL)
    {
        fs_log_error("[Trifecta] Serial receive failed: UART handle is NULL");
        return -1;
    }

    HAL_StatusTypeDef status =
        HAL_UART_Receive(uart, (uint8_t *)rx_buffer, length_bytes, timeout_micros / 1000);

    if (status == HAL_OK)
        return (ssize_t)length_bytes;

    if (status == HAL_TIMEOUT)
        return 0; // No data received within timeout

    fs_log_error("[Trifecta] HAL_UART_Receive failed with status %d", status);
    return -1;
}

///
/// In this example, data from the device is sent over serial.
///

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"

#include "FS_Trifecta.h"

static const char *TAG = "IMU_SERIAL_TEST";

// Prior to starting, please ensure you have set the device up to use UART
// and connected pin as appropriate.
#define TXD_IMU_RX 41 // UART RX signal from IMU - Pin "K1"
#define RXD_IMU_TX 42 // UART TX signal from IMU - Pin "K2"

#define IMU_UART_NUM UART_NUM_1
#define IMU_UART_BUF_SIZE FS_MAX_DATA_LENGTH

/// @brief The IMU device handle
static fs_device_info_t imu_device = FS_DEVICE_INFO_UNINITIALIZED;

// The following config values are recommended for ESP32 devices.
// Note that depending on your application structure, you may need to change the core assignment,
// task priorities, or polling interval.
fs_driver_config_t imu_config = FS_DRIVER_CONFIG_DEFAULT;

/// @brief Configure the UART interface.
/// @return 0 on success
int setup_uart()
{
    uart_config_t uart_config = {
        .baud_rate = FS_TRIFECTA_SERIAL_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    int intr_alloc_flags = 0;
    ESP_ERROR_CHECK(uart_param_config(IMU_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(IMU_UART_NUM, TXD_IMU_RX, RXD_IMU_TX, -1, -1));
    ESP_ERROR_CHECK(uart_driver_install(IMU_UART_NUM, IMU_UART_BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    return 0;
}

/// @brief Initialize the Trifecta IMU device, and wait for connection to succeed.
/// This will loop until the device has been detected.
/// @return 0 on success
int setup_imu()
{
    int status = -1;

    // Wait for connection to complete...
    vTaskDelay(100);
    while (status != 0)
    {
        status = fs_set_driver_parameters(&imu_device, &imu_config);
        if (status != 0)
        {
            ESP_LOGE(TAG, "Could not apply driver parameters!");
        }

        status = fs_initialize_serial(&imu_device, IMU_UART_NUM, FS_COMMUNICATION_MODE_UART);
        ESP_LOGI(TAG, "Waiting for IMU connection!");
        vTaskDelay(1000);
    }

    ESP_LOGI(TAG, "Connected to IMU!");

    return status;
}

/// @brief Demonstrates the setup and read of Trifecta IMU using serial connection.
/// The IMU is made to read the data once. You can do this in a loop if you would like.
/// @return
int app_main()
{
    setup_uart();
    setup_imu();

    // Start stream command turns on the device stream.
    // An alternate way to read IMU data would be to use fs_read_one_shot(),
    // but the stream mode is more suitable for real-time performance.
    fs_start_stream(&imu_device);

    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t delay_time = pdMS_TO_TICKS(5);
    uint32_t last_timestamp = 0;

    /// @brief Quaternion orientation of body relative to earth frame
    static fs_quaternion_t orientation_quaternion = {.w = 1, .x = 0, .y = 0, .z = 0};
    /// @brief Euler orientation of body relative to earth frame
    static fs_vector3_t orientation_euler = {.x = 0, .y = 0, .z = 0};

    int counter = 0;
    while (1)
    {
        // It could be a good idea to set a keepalive signal
        if (counter % 1000 == 0)
        {    
            fs_start_stream(&imu_device);
        }
        counter++;

        if (fs_get_last_timestamp(&imu_device, &last_timestamp) != 0)
        {
            ESP_LOGE(TAG, "Failed to get packet time stamp!\n");
        }

        // Orientation (quaternion) and orientation (euler) are 2 of the possible outputs that you can get from the IMU.
        // Please look at the functions in FS_Trifecta.h to see which other ones are available.
        if (fs_get_orientation(&imu_device, &orientation_quaternion) != 0)
        {
            ESP_LOGE(TAG, "Did not receive orientation quaternion update for some reason!\n");
        }
        if (fs_get_orientation_euler(&imu_device, &orientation_euler, true) != 0)
        {
            ESP_LOGE(TAG, "Did not receive orientation euler update for some reason!\n");
        }

        // Log the data.
        // NOTE: Logging is fairly CPU intensive, and you should consider turning it off in any case except debugging.
        ESP_LOGI(TAG, "Timestamp (%lu)\nQuaternion (%.6f, %.6f, %.6f, %.6f)\nEuler (%.2f, %.2f, %.2f)\n", last_timestamp,
                 orientation_quaternion.w, orientation_quaternion.x, orientation_quaternion.y,
                 orientation_quaternion.z, orientation_euler.x, orientation_euler.y, orientation_euler.z);
        
        vTaskDelayUntil(&last_wake_time, delay_time);
    }
    return 0;
}
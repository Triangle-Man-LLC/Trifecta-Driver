/// Driver for the Trifecta series of IMU/AHRS/INS devices
/// Copyright 2024 4rge.ai and/or Triangle Man LLC
/// Usage and redistribution of this code is permitted
/// but this notice must be retained in all copies of the code.

/// THIS SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
/// INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE,
/// AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
/// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "FS_Trifecta_Interfaces.h"

// Platform-specific: Functions for initializing communication drivers on target platform

int fs_logging_level = 0; // Logging level - 0 = OFF, 1 = ON

/// @brief Platform-specific start thread given a function handle.
/// @param thread_func Pointer to the thread function handle.
/// @param params Parameters to pass to the thread function.
/// @param thread_running_flag Pointer to the flag used to indicate thread status.
/// @param thread_handle_ptr Pointer to the task handle.
/// @param stack_size Size of the stack allocated for the thread.
/// @param priority Priority level of the thread.
/// @param core_affinity -1 for indifference, else preferred core number
/// @return Status of the thread creation (0 for success, -1 for failure).
int fs_thread_start(void(thread_func)(void *), void *params, fs_run_status_t *thread_running_flag, fs_thread_t *thread_handle, size_t stack_size, int priority, int core_affinity)
{
    if (thread_func == NULL || thread_running_flag == NULL)
    {
        fs_log_critical("[Trifecta] Error: Invalid thread function or running flag!\n");
        return -1;
    }

    if (thread_handle == NULL)
    {
        fs_log_output("[Trifecta] Warning: No task handle provided! This could interrupt certain functions.\n");
    }

    BaseType_t result;

    // Create the FreeRTOS task
    if (core_affinity < 0)
    {
        result = xTaskCreate(thread_func, "ThreadTask", stack_size, params, priority, &(thread_handle->handle));
    }
    else
    {
        result = xTaskCreatePinnedToCore(thread_func, "ThreadTask", stack_size, params, priority, &(thread_handle->handle), core_affinity);
    }

    if (result != pdPASS)
    {
        fs_log_output("[Trifecta] Error: Task creation failed!\n");
        *thread_running_flag = FS_RUN_STATUS_ERROR;
        return -1;
    }
    // Ensure the flag is set to indicate the thread is running
    *thread_running_flag = FS_RUN_STATUS_RUNNING;

    return 0;
}

/// @brief Some platforms (e.g. FreeRTOS) require thread exit to be properly handled.
/// This function should implement that behavior.
/// @return Should always return 0...
int fs_thread_exit(void *thread_handle)
{
    if (thread_handle == NULL)
    {
        vTaskDelete(NULL);
        return 0;
    }
    vTaskDelete(*(TaskHandle_t *)thread_handle);
    return 0; // Success
}

/// @brief Logs output with formatting.
/// @param format Format string.
/// @param ... Additional arguments.
/// @return Number of characters printed.
int fs_log_output(const char *format, ...)
{
    int chars_printed = 0;

    if (fs_logging_level <= 0)
    {
        return 0;
    }

    va_list args;
    va_start(args, format);

    // Print formatted string
    chars_printed = vprintf(format, args);

    // Flush stdout to ensure output is available
    fflush(stdout);

    // If last char wasn't newline, add one
    if (chars_printed > 0) {
        // Use fputc instead of indexing format
        if (ferror(stdout) == 0) {
            // Can't directly check last char printed, so safer approach:
            // Always append newline unless format already ends with '\n'
            size_t len = strlen(format);
            if (len == 0 || format[len - 1] != '\n') {
                putchar('\n');
                chars_printed++;
            }
        }
    }

    va_end(args);
    return chars_printed;
}

int fs_log_critical(const char *format, ...)
{
    int chars_printed = 0;

    va_list args;
    va_start(args, format);

    // Print formatted string
    chars_printed = vprintf(format, args);

    // Flush stdout to ensure output is available
    fflush(stdout);

    // If last char wasn't newline, add one
    if (chars_printed > 0) {
        // Use fputc instead of indexing format
        if (ferror(stdout) == 0) {
            // Can't directly check last char printed, so safer approach:
            // Always append newline unless format already ends with '\n'
            size_t len = strlen(format);
            if (len == 0 || format[len - 1] != '\n') {
                putchar('\n');
                chars_printed++;
            }
        }
    }

    va_end(args);
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
    vTaskDelay(pdMS_TO_TICKS(millis));
    return pdMS_TO_TICKS(millis);
}

/// @brief Real-time delay
/// @param current_time Pointer to the current time
/// @param millis The exact amount of time to delay
/// @return The number of ticks the delay lasted
int fs_delay_for(uint32_t *current_time, int millis)
{
    if (*current_time == 0)
    {
        *current_time = xTaskGetTickCount();
    }
    uint32_t initial_time = *current_time;
    vTaskDelayUntil(current_time, pdMS_TO_TICKS(millis));
    uint32_t elapsed_ticks = *current_time - initial_time;
    return (int)elapsed_ticks;
}

/// @brief Get the current system time
/// @param current_time Pointer to the current time
/// @param millis The exact amount of time to delay
/// @return 0 on success
int fs_get_current_time(uint32_t *current_time)
{
    *current_time = xTaskGetTickCount();
    return 0;
}
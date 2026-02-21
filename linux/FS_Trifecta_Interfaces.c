/// Generic driver for the Trifecta series of IMU/AHRS/INS devices.
/// Copyright 2024 4rge.ai and/or Triangle Man LLC
/// Usage and redistribution of this code is permitted
/// but this notice must be retained in all copies of the code.

/// THIS SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
/// INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE,
/// AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
/// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#define _GNU_SOURCE /* See feature_test_macros(7) */

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>

#include <pthread.h>
#include <sched.h>

#include <string.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/select.h>
#include <sys/ioctl.h>

#include <fcntl.h>
#include <termios.h>
#include <glob.h>

#include "FS_Trifecta_Interfaces.h"

int fs_logging_level = 1; // Logging level - 0 = OFF, 1 = ON

/// @brief Platform-specific start thread given a function handle.
/// @param thread_func Pointer to the thread function handle.
/// @param params Parameters to pass to the thread function.
/// @param thread_running_flag Pointer to the flag used to indicate thread status.
/// @param stack_size Size of the stack allocated for the thread.
/// @param priority Priority level of the thread.
/// @param core_affinity -1 for indifference, else preferred core number
/// @return Status of the thread creation (0 for success, -1 for failure).
int fs_thread_start(fs_thread_func_t (*thread_func)(void *), void *params, fs_run_status_t *thread_running_flag, fs_thread_t *thread_handle, size_t stack_size, int priority, int core_affinity)
{
    if (thread_func == NULL || thread_running_flag == NULL)
    {
        fs_log_output("[Trifecta] Error: Invalid thread function or running flag!\n");
        return -1;
    }

    pthread_t thread;
    pthread_attr_t attr;
    pthread_attr_init(&attr);

    // Apply system defaults for parameters if their values are < 0
    if (stack_size == 0)
    {
        stack_size = PTHREAD_STACK_MIN; // Default stack size, platform-defined minimum
    }
    if (priority < 0)
    {
        priority = sched_get_priority_min(SCHED_OTHER); // Default priority, lowest valid
    }
    if (core_affinity < 0)
    {
        core_affinity = -1; // Indifferent to core affinity
    }

    // Set stack size
    if (pthread_attr_setstacksize(&attr, stack_size) != 0)
    {
        fs_log_output("[Trifecta] Warning: Failed to set stack size!\n");
    }

    // Set thread priority if supported
    struct sched_param param;
    param.sched_priority = priority;
    if (pthread_attr_setschedparam(&attr, &param) != 0)
    {
        fs_log_output("[Trifecta] Warning: Failed to set thread priority!\n");
    }

    // Set CPU core affinity if specified and supported
    if (core_affinity >= 0)
    {
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(core_affinity, &cpuset);
        int affinity_result = pthread_attr_setaffinity_np(&attr, sizeof(cpu_set_t), &cpuset);
        if (affinity_result != 0)
        {
            fs_log_output("[Trifecta] Warning: Failed to set thread affinity!\n");
        }
    }

    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

    // Create the thread
    *thread_running_flag = FS_RUN_STATUS_RUNNING;
    int result = pthread_create(&thread, &attr, thread_func, params);
    pthread_attr_destroy(&attr);

    if (result != 0)
    {
        fs_log_output("[Trifecta] Error: Thread creation failed: errno %d!\n", errno);
        *thread_running_flag = FS_RUN_STATUS_ERROR;
        return -1;
    }

    // Populate the thread handle
    if (thread_handle != NULL)
    {
        thread_handle->handle = thread;
    }

    fs_log_output("[Trifecta] Thread created successfully.\n");
    return 0;
}

/// @brief Some platforms (e.g. FreeRTOS) require thread exit to be properly handled.
/// This function should implement that behavior.
/// @param thread_handle On Linux systems, this has no impact.
/// @return Should always return 0...
int fs_thread_exit(void *thread_handle)
{
    pthread_exit(NULL);
    return 0;
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
    if (chars_printed > 0)
    {
        // Use fputc instead of indexing format
        if (ferror(stdout) == 0)
        {
            // Can't directly check last char printed, so safer approach:
            // Always append newline unless format already ends with '\n'
            size_t len = strlen(format);
            if (len == 0 || format[len - 1] != '\n')
            {
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
    if (chars_printed > 0)
    {
        // Use fputc instead of indexing format
        if (ferror(stdout) == 0)
        {
            // Can't directly check last char printed, so safer approach:
            // Always append newline unless format already ends with '\n'
            size_t len = strlen(format);
            if (len == 0 || format[len - 1] != '\n')
            {
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <errno.h>
#include <limits.h>

/// @brief Redirect logs to the indicated path.
/// Only some platforms support this. (E.g. a filesystem needed.)
/// @param directory The path to store logs into.
/// @return 0 on success, or a negative error code on failure.
int fs_set_log_location(const char *directory)
{
    if (!directory)
    {
        return -EINVAL; // invalid argument
    }

    // Check that directory exists and is writable
    struct stat st;
    if (stat(directory, &st) != 0)
    {
        return -errno; // directory not found
    }
    if (!S_ISDIR(st.st_mode))
    {
        return -ENOTDIR; // not a directory
    }
    if (access(directory, W_OK) != 0)
    {
        return -EACCES; // not writable
    }

    // Construct log file path
    char path[PATH_MAX];
    snprintf(path, sizeof(path), "%s/fs_trifecta_log.txt", directory);

    // Open log file for write (truncate existing)
    FILE *logf = fopen(path, "w");
    if (!logf)
    {
        return -errno;
    }

    // Redirect stdout and stderr
    if (dup2(fileno(logf), STDOUT_FILENO) < 0)
    {
        fclose(logf);
        return -errno;
    }
    if (dup2(fileno(logf), STDERR_FILENO) < 0)
    {
        fclose(logf);
        return -errno;
    }

    // Do not fclose(logf) here, because stdout/stderr now share the fd.
    return 0;
}

/// @brief Delay by at least this amount of time
/// @param millis Number of milliseconds to delay
/// @return The number of ticks the delay lasted
int fs_delay(int millis)
{
    struct timespec req, rem;
    req.tv_sec = millis / 1000;
    req.tv_nsec = (millis % 1000) * 1000000;

    while (nanosleep(&req, &rem) == -1 && errno == EINTR)
    {
        req = rem;
    }

    return millis;
}

/// @brief Real-time delay
/// @param current_time Pointer to the current time
/// @param millis The exact amount of time to delay
/// @return The number of ticks the delay lasted
int fs_delay_for(uint32_t *current_time, int millis)
{
    if (current_time == NULL)
    {
        return -1;
    }

    struct timespec start, end;
    clock_gettime(CLOCK_MONOTONIC, &start);

    fs_delay(millis);

    clock_gettime(CLOCK_MONOTONIC, &end);

    uint32_t elapsed_ms = (end.tv_sec - start.tv_sec) * 1000 + (end.tv_nsec - start.tv_nsec) / 1000000;

    *current_time += elapsed_ms;
    return elapsed_ms;
}

/// @brief Get the current system time
/// @param current_time Pointer to the current time
/// @return 0 on success
int fs_get_current_time(uint32_t *current_time)
{
    struct timespec ts;
    if (clock_gettime(CLOCK_REALTIME, &ts) == -1)
    {
        return -1;
    }
    *current_time = ts.tv_sec * 1000 + ts.tv_nsec / 1000000; // Convert to milliseconds
    return 0;
}

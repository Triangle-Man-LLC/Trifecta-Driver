/// Generic driver for the Trifecta series of IMU/AHRS/INS devices.
/// Copyright 2025 4rge.ai and/or Triangle Man LLC
/// Usage and redistribution of this code is permitted
/// but this notice must be retained in all copies of the code.

/// THIS SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
/// INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE,
/// AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
/// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <tchar.h>
#include <time.h>
#include <string.h>

#define WIN32_LEAN_AND_MEAN
#define _WINSOCKAPI_ // Prevent inclusion of winsock.h by windows.h

#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <process.h>
#include <io.h>
#include <fcntl.h>
#include <tchar.h>
#include <conio.h>

#include "FS_Trifecta_Interfaces.h"

// Platform-specific: Functions for initializing communication drivers on target platform
#pragma comment(lib, "ws2_32.lib")
#define FS_TRIFECTA_SERIAL_BAUDRATE_WINDOWS 2000000

int fs_logging_level = 0; // Logging level - 0 = OFF, 1 = ON
static FILE *log_file_ptr = NULL;

/// @brief Platform-specific start thread given a function handle.
/// @param thread_func Pointer to the thread function handle.
/// @param params Parameters to pass to the thread function.
/// @param thread_running_flag Pointer to the flag used to indicate thread status.
/// @param stack_size Size of the stack allocated for the thread.
/// @param priority Priority level of the thread.
/// @param core_affinity -1 for indifference, else preferred core number
/// @return Status of the thread creation (0 for success, -1 for failure).
int fs_thread_start(void (*thread_func)(void *), void *params, fs_run_status_t *thread_running_flag, size_t stack_size, int priority, int core_affinity)
{
    if (thread_func == NULL || thread_running_flag == NULL)
    {
        fs_log_output("[Trifecta-Interface] Error: Invalid thread function or running flag!\n");
        return -1;
    }

    *thread_running_flag = FS_RUN_STATUS_RUNNING;

    stack_size = 0; // Override to use default stack size on Windows

    HANDLE thread_handle = (HANDLE)_beginthreadex(
        NULL, // Security attributes
        (unsigned)stack_size,
        (unsigned(__stdcall *)(void *))thread_func,
        params,
        0,   // Run immediately
        NULL // Optionally capture thread ID
    );

    if (thread_handle == 0)
    {
        fs_log_output("[Trifecta-Interface] Error: Thread creation failed: errno %d!\n", errno);
        *thread_running_flag = FS_RUN_STATUS_ERROR;
        return -1;
    }

    // Set thread priority - EDIT: Disabled on Windows
    // if (priority >= 0)
    // {
    //     if (!SetThreadPriority(thread_handle, priority))
    //     {
    //         fs_log_output("[Trifecta-Interface] Warning: Failed to set thread priority: %lu\n", GetLastError());
    //     }
    // }

    // Set core affinity - EDIT: Disabled on Windows
    // if (core_affinity >= 0)
    // {
    //     DWORD_PTR affinity_mask = 1ULL << core_affinity;
    //     if (SetThreadAffinityMask(thread_handle, affinity_mask) == 0)
    //     {
    //         fs_log_output("[Trifecta-Interface] Warning: Failed to set thread affinity: %lu\n", GetLastError());
    //     }
    // }

    CloseHandle(thread_handle); // Detach thread

    fs_log_output("[Trifecta-Interface] Thread created successfully.\n");
    return 0;
}

/// @brief Some platforms (e.g. FreeRTOS) require thread exit to be properly handled.
/// This function should implement that behavior.
/// @param thread_handle On Linux systems, this has no impact.
/// @return Should always return 0...
int fs_thread_exit(void *thread_handle)
{
    ExitThread(0);
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

/// @brief Redirect logs to the indicated path.
/// Only some platforms support this. (E.g. a filesystem needed.)
/// @param context The path to store logs into.
/// @return 0 on success, or a negative error code on failure.
int fs_set_log_location(const char *path)
{
    if (!log_file_ptr && strnlen(path, 256) > 0 && strnlen(path, 256) <= 255)
    {
        log_file_ptr = freopen(path, "w", stdout);
        if (!log_file_ptr)
        {
            fs_log_output("[Trifecta-Interface] Failed to redirect stdout");
            return -1;
        }
    }
    else if (log_file_ptr)
    {
        fclose(log_file_ptr);
        log_file_ptr = NULL;
        // TODO: Maybe restore stdout to console if needed (platform-specific)
    }
    return 0;
}

/// @brief Delay by at least this amount of time
/// @param millis Number of milliseconds to delay
/// @return The number of ticks the delay lasted
int fs_delay(int millis)
{
    Sleep(millis);
    return millis;
}

/// @brief Real-time delay
/// @param current_time Pointer to the current time
/// @param millis The exact amount of time to delay
/// @return The number of ticks the delay lasted
/// @brief Real-time delay (Windows version)
/// @param current_time Pointer to the current time
/// @param millis The exact amount of time to delay
/// @return The number of ticks the delay lasted
int fs_delay_for(uint32_t *current_time, int millis)
{
    if (current_time == NULL)
    {
        return -1;
    }

    LARGE_INTEGER freq, start, end;
    QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&start);

    fs_delay(millis);

    QueryPerformanceCounter(&end);

    uint32_t elapsed_ms = (uint32_t)((end.QuadPart - start.QuadPart) * 1000 / freq.QuadPart);
    *current_time += elapsed_ms;
    return elapsed_ms;
}

/// @brief Get the current system time
/// @param current_time Pointer to the current time
/// @return 0 on success
int fs_get_current_time(uint32_t *current_time)
{
    if (current_time == NULL)
    {
        return -1;
    }

    FILETIME ft;
    GetSystemTimeAsFileTime(&ft);

    // Convert FILETIME (100-ns intervals since Jan 1, 1601) to milliseconds since Unix epoch
    ULARGE_INTEGER time;
    time.LowPart = ft.dwLowDateTime;
    time.HighPart = ft.dwHighDateTime;

    uint64_t ms_since_1601 = time.QuadPart / 10000;
    uint64_t ms_since_1970 = ms_since_1601 - 11644473600000ULL;

    *current_time = (uint32_t)(ms_since_1970 & 0xFFFFFFFF);
    return 0;
}
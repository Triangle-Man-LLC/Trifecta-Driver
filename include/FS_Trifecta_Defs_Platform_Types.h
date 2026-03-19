/// Driver for the Trifecta series of IMU/AHRS/INS devices
/// Copyright 2026 4rge.ai and/or Triangle Man LLC
/// Usage and redistribution of this code is permitted
/// but this notice must be retained in all copies of the code.

/// THIS SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
/// INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE,
/// AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
/// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#ifndef TRIFECTA_DEFS_PLATFORM_TYPES_H
#define TRIFECTA_DEFS_PLATFORM_TYPES_H

// Windows (MSVC or MinGW)
#if defined(_WIN32)

    #if defined(__MINGW32__) || defined(__MINGW64__)
        #include <basetsd.h>   // MinGW version
    #else
        #include <BaseTsd.h>   // MSVC Windows SDK version
    #endif

    // --- ssize_t on Windows ---
    typedef SSIZE_T ssize_t;

    // --- DLL import/export ---
    #ifdef FS_DRIVER_EXPORTS
        #define FS_API __declspec(dllexport)
    #else
        #define FS_API __declspec(dllimport)
    #endif

// GCC/Clang (Linux, Android, etc.)
#elif defined(__GNUC__)
    #define FS_API __attribute__((visibility("default")))
    
#else
    #define FS_API
#endif

typedef intptr_t fs_sock_t;
typedef intptr_t fs_serial_handle_t;

#if defined(__linux__)
typedef void *fs_thread_func_t;
#define FS_THREAD_RETVAL NULL
#else
typedef void fs_thread_func_t;
#define FS_THREAD_RETVAL
#endif

#if defined(ESP_PLATFORM) || defined(STM32_FREERTOS)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#elif defined(_WIN32)
#include <windows.h>
#else
#include <pthread.h>
#endif

typedef struct fs_thread
{
#if defined(ESP_PLATFORM)
    TaskHandle_t handle;
#elif defined(STM32_FREERTOS)
    TaskHandle_t handle;
#elif defined(_WIN32)
    HANDLE handle;
#else
    pthread_t handle;
#endif
} fs_thread_t;

#endif // TRIFECTA_DEFS_PLATFORM_TYPES_H
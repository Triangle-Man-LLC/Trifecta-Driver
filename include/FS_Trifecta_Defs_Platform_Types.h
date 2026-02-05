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

#ifdef _WIN32
typedef intptr_t fs_sock_t;
typedef intptr_t fs_serial_handle_t;
#else
typedef int fs_sock_t;
typedef int fs_serial_handle_t;
#endif

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
    pthread_t *handle;
#endif
} fs_thread_t;

#endif // TRIFECTA_DEFS_PLATFORM_TYPES_H
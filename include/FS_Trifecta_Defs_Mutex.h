#pragma once

#if defined(ESP_PLATFORM)
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#elif defined(_WIN32)
#include <windows.h>
#else // POSIX
#include <pthread.h>
#endif

typedef struct
{
#if defined(ESP_PLATFORM)
    SemaphoreHandle_t handle;
#elif defined(_WIN32)
    CRITICAL_SECTION handle;
#else
    pthread_mutex_t handle;
#endif
} fs_mutex_t;

static inline void fs_mutex_init(fs_mutex_t *mutex)
{
    if (!mutex)
    {
        return; // Prevent NULL dereference
    }
#if defined(ESP_PLATFORM)
    mutex->handle = xSemaphoreCreateMutex();
#elif defined(_WIN32)
    InitializeCriticalSection(&mutex->handle);
#else
    pthread_mutex_init(&mutex->handle, NULL);
#endif
}

static inline void fs_mutex_lock(fs_mutex_t *mutex)
{
#if defined(ESP_PLATFORM)
    if (!mutex)
    {
        return; // Prevent NULL dereference
    }
    xSemaphoreTake(mutex->handle, portMAX_DELAY);
#elif defined(_WIN32)
    if (!mutex)
    {
        return; // Prevent NULL dereference
    }
    EnterCriticalSection(&mutex->handle);
#else
    if (!mutex)
    {
        return; // Prevent NULL dereference
    }
    pthread_mutex_lock(&mutex->handle);
#endif
}

static inline void fs_mutex_unlock(fs_mutex_t *mutex)
{
#if defined(ESP_PLATFORM)
    if (!mutex)
    {
        return; // Prevent NULL dereference
    }
    xSemaphoreGive(mutex->handle);
#elif defined(_WIN32)
    if (!mutex)
    {
        return; // Prevent NULL dereference
    }
    LeaveCriticalSection(&mutex->handle);
#else
    if (!mutex)
    {
        return; // Prevent NULL dereference
    }
    pthread_mutex_unlock(&mutex->handle);
#endif
}

static inline void fs_mutex_destroy(fs_mutex_t *mutex)
{
#if defined(ESP_PLATFORM)
    if (!mutex)
    {
        return; // Prevent NULL dereference
    }
    vSemaphoreDelete(mutex->handle);
#elif defined(_WIN32)
    if (!mutex)
    {
        return; // Prevent NULL dereference
    }
    DeleteCriticalSection(&mutex->handle);
#else
    if (!mutex)
    {
        return; // Prevent NULL dereference
    }
    pthread_mutex_destroy(&mutex->handle);
#endif
}

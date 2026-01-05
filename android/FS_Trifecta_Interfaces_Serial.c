#include <jni.h>
#include <pthread.h>

#include "FS_Trifecta_Interfaces.h"
#include "FS_Trifecta_Device_Utils.h"

static JavaVM *g_vm = NULL;
static jmethodID g_write_method = NULL;

typedef struct
{
    fs_device_info_t *dev_ptr;
    int assigned_index;
} fs_device_serial_port_map_t;

static jobject device_java_serial_obj[FS_MAX_NUMBER_DEVICES] = {NULL}; // Global references to Java serial objects
static fs_device_serial_port_map_t device_port_map[FS_MAX_NUMBER_DEVICES] = {0};

static fs_mutex_t java_mutex = {0};
static int java_mutex_initialized = 0;

static int fs_get_serial_id_for_device(const fs_device_info_t *device)
{
    // Check if already assigned
    for (int i = 0; i < FS_MAX_NUMBER_DEVICES; ++i)
    {
        if (device_port_map[i].dev_ptr == device)
            return device_port_map[i].assigned_index;
    }
    // Find a free slot
    for (int i = 0; i < FS_MAX_NUMBER_DEVICES; ++i)
    {
        if (device_port_map[i].dev_ptr == NULL)
        {
            device_port_map[i].dev_ptr = (fs_device_info_t *)device;
            device_port_map[i].assigned_index = i;
            return i;
        }
    }
    // No free slots
    return -1;
}

static void fs_release_serial_port_for_device(const fs_device_info_t *device)
{
    for (int i = 0; i < FS_MAX_NUMBER_DEVICES; ++i)
    {
        if (device_port_map[i].dev_ptr == device)
        {
            device_port_map[i].dev_ptr = NULL;
            device_port_map[i].assigned_index = -1;
            return;
        }
    }
}

static fs_bytes_ringbuffer_t data_buffers[FS_MAX_NUMBER_DEVICES] = {0};

JNIEXPORT void JNICALL
Java_com_trifecta_SerialBridge_nativeOnBytesReceived(JNIEnv *env,
                                                           jclass clazz,
                                                           jlong devPtr,
                                                           jbyteArray data)
{
    if (devPtr == 0)
        return;

    fs_device_info_t *device_handle = (fs_device_info_t *)devPtr;

    int index = device_handle->device_params.serial_port;
    if (index < 0 || index >= FS_MAX_NUMBER_DEVICES)
        return;

    jsize len = (*env)->GetArrayLength(env, data);
    jbyte *bytes = (*env)->GetByteArrayElements(env, data, NULL);

    fs_mutex_lock(&device_handle->lock);

    fs_bytes_ringbuffer_t *rb = &data_buffers[index];
    fs_cb_push(rb, (const uint8_t *)bytes, (size_t)len);

    fs_mutex_unlock(&device_handle->lock);

    (*env)->ReleaseByteArrayElements(env, data, bytes, JNI_ABORT);
}

JNIEXPORT void JNICALL
Java_com_trifecta_SerialBridge_nativeRegister(JNIEnv *env,
                                                    jclass clazz,
                                                    jlong devPtr,
                                                    jobject serialObj)
{
    fs_device_info_t *device = (fs_device_info_t *)devPtr;

    // Cache JVM once
    if (g_vm == NULL)
        (*env)->GetJavaVM(env, &g_vm);

    // Initialize the mutex once
    if (!java_mutex_initialized)
    {
        fs_mutex_init(&java_mutex);
        java_mutex_initialized = 1;
    }

    // Map device → index
    int index = fs_get_serial_id_for_device(device);
    if (index < 0 || index >= FS_MAX_NUMBER_DEVICES)
        return;

    // Store per-device Java serial object (global ref)
    device_java_serial_obj[index] = (*env)->NewGlobalRef(env, serialObj);

    // Cache writeBytes() method ID once (class is the same for all)
    if (g_write_method == NULL)
    {
        jclass cls = (*env)->GetObjectClass(env, serialObj);
        g_write_method = (*env)->GetMethodID(env, cls, "writeBytes", "([B)I");
    }

    // Init this device's ringbuffer
    FS_RINGBUFFER_INIT(&data_buffers[index]);
}

static uint64_t fs_get_time_us()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000ULL;
}

/// @brief Initializes a serial communication driver for the given device.
/// @param device_handle Pointer to the device information structure.
/// @return 0 on success, or a negative error code on failure.
int fs_init_serial_driver(fs_device_info_t *device_handle)
{
    if (!device_handle)
        return -1;

    device_handle->device_params.serial_port = fs_get_serial_id_for_device(device_handle);
    if (device_handle->device_params.serial_port < 0)
        return -1;

    device_handle->device_params.communication_mode = FS_COMMUNICATION_MODE_USB_CDC;
    return 0;
}

/// @brief Whether interrupt-driven UART etc. is supported by the platform.
/// Many RTOSes support this, but Linux does not, etc.
/// @return OR FLAG of serial interfaces which support interrupts.
/// If the return is FS_COMMUNICATION_MODE_UNINITIALIZED, then no interrupt-driven serial is allowed.
int fs_platform_supported_serial_interrupts()
{
    return FS_COMMUNICATION_MODE_UNINITIALIZED;
}

/// @brief Start serial in interrupt mode on platforms that support it.
/// This enables more precise and low latency serial reads than polling.
/// @param device_handle
/// @param status_flag
/// @return 0 on success, -1 on fail (e.g. not supported on platform)
int fs_init_serial_interrupts(fs_device_info_t *device_handle, fs_run_status_t *status_flag)
{
    return -1;
}

/// @brief Sends data via serial communication to the specified device.
/// @param device_handle Pointer to the device information structure.
/// @param tx_buffer Pointer to the data buffer to transmit.
/// @param length_bytes Number of bytes to transmit.
/// @param timeout_micros Timeout for the operation in microseconds.
/// @return Number of bytes sent on success, or a negative error code on failure.
ssize_t fs_transmit_serial(fs_device_info_t *device_handle,
                           void *tx_buffer,
                           size_t length_bytes,
                           int timeout_micros)
{
    if (!device_handle || !tx_buffer)
        return -1;

    // Map device → index
    int index = fs_get_serial_id_for_device(device_handle);
    if (index < 0 || index >= FS_MAX_NUMBER_DEVICES)
        return -1;

    jobject serialObj = device_java_serial_obj[index];
    if (serialObj == NULL)
        return -1;

    fs_mutex_lock(&java_mutex);

    // Attach thread to JVM
    JNIEnv *env;
    (*g_vm)->AttachCurrentThread(g_vm, &env, NULL);

    // Create Java byte[] and copy data
    jbyteArray arr = (*env)->NewByteArray(env, length_bytes);
    (*env)->SetByteArrayRegion(env, arr, 0, length_bytes, (jbyte *)tx_buffer);

    // Call Java writeBytes(byte[])
    jint written = (*env)->CallIntMethod(env, serialObj, g_write_method, arr);

    // Cleanup
    (*env)->DeleteLocalRef(env, arr);

    fs_mutex_unlock(&java_mutex);

    return written;
}

/// @brief Receives data via serial communication from the specified device.
/// @param device_handle Pointer to the device information structure.
/// @param rx_buffer Pointer to the buffer to store received data.
/// @param length_bytes Number of bytes to receive.
/// @param timeout_micros Timeout for the operation in microseconds.
/// @return Number of bytes received on success, or a negative error code on failure.
ssize_t fs_receive_serial(fs_device_info_t *device_handle,
                          void *rx_buffer,
                          size_t length_bytes,
                          int timeout_micros)
{
    if (!device_handle || !rx_buffer)
        return -1;

    uint8_t *out = (uint8_t *)rx_buffer;

    // Map device → index
    int index = fs_get_serial_id_for_device(device_handle);
    if (index < 0 || index >= FS_MAX_NUMBER_DEVICES)
        return -1;

    fs_bytes_ringbuffer_t *rb = &data_buffers[index];

    size_t received = 0;
    uint64_t start = fs_get_time_us();

    while (received < length_bytes)
    {
        fs_mutex_lock(&java_mutex);

        // Pop as many bytes as available (up to remaining space)
        size_t popped = fs_cb_pop(rb, out + received, length_bytes - received);
        received += popped;

        fs_mutex_unlock(&java_mutex);

        if (received >= length_bytes)
            break;

        // Timeout check
        if ((fs_get_time_us() - start) > timeout_micros)
            break;

        // Avoid busy-waiting
        fs_delay(1);
    }

    return received;
}

/// @brief Shuts down the serial communication driver for the specified device.
/// @param device_handle Pointer to the device information structure.
/// @return 0 on success, or a negative error code on failure.
int fs_shutdown_serial_driver(fs_device_info_t *device_handle)
{
    if (!device_handle)
        return -1;
    device_handle->device_params.communication_mode = FS_COMMUNICATION_MODE_UNINITIALIZED;
    fs_release_serial_port_for_device(device_handle);
    return 0;
}

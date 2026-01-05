
package com.trifecta;

public final class TrifectaAPI {

    static {
        System.loadLibrary("DriverTrifectaNative");
    }

    // Device handle = fs_device_info_t*
    public static final class FsDeviceInfo {
        public long ptr;
        public FsDeviceInfo(long p) { this.ptr = p; }
    }

    // ===== Device allocation and lifecycle =====

    public static native long fs_export_allocate_device();

    public static native void fs_export_free_device(long devicePtr);

    public static native int fs_set_driver_parameters(long devicePtr, FSDriverConfig config);

    public static native int fs_initialize_networked(long devicePtr, String ipAddress);

    public static native int fs_initialize_serial(long devicePtr, int port, int mode /* FSCommunicationMode.value */);

    public static native int fs_closedown(long devicePtr);

    // ===== Stream control =====

    public static native int fs_start_stream(long devicePtr);

    public static native int fs_stop_stream(long devicePtr);

    public static native int fs_read_one_shot(long devicePtr);

    public static native int fs_reboot_device(long devicePtr);

    // ===== Data export / viewing =====

    // out uint → Java uses int[1]
    public static native int fs_get_last_timestamp(long devicePtr, int[] timeOut);

    public static native int fs_get_raw_packet(long devicePtr, FSPacketUnion packetBuffer);

    public static native int fs_get_raw_packet_queue_size(long devicePtr);

    public static native int fs_get_raw_packet_from_queue(long devicePtr, FSPacketUnion packetBuffer, int pos);

    public static native int fs_get_orientation(long devicePtr, FSQuaternion orientationBuffer);

    public static native int fs_get_orientation_euler(long devicePtr, FSVector3 orientationBuffer, boolean degrees);

    public static native int fs_get_acceleration(long devicePtr, FSVector3 accelerationBuffer);

    public static native int fs_get_angular_velocity(long devicePtr, FSVector3 angularVelocityBuffer);

    public static native int fs_get_velocity(long devicePtr, FSVector3 velocityBuffer);

    // out FsRunStatus → Java uses int[1]
    public static native int fs_get_movement_state(long devicePtr, int[] stateOut);

    public static native int fs_get_position(long devicePtr, FSVector3 positionBuffer);

    // ===== Device configuration =====

    public static native int fs_set_ahrs_heading(long devicePtr, float headingDeg);

    public static native int fs_set_ins_position(long devicePtr, FSVector3 position);

    public static native int fs_set_device_name(long devicePtr, String name);

    public static native int fs_set_communication_mode(long devicePtr, int modes);

    public static native int fs_set_network_parameters(long devicePtr, String ssid, String pw, boolean accessPoint);

    public static native int fs_set_network_udp_port(long devicePtr, int port);

    public static native int fs_set_serial_uart_baudrate(long devicePtr, int baudrate);

    public static native int fs_get_device_operating_state(long devicePtr, FSDeviceParams deviceParamsInfo);

    public static native int fs_get_device_descriptors(long devicePtr, FSDeviceDescriptor desc);

    // ===== Debug utilities =====

    public static native int fs_enable_logging(boolean doEnable);

    public static native int fs_enable_logging_at_path(String path, boolean doEnable);

    public static native int fs_factory_reset(long devicePtr);
}

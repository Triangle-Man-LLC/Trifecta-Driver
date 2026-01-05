
package com.trifecta;

// Constants

public final class FSConstants {
    private FSConstants() {}

    public static final float FS_PI = 3.14159265358979f;

    public static final int FS_MAX_NUMBER_DEVICES = 16;

    public static final int FS_GYRO_SCALER_DPS = 500;
    public static final int FS_ACCEL_SCALER_Gs = 4;

    public static final float DEGREES_TO_RADIANS = 0.0174533f;
}


// Device ID enum

public enum FSDeviceId {
    FS_DEVICE_ID_UNKNOWN(0),

    // Trifecta-K (IMU - Generic)
    FS_DEVICE_ID_TK(1),

    // Trifecta-K0/K1/K2
    FS_DEVICE_ID_TK0(10),
    FS_DEVICE_ID_TK1(11),
    FS_DEVICE_ID_TK2(12),

    // Trifecta-M (GNSS/INS)
    FS_DEVICE_ID_TM(2),
    FS_DEVICE_ID_TM0(20),
    FS_DEVICE_ID_TM1(21),
    FS_DEVICE_ID_TM2(22),

    // Super Trifecta
    FS_DEVICE_ID_STV(3),
    FS_DEVICE_ID_STV1(31),
    FS_DEVICE_ID_STV2(32);

    public final int value;

    FSDeviceId(int value) {
        this.value = value;
    }

    public static FSDeviceId fromInt(int v) {
        for (FSDeviceId id : values()) {
            if (id.value == v) return id;
        }
        return FS_DEVICE_ID_UNKNOWN;
    }
}


// Run status enum

public enum FSRunStatus {
    FS_RUN_STATUS_ERROR(-1),
    FS_RUN_STATUS_IDLE(0),
    FS_RUN_STATUS_RUNNING(1);

    public final int value;

    FSRunStatus(int value) {
        this.value = value;
    }

    public static FSRunStatus fromInt(int v) {
        for (FSRunStatus s : values()) {
            if (s.value == v) return s;
        }
        return FS_RUN_STATUS_ERROR;
    }
}


// Driver configuration

public final class FSDriverConfig {
    public boolean useSerialInterruptMode;    // use_serial_interrupt_mode
    public int backgroundTaskPriority;        // background_task_priority
    public int backgroundTaskCoreAffinity;    // background_task_core_affinity
    public int readTimeoutMicros;             // read_timeout_micros
    public int taskWaitMs;                    // task_wait_ms
    public int taskStackSizeBytes;            // task_stack_size_bytes
}


// Communication mode placeholder – should mirror fs_communication_mode_t
public enum FSCommunicationMode {
    // Fill with actual values from FS_Trifecta_Defs_Communication.h
    FS_COMMUNICATION_MODE_UNINITIALIZED(0);

    public final int value;

    FSCommunicationMode(int value) {
        this.value = value;
    }

    public static FSCommunicationMode fromInt(int v) {
        for (FSCommunicationMode m : values()) {
            if (m.value == v) return m;
        }
        return FS_COMMUNICATION_MODE_UNINITIALIZED;
    }
}


// Device parameters

public final class FSDeviceParams {
    public FSCommunicationMode communicationMode; // communication_mode
    public FSRunStatus status;                    // status

    public int allEnabledInterfaces;

    public String ipAddr;   // ip_addr[39]
    public String ssid;     // ssid[32]
    public String ssidAp;   // ssid_ap[32]
    public String pwAp;     // pw_ap[64]

    public int tcpPort;
    public int udpPort;

    // Sockets/handles – platform dependent
    public long tcpSock;          // fs_sock_t
    public long udpSock;          // fs_sock_t
    public long serialPort;       // fs_serial_handle_t

    public int baudrate;          // int32_t
    public int ping;              // int32_t

    public long hpTimestampMicros; // uint64_t
}


// Device descriptor

public final class FSDeviceDescriptor {
    public FSDeviceId deviceId;  // device_id

    public String deviceName;    // device_name[32]
    public String deviceFw;      // device_fw[32]
    public String deviceDesc;    // device_desc[64]
    public String deviceSn;      // device_sn[32]
    public String deviceModel;   // device_model[32]
}


// Device information container

public final class FSDeviceInfo {

    public final FSDeviceDescriptor deviceDescriptor = new FSDeviceDescriptor();

    public final FSDeviceParams deviceParams = new FSDeviceParams();
    public final FSDriverConfig driverConfig = new FSDriverConfig();

    public final FSMutex lock = new FSMutex();

    public final FSPacketUnion lastReceivedPacket = new FSPacketUnion();

    public final FSBytesRingbuffer dataBuffer = new FSBytesRingbuffer();
    public final FSPacketRingbuffer packetBufQueue = new FSPacketRingbuffer();
    public final FSCommandRingbuffer commandQueue = new FSCommandRingbuffer();
}

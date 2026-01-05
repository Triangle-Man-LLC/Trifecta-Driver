
package com.trifecta;

public class Trifecta_Device {

    private final long devPtr;
    private final SerialBackend serial;

    public Trifecta_Device(SerialPort port) {
        this.devPtr = fs_export_allocate_device();
        this.serial = new SerialBackend(devPtr, port);
    }

    private static class SerialBackend {

        private final long devPtr;
        private final SerialPort port;

        SerialBackend(long devPtr, SerialPort port) {
            this.devPtr = devPtr;
            this.port = port;
            nativeRegister(devPtr, this);
        }

        public int writeBytes(byte[] data) {
            return port.write(data);
        }

        public void onBytesReceived(byte[] data) {
            nativeOnBytesReceived(devPtr, data);
        }

        private static native void nativeRegister(long devPtr, Object serialObj);
        private static native void nativeOnBytesReceived(long devPtr, byte[] data);
    }
}

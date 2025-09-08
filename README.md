# ESP-IDF Driver Setup #

Espressif ESP32 devices are supported under the <b>ESP-IDF</b> framework. This entire sub-folder `Trifecta-ESP-IDF` should be copied into the `components` folder of your ESP-IDF project. 

Typical folder structure: 

```
YOUR_PROJECT_NAME/
  ├── components/
  │   └── Trifecta-ESP-IDF/
  └── main/
      └── main.c
```

Please see the following for example project:

<a href = "https://github.com/Triangle-Man-LLC/Trifecta-Sample-ESP-IDF">ESP-IDF Project Template</a>

You can get additional templates from the <a href = "https://github.com/Triangle-Man-LLC/Trifecta-Driver/tree/esp-idf/examples">project examples folder</a>.

You are strongly advised to set `CONFIG_FREERTOS_HZ=1000` in `sdkconfig`. This ensures that the RTOS tick frequency is exactly 1 millisecond, which is fast enough to read from the IMU at 200 Hz.

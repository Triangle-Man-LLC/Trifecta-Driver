# Drivers for Trifecta IMU Devices #

This repository contains driver software for the <b>Intelligent Sensor Fusion</b> IMU/navigation device series. 

![CI](https://github.com/Triangle-Man-LLC/Trifecta-Driver/actions/workflows/static_analyzer.yml/badge.svg)
![CI](https://github.com/Triangle-Man-LLC/Trifecta-Driver/actions/workflows/execute_tests.yml/badge.svg)

### Installation and Usage ###
Currently supported platforms are <b><a href = "https://github.com/Triangle-Man-LLC/Trifecta-Driver/tree/esp-idf">ESP-IDF (C/C++)</a></b>, <b><a href = "https://github.com/Triangle-Man-LLC/Trifecta-Driver/tree/linux">Linux (C/C++)</a></b>, <b><a href="https://github.com/Triangle-Man-LLC/Trifecta-Python-Samples/tree/main/Trifecta-Python">Python</a></b>, and <b><a href="https://github.com/Triangle-Man-LLC/Trifecta-Python-Samples/tree/main/Trifecta-MicroPython">Micropython</a></b>. (Platforms with in-progress support: <b><a href="https://github.com/Triangle-Man-LLC/Trifecta-Driver/tree/stm32">STM32 (C/C++)</a></b>.) 

Platform-specific installation instructions are located in the README.md of their respective folders. A recommended deployment platform should be at least 32-bit processor with clock speed of 40 MHz and 64 kB of memory.

### Porting the Drivers to Another Platform ###

The drivers can be ported to platforms by implementing the relevant methods listed in the <a href = "/include/FS_Trifecta_Interfaces.h">Interfaces Definitions</a> file. 

Any method which is not relevant/supported (e.g. wireless networking methods on STM32 which does not have wireless networking support) can be made to return -1 to indicate the lack of support.

The specific implementation of the `FS_Interfaces.h` file should be placed in the `/Trifecta-Driver/<PLATFORM_NAME>/FS_Interfaces.c` file of the project.

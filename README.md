
# Drivers for Trifecta IMU Devices #

This repository contains driver software for the <b>Intelligent Sensor Fusion</b> IMU and navigation device series from <b>Triangle Man LLC</b> and/or <b>4rge</b>. 

### Installation and Usage ###

Currently supported platforms are <b><a href = "/Trifecta-ESP-IDF/">ESP-IDF (C/C++)</a></b>, <b><a href = "/Trifecta-Linux/">Linux (C/C++)</a></b>, <b><a href="/Trifecta-Python/">Python</a></b>, and <b><a href="/Trifecta-Micropython/">Micropython</a></b>. (Platforms with in-progress support: <b><a href="/Trifecta-STM32/">STM32 (C/C++)</a></b>.) 

Platform-specific installation instructions are located in the README.md of their respective folders. A recommended deployment platform should be at least 32-bit processor with clock speed of 40 MHz and 64 kB of memory.

### Porting the Drivers to Another Platform ###

The drivers can be ported to platforms by implementing the relevant methods listed in the <a href = "/include/FS_Trifecta_Interfaces.h">Interfaces Definitions</a> file. 
Any method which is not relevant (e.g. wireless networking methods on STM32 which does not have wireless networking support) can be made to return -1 to indicate the lack of support.

The specific implementation of the `FS_Interfaces.h` file should be placed in the `/Trifecta-Driver/<PLATFORM>/FS_Interfaces.c` file of the project.

# Drivers for Trifecta IMU Devices #

This repository contains driver software for the <b>Intelligent Sensor Fusion</b> IMU/navigation device series. 

![CI](https://github.com/Triangle-Man-LLC/Trifecta-Driver/actions/workflows/static_analyzer.yml/badge.svg)
![CI](https://github.com/Triangle-Man-LLC/Trifecta-Driver/actions/workflows/execute_tests.yml/badge.svg)

This branch is the Windows port of the driver. To use, see the following instructions:

### Installation and Usage ###

Building as `.lib/.dll` libraries:

Clone the repository, then go to the root folder `Trifecta-Driver`, and run the following commands in a powershell or command prompt terminal:

```
mkdir build && cd build
cmake -G "Visual Studio 16 2019" .. 
cmake --build . --config Release 
```

The libraries should then be found in the `build/Release/` folder.
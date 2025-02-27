
# Linux/Unix/POSIX Driver Setup #

<b>Linux/POSIX-Compliant Systems</b> are supported through a direct C API. 

### General Usage ###

In general, this entire sub-folder `Trifecta-Linux` should be copied into the `lib` folder of your project. 

Typical folder structure:
```
YOUR_PROJECT_NAME/ 
  ├── lib/ 
  │ └── Trifecta-Linux/ 
  └── src/ 
    └── main.c
```
CMake is recommended as a build system. 

This can also be used as a component in a ROS 1 or ROS 2 project. More on that soon.

### Trying the Examples ###

NOTE: To access serial over USB ports, it is necessary to have `dialout` permissions on most Linux distributions.
If you encounter the `Permission denied` error, use `sudo usermod -a -G dialout $USER` and restart the computer.

For a sample project, see the following:

<a href = "https://github.com/Triangle-Man-LLC/Trifecta-Sample-Linux">Linux Project Template</a>

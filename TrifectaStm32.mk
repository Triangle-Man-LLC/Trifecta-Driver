# List of source files for the FS_Trifecta driver
TRIFECTA_SOURCES = \
    Drivers/Trifecta-Driver/common/FS_Trifecta.c \
    Drivers/Trifecta-Driver/common/FS_Trifecta_Device.c \
    Drivers/Trifecta-Driver/common/FS_Trifecta_Networked.c \
    Drivers/Trifecta-Driver/common/FS_Trifecta_Serial.c \
    Drivers/Trifecta-Driver/stm32/FS_Trifecta_Interfaces.c

# List of include directories for the FS_Trifecta driver
TRIFECTA_INCLUDES = \
    -IDrivers/Trifecta-Driver \
    -IDrivers/Trifecta-Driver/include

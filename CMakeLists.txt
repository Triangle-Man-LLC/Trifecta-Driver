set(trifecta_component_srcs "esp-idf/FS_Trifecta_Interfaces.c" 
                            "esp-idf/FS_Trifecta_Interfaces_Serial.c" 
                            "esp-idf/FS_Trifecta_Interfaces_Networked.c" 
                            "common/FS_Trifecta.c" 
                            "common/FS_Trifecta_Device.c" 
                            "common/FS_Trifecta_Device_Utils.c"
                            "common/FS_Trifecta_Serial.c" 
                            "common/FS_Trifecta_Networked.c")
set(trifecta_required_components driver esp_timer)

if(CONFIG_USB_ENABLED)
    list(APPEND trifecta_required_components usb)
endif()

idf_component_register(
    SRCS "${trifecta_component_srcs}"
    PRIV_REQUIRES ${trifecta_required_components}
    INCLUDE_DIRS "." "include/"
)
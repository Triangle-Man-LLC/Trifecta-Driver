set(trifecta_component_srcs "esp-idf/FS_Trifecta_Interfaces.c" 
                            "common/FS_Trifecta.c" 
                            "common/FS_Trifecta_Device.c" 
                            "common/FS_Trifecta_Device_Utils.c"
                            "common/FS_Trifecta_Serial.c" 
                            "common/FS_Trifecta_Networked.c")

idf_component_register(SRCS "${trifecta_component_srcs}"
                       PRIV_REQUIRES driver
                       INCLUDE_DIRS "." "include/") 

idf_component_register(SRC_DIRS 
                                "./src/led"
                                "./src/imu6500"
                                "./src/modem"
                                "./src/rtc"
                                "./src/sdcard"
                                "./src/web_server"
                                "./src/imu6500/interface"
                                "./src/wifi"
                                "./src/power"
                                "./src"
                        INCLUDE_DIRS 
                                "./src/power"
                                "./src/led"
                                "./src/imu6500"
                                "./src/imu6500/interface"
                                "./src/modem"
                                "./src/rtc"
                                "./src/sdcard"
                                "./src/web_server"
                                "./src/wifi"
                                "./src"                   
                    REQUIRES arduino-esp32 nvs_flash )

add_definitions(
    -DTINY_GSM_YIELD_MS=20
    -DTINY_GSM_RX_BUFFER=1024
)

littlefs_create_partition_image(storage ../data FLASH_IN_PROJECT)

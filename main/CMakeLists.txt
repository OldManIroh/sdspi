set(srcs)

idf_component_register(SRCS "sd_card_main.c"
                       INCLUDE_DIRS "."
                       REQUIRES fatfs sd_card esp_wifi nvs_flash fatfs
                       WHOLE_ARCHIVE)

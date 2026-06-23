idf_component_register(
    SRCS "zdt_can.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_twai libzdt
)

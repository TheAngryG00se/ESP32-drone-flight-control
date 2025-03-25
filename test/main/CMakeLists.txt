idf_component_register(SRCS "main.c" "test_mpu6050.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver
                    PRIV_REQUIRES esp_wifi)
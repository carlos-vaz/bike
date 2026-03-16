set(srcs "tilt_throttle_main.c"
         "imu_task.c"
         "motor_pwm_task.c"
         "../../icm20948/icm20948.c"
         "../../icm20948/icm20948_i2c.c")
idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS ""
                       PRIV_REQUIRES spi_flash esp_driver_i2c driver)

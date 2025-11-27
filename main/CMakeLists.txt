idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES
        stampfly_state
        stampfly_imu
        stampfly_tof
        stampfly_opticalflow
        stampfly_mag
        stampfly_baro
        stampfly_filter
        stampfly_comm
        stampfly_power
        stampfly_buzzer
        stampfly_led
        stampfly_button
        stampfly_motor
        stampfly_cli
)

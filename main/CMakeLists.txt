idf_component_register(
    SRCS "main.cpp" "globals.cpp" "init.cpp" "tasks/tasks.cpp"
    INCLUDE_DIRS "."
    REQUIRES
        stampfly_imu
        stampfly_tof
        stampfly_opticalflow
        stampfly_mag
        stampfly_baro
        stampfly_power
        stampfly_motor
        stampfly_led
        stampfly_buzzer
        stampfly_button
        stampfly_filter
        stampfly_eskf
        sf_algo_fusion
        sf_svc_health
        stampfly_state
        stampfly_comm
        stampfly_cli
        stampfly_telemetry
)

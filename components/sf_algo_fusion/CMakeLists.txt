idf_component_register(
    SRCS "sensor_fusion.cpp"
    INCLUDE_DIRS "include"
    REQUIRES sf_algo_eskf stampfly_math
)

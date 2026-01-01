idf_component_register(
    SRCS "sensor_fusion.cpp"
    INCLUDE_DIRS "include"
    REQUIRES stampfly_eskf stampfly_math
)

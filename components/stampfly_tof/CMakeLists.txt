idf_component_register(
    SRCS "src/vl53l3cx_wrapper.cpp"
    INCLUDE_DIRS "include"
    REQUIRES driver freertos
)

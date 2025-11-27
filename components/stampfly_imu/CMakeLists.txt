idf_component_register(
    SRCS "src/bmi270_wrapper.cpp"
    INCLUDE_DIRS "include"
    REQUIRES driver freertos
)

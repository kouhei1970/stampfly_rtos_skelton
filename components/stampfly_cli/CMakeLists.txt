idf_component_register(
    SRCS "src/cli.cpp"
    INCLUDE_DIRS "include"
    REQUIRES console freertos nvs_flash
)

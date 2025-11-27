idf_component_register(
    SRCS "stampfly_state.cpp" "system_manager.cpp"
    INCLUDE_DIRS "include"
    REQUIRES freertos stampfly_filter
)

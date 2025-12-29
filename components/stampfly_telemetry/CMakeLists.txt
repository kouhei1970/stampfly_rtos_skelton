idf_component_register(
    SRCS "telemetry.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_wifi esp_http_server esp_event freertos stampfly_state
    EMBED_FILES "www/index.html" "www/three.min.js"
)

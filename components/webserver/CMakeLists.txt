idf_component_register(SRCS "webserver.c" "dnsserver.c"
    INCLUDE_DIRS "include"
    REQUIRES freertos commandhandler esp_http_server esp-tls
    EMBED_FILES index.html)

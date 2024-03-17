idf_component_register(SRCS "webserver.c" "dnsserver.c"
    INCLUDE_DIRS "include"
    REQUIRES freertos commandhandler esp_http_server esp-tls unit_acmeasure
    EMBED_FILES index.html)

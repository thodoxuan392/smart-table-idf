idf_component_register(SRCS "smart-table.c"
        INCLUDE_DIRS "."
        REQUIRES nvs_flash
        relay
        commandhandler
        webserver
        wifi
)

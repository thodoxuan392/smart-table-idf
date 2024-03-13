#ifndef __WEBSERVER_H
#define __WEBSERVER_H

#include "stdio.h"
#include "stdbool.h"

#define CONFIG_BODY_MAX_LENGTH  2048
#define COMMAND_BODY_MAX_LENGTH 256
#define USERNAME_MAX_LENGTH     20
#define PASSWORD_MAX_LENGTH     64
typedef struct {
    char username[USERNAME_MAX_LENGTH];
    char password[PASSWORD_MAX_LENGTH];
}WEBSERVER_configuration_t;

bool WEBSERVER_init(void);

#endif /*__WEBSERVER_H*/
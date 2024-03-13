#ifndef COMMANDHANDLER_H
#define COMMANDHANDLER_H

#include "stdbool.h"
#include "stdio.h"

typedef struct {
  bool relay;
} COMMAND_t;

void COMMANDHANDLER_init();
bool COMMANDHANDLER_parse_command(char *payload, COMMAND_t *command);
bool COMMANDHANDLER_handle_command(COMMAND_t *command);

#endif
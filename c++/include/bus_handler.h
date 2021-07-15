
#ifndef BUS_HANDLER_H
#define BUS_HANDLER_H

#include <modbus.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <errno.h>
#include <stdbool.h>

extern modbus_t *ctx;

int init_port(const char *device, bool debug);
void _dealWithModbusError(modbus_t *dev, int addr);

#endif
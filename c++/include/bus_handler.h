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

/**
 * Инициализация порта шины MODBUS RTU.
 * @param const char *device - имя порта шины данных. Например /dev/RS485.
 * @param bool debug - включение режима отладки.
 * @return true если инициализация успешна, false если ошибка инициализации.
 * @note .
 */
int init_port(const char *device, bool debug);

/**
 * Обработка ошибки modbus
 * @param dev - дескриптор modbus устройства
 * @param addr - адрес устройства
 */
void _dealWithModbusError(modbus_t *dev, int addr);

#endif

/******************************************************************************
 * File: bus_handle.c
 * Description: Работа с шиной данных.
 ******************************************************************************/
#include "bus_handler.h"

modbus_t *ctx = nullptr;

int init_port(const char *device, bool debug)
{
	ctx = modbus_new_rtu(device, 460800, 'N', 8, 1);

	if (modbus_connect(ctx) == -1)
	{
		fprintf(stderr, "Connection failed: %s\n",
			modbus_strerror(errno));
		modbus_free(ctx);
		return -1;
	}
	modbus_set_response_timeout(ctx, 0, 100000);
	modbus_set_debug(ctx, debug);
	return 0;
}

void _dealWithModbusError(modbus_t *dev, int addr)
{
	fprintf(stderr, "Error on servo: %i. %s\n\n",
		addr, modbus_strerror(errno));
}

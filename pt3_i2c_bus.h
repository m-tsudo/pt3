#ifndef		__PT3_I2C_BUS_H__
#define		__PT3_I2C_BUS_H__

#include <linux/mutex.h>
#include "pt3_pci.h"

#define PT3_I2C_INST_ADDR0 4096 + 0
#define PT3_I2C_INST_ADDR1 4096 + 2048

typedef struct _PT3_I2C_BUS {
	BAR *bar;
	struct mutex lock;
	__u32 inst_addr;
	__u32 inst_count;
	void *priv;
} PT3_I2C_BUS;

void pt3_i2c_bus_start(PT3_I2C_BUS *bus);
void pt3_i2c_bus_stop(PT3_I2C_BUS *bus);
void pt3_i2c_bus_write(PT3_I2C_BUS *bus, const __u8 *data, __u32 size);
size_t pt3_i2c_bus_read(PT3_I2C_BUS *bus, __u32 size);
__u8 pt3_i2c_bus_data1(PT3_I2C_BUS *bus, size_t index);
void pt3_i2c_bus_end(PT3_I2C_BUS *bus);
void pt3_i2c_bus_sleep(PT3_I2C_BUS *bus, __u32 ms);
void pt3_i2c_bus_copy(PT3_I2C_BUS *bus);
STATUS pt3_i2c_bus_run(PT3_I2C_BUS *bus, __u32 *ack, int copy);
int pt3_i2c_bus_is_clean(PT3_I2C_BUS *bus);
void pt3_i2c_bus_reset(PT3_I2C_BUS *bus);

PT3_I2C_BUS* create_pt3_i2c_bus(BAR *bar);
void free_pt3_i2c_bus(PT3_I2C_BUS *bus);

#endif

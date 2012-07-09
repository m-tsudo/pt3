#ifndef		__PT3_I2C_BUS_H__
#define		__PT3_I2C_BUS_H__

#include <linux/mutex.h>

typedef struct _PT3_I2C_BUS {
	void __iomem *base;
	struct mutex lock;
	__u32 inst_addr;
	__u32 inst_count;
	void *priv;
} PT3_I2C_BUS;

void pt3_i2c_bus_start(PT3_I2C_BUS *bus);
void pt3_i2c_bus_stop(PT3_I2C_BUS *bus);
void pt3_i2c_bus_write(PT3_I2C_BUS *bus, const __u8 *data, __u32 size);
void pt3_i2c_bus_end(PT3_I2C_BUS *bus);
void pt3_i2c_bus_copy(PT3_I2C_BUS *bus);
int pt3_i2c_bus_run(PT3_I2C_BUS *bus, __u32 *ack, int copy);
int pt3_i2c_bus_is_clean(PT3_I2C_BUS *bus);
void pt3_i2c_bus_reset(PT3_I2C_BUS *bus);

PT3_I2C_BUS* create_pt3_i2c_bus(void __iomem *regs);
void free_pt3_i2c_bus(PT3_I2C_BUS *bus);

#endif

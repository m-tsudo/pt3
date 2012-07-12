#ifndef		__PT3_I2C_H__
#define		__PT3_I2C_H__

#include <linux/mutex.h>
#include "pt3_pci.h"
#include "pt3_bus.h"

typedef struct _PT3_I2C {
	BAR *bar;
	struct mutex lock;
} PT3_I2C;

void pt3_i2c_copy(PT3_I2C *i2c, PT3_BUS *bus);
STATUS pt3_i2c_run(PT3_I2C *i2c, PT3_BUS *bus, __u32 *ack, int copy);
int pt3_i2c_is_clean(PT3_I2C *i2c);
void pt3_i2c_reset(PT3_I2C *i2c);

PT3_I2C * create_pt3_i2c(BAR *bar);
void free_pt3_i2c(PT3_I2C *i2c);

#endif

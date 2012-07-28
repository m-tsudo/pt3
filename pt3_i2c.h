/*******************************************************************************
   earthsoft PT3 Linux driver

   This program is free software; you can redistribute it and/or modify it
   under the terms and conditions of the GNU General Public License,
   version 3, as published by the Free Software Foundation.

   This program is distributed in the hope it will be useful, but WITHOUT
   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
   more details.

   The full GNU General Public License is included in this distribution in
   the file called "COPYING".

 *******************************************************************************/

#ifndef		__PT3_I2C_H__
#define		__PT3_I2C_H__

#include <linux/mutex.h>
#include "pt3_pci.h"
#include "pt3_bus.h"

typedef struct _PT3_I2C {
	__u8 __iomem* bar[2];
	struct mutex lock;
} PT3_I2C;

void pt3_i2c_copy(PT3_I2C *i2c, PT3_BUS *bus);
STATUS pt3_i2c_run(PT3_I2C *i2c, PT3_BUS *bus, __u32 *ack, int copy);
int pt3_i2c_is_clean(PT3_I2C *i2c);
void pt3_i2c_reset(PT3_I2C *i2c);

PT3_I2C * create_pt3_i2c(__u8 __iomem *bar[]);
void free_pt3_i2c(PT3_I2C *i2c);

#endif

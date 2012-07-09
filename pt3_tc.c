#include "version.h"

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/version.h>
#include <linux/mutex.h>
#include <linux/sched.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,4,0)
#include <asm/system.h>
#endif
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>

#include "pt3_com.h"
#include "pt3_pci.h"
#include "pt3_i2c_bus.h"
#include "pt3_tc.h"

int
pt3_tc_write(PT3_I2C_BUS *bus, PT3_TC *tc, __u8 addr, const __u8 *data, __u32 size)
{
	__u8 buf;

	pt3_i2c_bus_start(bus);
	buf = tc->tc_addr << 1;
	pt3_i2c_bus_write(bus, &buf, 1);
	pt3_i2c_bus_write(bus, &addr, 1);
	pt3_i2c_bus_write(bus, data, size);
	pt3_i2c_bus_stop(bus);

	pt3_i2c_bus_end(bus);

	return pt3_i2c_bus_run(bus, NULL, 0);
}

int
pt3_tc_write_tuner(PT3_I2C_BUS *bus, PT3_TC *tc, __u8 addr, const __u8 *data, __u32 size)
{
	__u8 buf;

	pt3_i2c_bus_start(bus);
	buf = tc->tc_addr;
	pt3_i2c_bus_write(bus, &buf, 1);
	buf = TC_THROUGH;
	pt3_i2c_bus_write(bus, &buf, 1);
	pt3_i2c_bus_write(bus, &addr, 1);
	pt3_i2c_bus_write(bus, data, size);
	pt3_i2c_bus_stop(bus);

	pt3_i2c_bus_end(bus);

	return pt3_i2c_bus_run(bus, NULL, 0);
}

/* TC_S */

static int
write_pskmsrst(PT3_I2C_BUS *bus, PT3_TC *tc)
{
	__u8 buf;

	buf = 0x01;
	return pt3_tc_write(bus, tc, 0x03, &buf, 1);
}

int
pt3_tc_init_s(PT3_I2C_BUS *bus, PT3_TC *tc)
{
	__u8 buf;

	write_pskmsrst(bus, tc);
	buf = 0x10;
	return pt3_tc_write(bus, tc, 0x1e, &buf, 1);
}

/* TC_T */

static int
write_imsrst(PT3_I2C_BUS *bus, PT3_TC *tc)
{
	__u8 buf;

	buf = 0x01 << 6;
	return pt3_tc_write(bus, tc, 0x01, &buf, 1);
}

int
pt3_tc_init_t(PT3_I2C_BUS *bus, PT3_TC *tc)
{
	__u8 buf;

	write_imsrst(bus, tc);
	buf = 0x10;
	return pt3_tc_write(bus, tc, 0x1c, &buf, 1);
}

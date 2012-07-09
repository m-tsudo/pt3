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

__u8
pt3_tc_address(__u32 pin, int isdb, __u32 index)
{
	__u8 isdb2 = (isdb == PT3_ISDB_S) ? 1 : 0;
	return (__u8)(1 << 4 | pin << 2 | index << 1 | isdb2);
}

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

	return pt3_i2c_bus_run(bus, NULL, 1);
}

int
pt3_tc_read_tuner(PT3_I2C_BUS *bus, PT3_TC *tc, __u8 addr, __u8 *data, __u32 size)
{
	int status;
	__u8 buf;
	__u32 i;
	size_t rindex;

	pt3_i2c_bus_start(bus);
	buf = tc->tc_addr << 1;
	pt3_i2c_bus_write(bus, &buf, 1);
	buf = TC_THROUGH;
	pt3_i2c_bus_write(bus, &buf, 1);
	buf = tc->qm_addr << 1;
	pt3_i2c_bus_write(bus, &buf, 1);
	pt3_i2c_bus_write(bus, &addr, 1);

	pt3_i2c_bus_start(bus);
	buf = tc->tc_addr << 1;
	pt3_i2c_bus_write(bus, &buf, 1);
	buf = TC_THROUGH;
	pt3_i2c_bus_write(bus, &buf, 1);
	buf = tc->qm_addr << 1 | 1;
	pt3_i2c_bus_write(bus, &buf, 1);

	pt3_i2c_bus_start(bus);
	buf = tc->tc_addr << 1 | 1;
	pt3_i2c_bus_write(bus, &buf, 1);
	rindex = pt3_i2c_bus_read(bus, size);
	pt3_i2c_bus_stop(bus);

	pt3_i2c_bus_end(bus);
	status = pt3_i2c_bus_run(bus, NULL, 0);
	for (i = 0; i < size; i++)
		data[i] = pt3_i2c_bus_data1(bus, rindex + i);

	return status;
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

int
pt3_tc_set_powers(PT3_I2C_BUS *bus, PT3_TC *tc, int tuner, int amp)
{
	int status;
	__u8 tuner_power = tuner ? 0x03 : 0x02;
	__u8 amp_power = amp ? 0x03 : 0x02;

	__u8 data = tuner_power << 6 | 0x01 << 4 | amp_power << 2 | 0x01 << 0;

	status = pt3_tc_write(bus, tc, 0x1e, &data, 1);

	return status;
}

PT3_TC *
create_pt3_tc(__u8 tc_addr, __u8 qm_addr)
{
	PT3_TC *tc;

	tc = NULL;

	tc = vzalloc(sizeof(PT3_TC));
	if (tc == NULL)
		goto fail;

	tc->tc_addr = tc_addr;
	tc->qm_addr = qm_addr;
	tc->master_clock_freq = 78;

	return tc;
fail:
	if (tc != NULL)
		vfree(tc);
	return NULL;
}

void
free_pt3_tc(PT3_TC *tc)
{
	vfree(tc);
}

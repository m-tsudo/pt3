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

STATUS
pt3_tc_write(PT3_TC *tc, __u8 addr, const __u8 *data, __u32 size)
{
	__u8 buf;

	pt3_i2c_bus_start(tc->bus);
	buf = tc->tc_addr << 1;
	pt3_i2c_bus_write(tc->bus, &buf, 1);
	pt3_i2c_bus_write(tc->bus, &addr, 1);
	pt3_i2c_bus_write(tc->bus, data, size);
	pt3_i2c_bus_stop(tc->bus);

	pt3_i2c_bus_end(tc->bus);

	return pt3_i2c_bus_run(tc->bus, NULL, 1);
}

STATUS
pt3_tc_write_tuner_without_addr(PT3_TC *tc, const __u8 *data, __u32 size)
{
	__u8 buf;

	pt3_i2c_bus_start(tc->bus);
	buf = tc->tc_addr << 1;
	pt3_i2c_bus_write(tc->bus, &buf, 1);
	buf = TC_THROUGH;
	pt3_i2c_bus_write(tc->bus, &buf, 1);
	buf = tc->tuner_addr << 1;
	pt3_i2c_bus_write(tc->bus, &buf, 1);
	pt3_i2c_bus_write(tc->bus, data, size);
	pt3_i2c_bus_stop(tc->bus);

	pt3_i2c_bus_end(tc->bus);
	return pt3_i2c_bus_run(tc->bus, NULL, 1);
}

STATUS
pt3_tc_read_tuner(PT3_TC *tc, __u8 addr, __u8 *data, __u32 size)
{
	STATUS status;
	__u8 buf;
	__u32 i;
	size_t rindex;

	pt3_i2c_bus_start(tc->bus);
	buf = tc->tc_addr << 1;
	pt3_i2c_bus_write(tc->bus, &buf, 1);
	buf = TC_THROUGH;
	pt3_i2c_bus_write(tc->bus, &buf, 1);
	buf = tc->tuner_addr << 1;
	pt3_i2c_bus_write(tc->bus, &buf, 1);
	pt3_i2c_bus_write(tc->bus, &addr, 1);

	pt3_i2c_bus_start(tc->bus);
	buf = tc->tc_addr << 1;
	pt3_i2c_bus_write(tc->bus, &buf, 1);
	buf = TC_THROUGH;
	pt3_i2c_bus_write(tc->bus, &buf, 1);
	buf = tc->tuner_addr << 1 | 1;
	pt3_i2c_bus_write(tc->bus, &buf, 1);

	pt3_i2c_bus_start(tc->bus);
	buf = tc->tc_addr << 1 | 1;
	pt3_i2c_bus_write(tc->bus, &buf, 1);
	rindex = pt3_i2c_bus_read(tc->bus, size);
	pt3_i2c_bus_stop(tc->bus);

	pt3_i2c_bus_end(tc->bus);
	status = pt3_i2c_bus_run(tc->bus, NULL, 1);
	for (i = 0; i < size; i++)
		data[i] = pt3_i2c_bus_data1(tc->bus, rindex + i);

	return status;
}

STATUS
pt3_tc_write_tuner(PT3_TC *tc, __u8 addr, const __u8 *data, __u32 size)
{
	__u8 buf;

	pt3_i2c_bus_start(tc->bus);
	buf = tc->tc_addr << 1;
	pt3_i2c_bus_write(tc->bus, &buf, 1);
	buf = TC_THROUGH;
	pt3_i2c_bus_write(tc->bus, &buf, 1);
	buf = tc->tuner_addr << 1;
	pt3_i2c_bus_write(tc->bus, &buf, 1);
	pt3_i2c_bus_write(tc->bus, &addr, 1);
	pt3_i2c_bus_write(tc->bus, data, size);
	pt3_i2c_bus_stop(tc->bus);

	pt3_i2c_bus_end(tc->bus);

	return pt3_i2c_bus_run(tc->bus, NULL, 1);
}

/* TC_S */

static STATUS
write_pskmsrst(PT3_TC *tc)
{
	__u8 buf;

	buf = 0x01;
	return pt3_tc_write(tc, 0x03, &buf, 1);
}

STATUS
pt3_tc_init_s(PT3_TC *tc)
{
	STATUS status;
	__u8 buf;

	status = write_pskmsrst(tc);
	if (status)
		return status;
	buf = 0x10;
	status = pt3_tc_write(tc, 0x1e, &buf, 1);
	if (status)
		return status;
	
	return status;
}

/* TC_T */

static STATUS
write_imsrst(PT3_TC *tc)
{
	__u8 buf;

	buf = 0x01 << 6;
	return pt3_tc_write(tc, 0x01, &buf, 1);
}

STATUS
pt3_tc_init_t(PT3_TC *tc)
{
	STATUS status;
	__u8 buf;

	status = write_imsrst(tc);
	if (status)
		return status;
	buf = 0x10;
	status = pt3_tc_write(tc, 0x1c, &buf, 1);
	if (status)
		return status;

	return status;
}

STATUS
pt3_tc_set_powers(PT3_TC *tc, int tuner, int amp)
{
	STATUS status;
	__u8 tuner_power = tuner ? 0x03 : 0x02;
	__u8 amp_power = amp ? 0x03 : 0x02;

	__u8 data = tuner_power << 6 | 0x01 << 4 | amp_power << 2 | 0x01 << 0;

	status = pt3_tc_write(tc, 0x1e, &data, 1);

	return status;
}

static __u8 agc_data_s[2] = { 0xb0, 0x30 };
STATUS
pt3_tc_set_agc_s(PT3_TC *tc, PT3_TC_AGC agc)
{
	STATUS status;
	__u8 data;

	data = (agc == PT3_TC_AGC_AUTO) ? 0xff : 0x00;
	status = pt3_tc_write(tc, 0x0a, &data, 1);
	if (status)
		return status;
	
	data = agc_data_s[BIT_SHIFT_MASK(tc->tc_addr, 1, 1)];
	data |= (agc == PT3_TC_AGC_AUTO) ? 0x01 : 0x00;
	status = pt3_tc_write(tc, 0x10, &data, 1);
	if (status)
		return status;

	data = (agc == PT3_TC_AGC_AUTO) ? 0x40 : 0x00;
	status = pt3_tc_write(tc, 0x10, &data, 1);
	if (status)
		return status;

	status = write_pskmsrst(tc);

	return status;
}

STATUS
pt3_tc_set_agc_t(PT3_TC *tc, PT3_TC_AGC agc)
{
	STATUS status;
	__u8 data;

	data = (agc == PT3_TC_AGC_AUTO) ? 64 : 0;
	status = pt3_tc_write(tc, 0x25, &data, 1);
	if (status)
		return status;

	data = 0x4c;
	data |= (agc == PT3_TC_AGC_AUTO) ? 0x00 : 0x01;
	status = pt3_tc_write(tc, 0x23, &data, 1);
	if (status)
		return status;
	
	status = write_imsrst(tc);

	return status;
}

STATUS
pt3_tc_set_sleep_s(PT3_TC *tc, int sleep)
{
	STATUS status;
	__u8 buf;

	buf = sleep ? 1 : 0;
	status = pt3_tc_write(tc, 0x17, &buf, 1);

	return status;
}

STATUS
pt3_tc_set_ts_pins_mode_s(PT3_TC *tc, PT3_TS_PINS_MODE *mode)
{
	__u32 clock_data, byte, valid;
	__u8 data[2];
	STATUS status;

	clock_data = mode->clock_data;
	byte = mode->byte;
	valid = mode->valid;

	if (clock_data)
		clock_data++;
	if (byte)
		byte++;
	if (valid)
		valid++;

	data[0] = 0x15 | valid << 6;
	data[1] = 0x04 | clock_data << 4 | byte;

	status = pt3_tc_write(tc, 0x1c, &data[0], 1);
	if (status)
		return status;
	status = pt3_tc_write(tc, 0x1f, &data[1], 1);
	if (status)
		return status;

	return status;
}

STATUS
pt3_tc_set_ts_pins_mode_t(PT3_TC *tc, PT3_TS_PINS_MODE *mode)
{
	__u32 clock_data, byte, valid;
	__u8 data;
	STATUS status;

	clock_data = mode->clock_data;
	byte = mode->byte;
	valid = mode->valid;

	if (clock_data)
		clock_data++;
	if (byte)
		byte++;
	if (valid)
		valid++;

	data = (__u8)(0x01 | clock_data << 6 | byte << 4 | valid << 2) ;
	status = pt3_tc_write(tc, 0x1d, &data, 1);

	return status;
}

__u32
pt3_tc_index(PT3_TC *tc)
{
	return BIT_SHIFT_MASK(tc->tc_addr, 1, 1);
}

STATUS
pt3_tc_write_slptim(PT3_TC *tc, int sleep)
{
	STATUS status;
	__u8 data;

	data = 1 << 7 | ((sleep ? 1 :0) <<4);
	status = pt3_tc_write(tc, 0x03, &data, 1);

	return status;
}

PT3_TC *
create_pt3_tc(PT3_I2C_BUS *bus, __u8 tc_addr, __u8 tuner_addr)
{
	PT3_TC *tc;

	tc = NULL;

	tc = vzalloc(sizeof(PT3_TC));
	if (tc == NULL)
		goto fail;

	tc->bus = bus;
	tc->tc_addr = tc_addr;
	tc->tuner_addr = tuner_addr;
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

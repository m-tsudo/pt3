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
#include "pt3_tc.h"

__u8
pt3_tc_address(__u32 pin, int isdb, __u32 index)
{
	__u8 isdb2 = (isdb == PT3_ISDB_S) ? 1 : 0;
	return (__u8)(1 << 4 | pin << 2 | index << 1 | isdb2);
}

STATUS
pt3_tc_write(PT3_TC *tc, PT3_BUS *bus, __u8 addr, const __u8 *data, __u32 size)
{
	STATUS status;
	__u8 buf;
	PT3_BUS *p;

	p = bus ? bus : create_pt3_bus();
	if (p == NULL) {
		printk(KERN_ERR "out of memory.");
		return STATUS_OUT_OF_MEMORY_ERROR;
	}

	pt3_bus_start(p);
	buf = tc->tc_addr << 1;
	pt3_bus_write(p, &buf, 1);
	pt3_bus_write(p, &addr, 1);
	pt3_bus_write(p, data, size);
	pt3_bus_stop(p);

	if (bus) {
		status = STATUS_OK;
	} else {
		pt3_bus_end(p);
		status =  pt3_i2c_run(tc->i2c, p, NULL, 1);
	}

	if (!bus)
		free_pt3_bus(p);

	return status;
}

STATUS
pt3_tc_read_tuner_without_addr(PT3_TC *tc, PT3_BUS *bus, __u8 *data, __u32 size)
{
	STATUS status;
	__u8 buf[size];
	__u32 i;
	size_t rindex;
	PT3_BUS *p;

	memset(buf, 0, size);

	p = bus ? bus : create_pt3_bus();
	if (p == NULL) {
		printk(KERN_ERR "out of memory.");
		return STATUS_OUT_OF_MEMORY_ERROR;
	}

	pt3_bus_start(p);
	buf[0] = tc->tc_addr << 1;
	pt3_bus_write(p, &buf[0], 1);
	buf[0] = TC_THROUGH;
	pt3_bus_write(p, &buf[0], 1);
	buf[0] = (tc->tuner_addr << 1) | 0x01;
	pt3_bus_write(p, &buf[0], 1);

	pt3_bus_start(p);
	buf[0] = (tc->tc_addr << 1) | 0x01;
	rindex = pt3_bus_read(p, &buf[0], size);
	pt3_bus_stop(p);

	if (bus) {
		status = STATUS_OK;
	} else {
		pt3_bus_end(p);
		status = pt3_i2c_run(tc->i2c, p, NULL, 1);
		for (i = 0; i < size; i++)
			data[i] = pt3_bus_data1(p, rindex + i);
	}

	if (!bus)
		free_pt3_bus(p);

#if 0
	printk(KERN_DEBUG "read_tuner_without tc_addr=0x%x tuner_addr=0x%x",
			tc->tc_addr, tc->tuner_addr);
#endif

	return status;
}

STATUS
pt3_tc_write_tuner_without_addr(PT3_TC *tc, PT3_BUS *bus, const __u8 *data, __u32 size)
{
	STATUS status;
	__u8 buf;
	PT3_BUS *p;

	p = bus ? bus : create_pt3_bus();
	if (p == NULL) {
		printk(KERN_ERR "out of memory.");
		return STATUS_OUT_OF_MEMORY_ERROR;
	}

	pt3_bus_start(p);
	buf = tc->tc_addr << 1;
	pt3_bus_write(p, &buf, 1);
	buf = TC_THROUGH;
	pt3_bus_write(p, &buf, 1);
	buf = tc->tuner_addr << 1;
	pt3_bus_write(p, &buf, 1);
	pt3_bus_write(p, data, size);
	pt3_bus_stop(p);

	if (bus)
		status = STATUS_OK;
	else {
		pt3_bus_end(p);
		status = pt3_i2c_run(tc->i2c, p, NULL, 1);
	}

	if (!bus)
		free_pt3_bus(p);

	return status;
}

STATUS
pt3_tc_read_tuner(PT3_TC *tc, PT3_BUS *bus, __u8 addr, __u8 *data, __u32 size)
{
	STATUS status;
	__u8 buf[size];
	__u32 i;
	size_t rindex;
	PT3_BUS *p;

	memset(buf, 0, size);

	p = bus ? bus : create_pt3_bus();
	if (p == NULL) {
		printk(KERN_ERR "out of memory.");
		return STATUS_OUT_OF_MEMORY_ERROR;
	}

	pt3_bus_start(p);
	buf[0] = tc->tc_addr << 1;
	pt3_bus_write(p, &buf[0], 1);
	buf[0] = TC_THROUGH;
	pt3_bus_write(p, &buf[0], 1);
	buf[0] = tc->tuner_addr << 1;
	pt3_bus_write(p, &buf[0], 1);
	pt3_bus_write(p, &addr, 1);

	pt3_bus_start(p);
	buf[0] = tc->tc_addr << 1;
	pt3_bus_write(p, &buf[0], 1);
	buf[0] = TC_THROUGH;
	pt3_bus_write(p, &buf[0], 1);
	buf[0] = (tc->tuner_addr << 1) | 1;
	pt3_bus_write(p, &buf[0], 1);

	pt3_bus_start(p);
	buf[0] = (tc->tc_addr << 1) | 1;
	pt3_bus_write(p, &buf[0], 1);
	rindex = pt3_bus_read(p, &buf[0], size);
	pt3_bus_stop(p);

	if (bus) {
		status = STATUS_OK;
	} else {
		pt3_bus_end(p);
		status = pt3_i2c_run(tc->i2c, p, NULL, 1);
		for (i = 0; i < size; i++)
			data[i] = pt3_bus_data1(p, rindex + i);
	}

	if (!bus)
		free_pt3_bus(p);

#if 0
	printk(KERN_DEBUG "read_tuner tc_addr=0x%x tuner_addr=0x%x",
			tc->tc_addr, tc->tuner_addr);
#endif

	return status;
}

STATUS
pt3_tc_write_tuner(PT3_TC *tc, PT3_BUS *bus, __u8 addr, const __u8 *data, __u32 size)
{
	STATUS status;
	__u8 buf;
	PT3_BUS *p;

	p = bus ? bus : create_pt3_bus();
	if (p == NULL) {
		printk(KERN_ERR "out of memory.");
		return STATUS_OUT_OF_MEMORY_ERROR;
	}

	pt3_bus_start(p);
	buf = tc->tc_addr << 1;
	pt3_bus_write(p, &buf, 1);
	buf = TC_THROUGH;
	pt3_bus_write(p, &buf, 1);
	buf = tc->tuner_addr << 1;
	pt3_bus_write(p, &buf, 1);
	pt3_bus_write(p, &addr, 1);
	pt3_bus_write(p, data, size);
	pt3_bus_stop(p);

	if (bus)
		status = STATUS_OK;
	else {
		pt3_bus_end(p);
		status = pt3_i2c_run(tc->i2c, p, NULL, 1);
	}

	if (!bus)
		free_pt3_bus(p);

	return status;
}

/* TC_S */

static STATUS
write_pskmsrst(PT3_TC *tc, PT3_BUS *bus)
{
	__u8 buf;

	buf = 0x01;
	return pt3_tc_write(tc, bus, 0x03, &buf, 1);
}

STATUS
pt3_tc_init_s(PT3_TC *tc, PT3_BUS *bus)
{
	STATUS status;
	__u8 buf;

	status = write_pskmsrst(tc, bus);
	if (status)
		return status;
	buf = 0x10;
	status = pt3_tc_write(tc, bus, 0x1e, &buf, 1);
	if (status)
		return status;
	
	return status;
}

/* TC_T */

static STATUS
write_imsrst(PT3_TC *tc, PT3_BUS *bus)
{
	__u8 buf;

	buf = 0x01 << 6;
	return pt3_tc_write(tc, bus, 0x01, &buf, 1);
}

STATUS
pt3_tc_init_t(PT3_TC *tc, PT3_BUS *bus)
{
	STATUS status;
	__u8 buf;

	status = write_imsrst(tc, bus);
	if (status)
		return status;
	buf = 0x10;
	status = pt3_tc_write(tc, bus, 0x1c, &buf, 1);
	if (status)
		return status;

	return status;
}

STATUS
pt3_tc_set_powers(PT3_TC *tc, PT3_BUS *bus, int tuner, int amp)
{
	STATUS status;
	__u8 tuner_power = tuner ? 0x03 : 0x02;
	__u8 amp_power = amp ? 0x03 : 0x02;

	__u8 data = (tuner_power << 6) | (0x01 << 4) | (amp_power << 2) | 0x01 << 0;

	status = pt3_tc_write(tc, bus, 0x1e, &data, 1);

	return status;
}

static __u8 agc_data_s[2] = { 0xb0, 0x30 };
STATUS
pt3_tc_set_agc_s(PT3_TC *tc, PT3_BUS *bus, PT3_TC_AGC agc)
{
	STATUS status;
	__u8 data;

	data = (agc == PT3_TC_AGC_AUTO) ? 0xff : 0x00;
	status = pt3_tc_write(tc, bus, 0x0a, &data, 1);
	if (status)
		return status;
	
	data = agc_data_s[pt3_tc_index(tc)];
	data |= (agc == PT3_TC_AGC_AUTO) ? 0x01 : 0x00;
	status = pt3_tc_write(tc, bus, 0x10, &data, 1);
	if (status)
		return status;

	data = (agc == PT3_TC_AGC_AUTO) ? 0x40 : 0x00;
	status = pt3_tc_write(tc, bus, 0x11, &data, 1);
	if (status)
		return status;

	status = write_pskmsrst(tc, bus);

	return status;
}

STATUS
pt3_tc_set_agc_t(PT3_TC *tc, PT3_BUS *bus, PT3_TC_AGC agc)
{
	STATUS status;
	__u8 data;

	data = (agc == PT3_TC_AGC_AUTO) ? 64 : 0;
	status = pt3_tc_write(tc, bus, 0x25, &data, 1);
	if (status)
		return status;

	data = 0x4c;
	data |= (agc == PT3_TC_AGC_AUTO) ? 0x00 : 0x01;
	status = pt3_tc_write(tc, bus, 0x23, &data, 1);
	if (status)
		return status;
	
	status = write_imsrst(tc, bus);

	return status;
}

STATUS
pt3_tc_set_sleep_s(PT3_TC *tc, PT3_BUS *bus, int sleep)
{
	STATUS status;
	__u8 buf;

	buf = sleep ? 1 : 0;
	status = pt3_tc_write(tc, bus, 0x17, &buf, 1);

	return status;
}

STATUS
pt3_tc_set_ts_pins_mode_s(PT3_TC *tc, PT3_BUS *bus, PT3_TS_PINS_MODE *mode)
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

	data[0] = 0x15 | (valid << 6);
	data[1] = 0x04 | (clock_data << 4) | byte;

	status = pt3_tc_write(tc, bus, 0x1c, &data[0], 1);
	if (status)
		return status;
	status = pt3_tc_write(tc, bus, 0x1f, &data[1], 1);
	if (status)
		return status;

	return status;
}

STATUS
pt3_tc_set_ts_pins_mode_t(PT3_TC *tc, PT3_BUS *bus, PT3_TS_PINS_MODE *mode)
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

	data = (__u8)(0x01 | (clock_data << 6) | (byte << 4) | (valid << 2)) ;
	status = pt3_tc_write(tc, bus, 0x1d, &data, 1);

	return status;
}

__u32
pt3_tc_index(PT3_TC *tc)
{
	return BIT_SHIFT_MASK(tc->tc_addr, 1, 1);
}

STATUS
pt3_tc_write_slptim(PT3_TC *tc, PT3_BUS *bus, int sleep)
{
	STATUS status;
	__u8 data;

	data = (1 << 7) | ((sleep ? 1 :0) <<4);
	status = pt3_tc_write(tc, bus, 0x03, &data, 1);

	return status;
}

PT3_TC *
create_pt3_tc(PT3_I2C *i2c, __u8 tc_addr, __u8 tuner_addr)
{
	PT3_TC *tc;

	tc = NULL;

	tc = vzalloc(sizeof(PT3_TC));
	if (tc == NULL)
		goto fail;

	tc->i2c = i2c;
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

__u32
time_diff(struct timeval *st, struct timeval *et)
{
	__u32 diff;
	diff = (et->tv_sec - st->tv_sec) * 1000000 + (et->tv_usec - st->tv_usec);
#if 0
	printk(KERN_DEBUG "time diff = %d\n", diff / 1000);
#endif
	return diff / 1000;
}

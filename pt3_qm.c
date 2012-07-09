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
#include "pt3_qm.h"

#define INIT_DUMMY_RESET 0x0C

/* TUNER_S */
void
pt3_qm_get_channel_freq(__u32 channel, int *bs, __u32 *number, __u32 *freq)
{
	if (channel < 12) {
		*bs = 1;
		*number = 1 + 2 * channel;
		*freq = 104948 + 3836 * channel;
	} else if (channel < 24) {
		channel -= 12;
		*bs = 0;
		*number = 2 + 2 * channel;
		*freq = 161300 + 4000 * channel;
	} else {
		channel -= 24;
		*bs = 0;
		*number = 1 + 2 * channel;
		*freq = 159300 + 4000 * channel;
	}
}

/* QM */
static __u8 rw_reg[0x20] = {
	0x48, 0x1c, 0xa0, 0x10, 0xbc, 0xc5, 0x20, 0x33,
	0x06, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00,
	0x00, 0xff, 0xf3, 0x00, 0x2a, 0x64, 0xa6, 0x86,
	0x8c, 0xcf, 0xb8, 0xf1, 0xa8, 0xf2, 0x89, 0x00,
};

static __u8 flag[0x20] = {
	0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0,
	0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1
};

static int
qm_write(PT3_I2C_BUS *bus, PT3_TC *tc, PT3_QM *qm, __u8 addr, __u8 data)
{
	int ret;
	ret = pt3_tc_write_tuner(bus, tc, addr, &data, sizeof(data));
	qm->reg[addr] = data;
	return ret;
}

static int
qm_read(PT3_I2C_BUS *bus, PT3_TC *tc, PT3_QM *qm, __u8 addr, __u8 *data)
{
	int status;
	if ((addr = 0x00 ) || (addr == 0x0d))
		status = pt3_tc_read_tuner(bus, tc, addr, data, 1);
	else 
		status = 0;

	return status;
}

static void
qm_sleep(PT3_I2C_BUS *bus, __u32 ms)
{
	if (bus)
		pt3_i2c_bus_sleep(bus, ms);
	else 
		schedule_timeout_interruptible(msecs_to_jiffies(ms));	
}

static int
qm_set_sleep_mode(PT3_I2C_BUS *bus, PT3_TC *tc, PT3_QM *qm)
{
	int status;
	PT3_QM_PARAM *param;
	
	param = &qm->param;

	if (param->standby) {
		qm->reg[0x01] &= (~(1 <<3)) & 0xff;
		qm->reg[0x01] |= 1 << 0;
		qm->reg[0x05] |= 1 << 3;

		status = qm_write(bus, tc, qm, 0x05, qm->reg[0x05]);
		if (status)
			return status;
		status = qm_write(bus, tc, qm, 0x01, qm->reg[0x01]);
		if (status)
			return status;
	} else {
		qm->reg[0x01] |= 1 <<3;
		qm->reg[0x01] &= (~(1 << 0)) & 0xff;
		qm->reg[0x05] &= (~(1 << 3)) & 0xff;

		status = qm_write(bus, tc, qm, 0x01, qm->reg[0x01]);
		if (status)
			return status;
		status = qm_write(bus, tc, qm, 0x05, qm->reg[0x05]);
		if (status)
			return status;
	}

	return status;
}

static int
qm_set_search_mode(PT3_I2C_BUS *bus, PT3_TC *tc, PT3_QM *qm)
{
	int status;
	PT3_QM_PARAM *param;
	
	param = &qm->param;

	if (param->fast_search_mode) {
		qm->reg[0x03] |= 0x01;
		status = qm_write(bus, tc, qm, 0x03, qm->reg[0x03]);
		if (status)
			return status;
	} else {
		qm->reg[0x03] |= 0xfe;
		status = qm_write(bus, tc, qm, 0x03, qm->reg[0x03]);
		if (status)
			return status;
	}

	return status;
}

static __u8 qm_address[MAX_TUNER] = { 0x63, 0x60 };

__u8
pt3_qm_address(__u32 index)
{
	return qm_address[index];
}

int
pt3_qm_set_sleep(PT3_I2C_BUS *bus, PT3_TC *tc, PT3_QM *qm, int sleep)
{
	// TODO
	return 0;
}

void
pt3_qm_dummy_reset(PT3_I2C_BUS * bus, PT3_TC *tc, PT3_QM *qm)
{
	qm_write(bus, tc, qm, 0x01, INIT_DUMMY_RESET);
	qm_write(bus, tc, qm, 0x01, INIT_DUMMY_RESET);
}

void
pt3_qm_init_reg_param(PT3_QM *qm)
{
	memcpy(qm->reg, rw_reg, sizeof(rw_reg));

	qm->param.channel_freq = 0;
	qm->param.crystal_freq = 16000;
	qm->param.fast_search_mode = 0;
	qm->param.standby = 0;
	qm->param.lpf_wait_time = 20;
	qm->param.fast_search_wait_time = 4;
	qm->param.normal_search_wait_time = 15;
}

int
pt3_qm_init(PT3_I2C_BUS * bus, PT3_TC *tc, PT3_QM *qm)
{
	__u8 i_data;
	__u32 i;
	int status;
	status = qm_write(bus, tc, qm, 0x01, INIT_DUMMY_RESET);
	if (status)
		return status;

	qm_sleep(bus, 1);

	i_data = qm->reg[0x01];
	i_data |= 0x10;
	status = qm_write(bus, tc, qm, 0x1, i_data);
	if (status)
		return status;
	
	// ID check
	status = qm_read(bus, tc, qm, 0x00, &i_data);
	if (status)
		return status;

	if ((bus == NULL) && (i_data != 0x48))
		return -1;

	// LPF tuning on
	qm_sleep(bus, 1);
	qm->reg[0xc] |= 0x40;
	status = qm_write(bus, tc, qm, 0x0c, qm->reg[0x0c]);
	if (status)
		return status;
	qm_sleep(bus, qm->param.lpf_wait_time);

	for (i = 0; i < 0x20; i++) {
		if (flag[i] == 1) {
			status = qm_write(bus, tc, qm, i, qm->reg[i]);
			if (status)
				return status;
		}
	}

	status = qm_set_sleep_mode(bus, tc, qm);
	if (status)
		return status;

	status = qm_set_search_mode(bus, tc, qm);
	if (status)
		return status;

	return status;
}

PT3_QM *
create_pt3_qm()
{
	PT3_QM *qm;

	qm = NULL;

	qm = vzalloc(sizeof(PT3_QM));
	if (qm == NULL)
		goto fail;

	qm->sleep = 1;

	return qm;
fail:
	if (qm != NULL)
		vfree(qm);
	return NULL;
}

void
free_pt3_qm(PT3_QM *qm)
{
	vfree(qm);
}

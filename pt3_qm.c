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
#include <linux/time.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,4,0)
#include <asm/system.h>
#endif
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>

#include "pt3_com.h"
#include "pt3_pci.h"
#include "pt3_qm.h"

#define INIT_DUMMY_RESET 0x0c

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

static STATUS
qm_write(PT3_QM *qm, PT3_BUS *bus, __u8 addr, __u8 data)
{
	STATUS ret;
	ret = pt3_tc_write_tuner(qm->tc, bus, addr, &data, sizeof(data));
	qm->reg[addr] = data;
	return ret;
}

static STATUS
qm_read(PT3_QM *qm, PT3_BUS *bus, __u8 addr, __u8 *data)
{
	STATUS status;
	if ((addr == 0x00 ) || (addr == 0x0d)) {
		status = pt3_tc_read_tuner(qm->tc, bus, addr, data, 1);
#if 0
		if (!bus)
			PT3_PRINTK(7, KERN_DEBUG "qm_read addr=0x%02x data=0x%02x\n", addr, *data);
#endif
	} else 
		status = STATUS_OK;

	return status;
}

static void
qm_sleep(PT3_QM *qm, PT3_BUS *bus, __u32 ms)
{
	if (bus)
		pt3_bus_sleep(bus, ms);
	else 
		schedule_timeout_interruptible(msecs_to_jiffies(ms));	
}

static STATUS
qm_set_sleep_mode(PT3_QM *qm, PT3_BUS *bus)
{
	STATUS status;
	PT3_QM_PARAM *param;
	
	param = &qm->param;

	if (param->standby) {
		qm->reg[0x01] &= (~(1 << 3)) & 0xff;
		qm->reg[0x01] |= 1 << 0;
		qm->reg[0x05] |= 1 << 3;

		status = qm_write(qm, bus, 0x05, qm->reg[0x05]);
		if (status)
			return status;
		status = qm_write(qm, bus, 0x01, qm->reg[0x01]);
		if (status)
			return status;
	} else {
		qm->reg[0x01] |= 1 <<3;
		qm->reg[0x01] &= (~(1 << 0)) & 0xff;
		qm->reg[0x05] &= (~(1 << 3)) & 0xff;

		status = qm_write(qm, bus, 0x01, qm->reg[0x01]);
		if (status)
			return status;
		status = qm_write(qm, bus, 0x05, qm->reg[0x05]);
		if (status)
			return status;
	}

	return status;
}

static STATUS
qm_set_search_mode(PT3_QM *qm, PT3_BUS *bus)
{
	STATUS status;
	PT3_QM_PARAM *param;
	
	param = &qm->param;

	if (param->fast_search_mode) {
		qm->reg[0x03] |= 0x01;
		status = qm_write(qm, bus, 0x03, qm->reg[0x03]);
		if (status)
			return status;
	} else {
		qm->reg[0x03] &= 0xfe;
		status = qm_write(qm, bus, 0x03, qm->reg[0x03]);
		if (status)
			return status;
	}

	return status;
}

static __u32 FREQ_TABLE[9][3] = {
	{ 2151000, 1, 7 },
	{ 1950000, 1, 6 },
	{ 1800000, 1, 5 },
	{ 1600000, 1, 4 },
	{ 1450000, 1, 3 },
	{ 1250000, 1, 2 },
	{ 1200000, 0, 7 },
	{  975000, 0, 6 },
	{  950000, 0, 0 }
};

static __u32 SD_TABLE[24][2][3] = {
	{{0x38fae1, 0xd, 0x5},{0x39fae1, 0xd, 0x5},},
	{{0x3f570a, 0xe, 0x3},{0x570a, 0xe, 0x3},},
	{{0x5b333, 0xe, 0x5},{0x6b333, 0xe, 0x5},},
	{{0x3c0f5c, 0xf, 0x4},{0x3d0f5c, 0xf, 0x4},},
	{{0x26b85, 0xf, 0x6},{0x36b85, 0xf, 0x6},},
	{{0x38c7ae, 0x10, 0x5},{0x39c7ae, 0x10, 0x5},},
	{{0x3f23d7, 0x11, 0x3},{0x23d7, 0x11, 0x3},},
	{{0x58000, 0x11, 0x5},{0x68000, 0x11, 0x5},},
	{{0x3bdc28, 0x12, 0x4},{0x3cdc28, 0x12, 0x4},},
	{{0x23851, 0x12, 0x6},{0x33851, 0x12, 0x6},},
	{{0x38947a, 0x13, 0x5},{0x39947a, 0x13, 0x5},},
	{{0x3ef0a3, 0x14, 0x3},{0x3ff0a3, 0x14, 0x3},},
	{{0x3c8000, 0x16, 0x4},{0x3d8000, 0x16, 0x4},},
	{{0x48000, 0x16, 0x6},{0x58000, 0x16, 0x6},},
	{{0x3c8000, 0x17, 0x5},{0x3d8000, 0x17, 0x5},},
	{{0x48000, 0x18, 0x3},{0x58000, 0x18, 0x3},},
	{{0x3c8000, 0x18, 0x6},{0x3d8000, 0x18, 0x6},},
	{{0x48000, 0x19, 0x4},{0x58000, 0x19, 0x4},},
	{{0x3c8000, 0x1a, 0x3},{0x3d8000, 0x1a, 0x3},},
	{{0x48000, 0x1a, 0x5},{0x58000, 0x1a, 0x5},},
	{{0x3c8000, 0x1b, 0x4},{0x3d8000, 0x1b, 0x4},},
	{{0x48000, 0x1b, 0x6},{0x58000, 0x1b, 0x6},},
	{{0x3c8000, 0x1c, 0x5},{0x3d8000, 0x1c, 0x5},},
	{{0x48000, 0x1d, 0x3},{0x58000, 0x1d, 0x3},},
};

static STATUS
qm_tuning(PT3_QM *qm, PT3_BUS *bus, __u32 *sd, __u32 channel)
{
	STATUS status;
	PT3_QM_PARAM *param = &qm->param;
	__u8 i_data;
	__u32 index, i, N, A;
	// __u32 a;
	//double M, b;	// double

	qm->reg[0x08] &= 0xf0;
	qm->reg[0x08] |= 0x09;

	qm->reg[0x13] &= 0x9f;
	qm->reg[0x13] |= 0x20;

	for (i = 0; i < 8; i++) {
		if ((FREQ_TABLE[i+1][0] <= param->channel_freq) &&
				(param->channel_freq < FREQ_TABLE[i][0])) {
			i_data = qm->reg[0x02];
			i_data &= 0x0f;
			i_data |= FREQ_TABLE[i][1] << 7;
			i_data |= FREQ_TABLE[i][2] << 4;
			status = qm_write(qm, bus, 0x02, i_data);
		}
	}

#if 0
	//M = (double)(param->channel_freq) / (double)(param->crystal_freq);
	M = param->channel_freq / param->crystal_freq;
	//a = (__s32)(M + 0.5);
	a = (__s32)((M * 10 + 5) / 10);
	b = M - a;

	N = (a - 12) >> 2;
	A = a - 4 * (N + 1) - 5;

	if (0 <= b)
		//*sd = (__u32)(pow(2, 20.) * b);
		*sd = (__u32)((2^20) * b);
	else
		//*sd = (__u32)(pow(2, 20.) * b + (1 << 22);
		*sd = (__u32)((2 ^ 20) * b + (1 << 22));
#else
	index = pt3_tc_index(qm->tc);
	*sd = SD_TABLE[channel][index][0];
	N = SD_TABLE[channel][index][1];
	A = SD_TABLE[channel][index][2];
#endif
	qm->reg[0x06] &= 0x40;
	qm->reg[0x06] |= N;
	status = qm_write(qm, bus, 0x06, qm->reg[0x06]);
	if (status)
		return status;
	
	qm->reg[0x07] &= 0xf0;
	qm->reg[0x07] |= A & 0x0f;
	status = qm_write(qm, bus, 0x07, qm->reg[0x07]);
	if (status)
		return status;

	return status;
}

static STATUS
qm_local_lpf_tuning(PT3_QM *qm, PT3_BUS *bus, int lpf, __u32 channel)
{
	PT3_QM_PARAM *param = &qm->param;
	__u8 i_data;
	__u32 sd;
	STATUS status;

	sd = 0;
	status = qm_tuning(qm, bus, &sd, channel);
	if (status)
		return status;

	if (lpf) {
		i_data = qm->reg[0x08] & 0xf0;
		i_data |= 2;
		status = qm_write(qm, bus, 0x08, i_data);
	} else {
		status = qm_write(qm, bus, 0x08, qm->reg[0x08]);
	}
	if (status)
		return status;

	qm->reg[0x09] &= 0xc0;
	qm->reg[0x09] |= (sd >> 16) & 0x3f;
	qm->reg[0x0a] = (sd >> 8) & 0xff;
	qm->reg[0x0b] = (sd >> 0) & 0xff;
	status = qm_write(qm, bus, 0x09, qm->reg[0x09]);
	if (status)
		return status;
	status = qm_write(qm, bus, 0x0a, qm->reg[0x0a]);
	if (status)
		return status;
	status = qm_write(qm, bus, 0x0b, qm->reg[0x0b]);
	if (status)
		return status;

	if (!lpf) {
		status = qm_write(qm, bus, 0x13, qm->reg[0x13]);
		if (status)
			return status;
	}

	if (lpf) {
		i_data = qm->reg[0x0c];
		i_data &= 0x3f;
		status = qm_write(qm, bus, 0x0c, i_data);
		if (status)
			return status;
		qm_sleep(qm, bus, 1);

		i_data = qm->reg[0x0c];
		i_data |= 0xc0;
		status = qm_write(qm, bus, 0x0c, i_data);
		if (status)
			return status;
	} else {
		i_data = qm->reg[0x0c];
		i_data &= 0x7f;
		status = qm_write(qm, bus, 0x0c, i_data);
		if (status)
			return status;
		qm_sleep(qm, bus, 2);	// 1024usec

		i_data = qm->reg[0x0c];
		i_data |= 0x80;
		status = qm_write(qm, bus, 0x0c, i_data);
		if (status)
			return status;
	}

	if (lpf) {
		qm_sleep(qm, bus, param->lpf_wait_time);
	} else {
		if (qm->reg[0x03] & 0x01) {
			qm_sleep(qm, bus, param->fast_search_wait_time);
		} else {
			qm_sleep(qm, bus, param->normal_search_wait_time);
		}
	}

	if (lpf) {
		status = qm_write(qm, bus, 0x08, 0x09);
		if (status)
			return status;
		status = qm_write(qm, bus, 0x13, qm->reg[0x13]);
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

STATUS
pt3_qm_set_sleep(PT3_QM *qm, int sleep)
{
	STATUS status;
	PT3_TS_PIN_MODE mode;

	mode = sleep ? PT3_TS_PIN_MODE_LOW : PT3_TS_PIN_MODE_NORMAL;
	qm->param.standby = sleep;

	if (sleep) {
		status = pt3_tc_set_agc_s(qm->tc, NULL, PT3_TC_AGC_MANUAL);
		if (status)
			return status;
		qm_set_sleep_mode(qm, NULL);
		pt3_tc_set_sleep_s(qm->tc, NULL, sleep);
	} else {
		pt3_tc_set_sleep_s(qm->tc, NULL, sleep);
		qm_set_sleep_mode(qm, NULL);
	}

	qm->sleep = sleep;

	return STATUS_OK;
}

void
pt3_qm_dummy_reset(PT3_QM *qm, PT3_BUS *bus)
{
	qm_write(qm, bus, 0x01, INIT_DUMMY_RESET);
	qm_write(qm, bus, 0x01, INIT_DUMMY_RESET);
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

STATUS
pt3_qm_init(PT3_QM *qm, PT3_BUS *bus)
{
	__u8 i_data;
	__u32 i;
	STATUS status;

	// soft reset on
	status = qm_write(qm, bus, 0x01, INIT_DUMMY_RESET);
	if (status)
		return status;

	qm_sleep(qm, bus, 1);

	// soft reset off
	i_data = qm->reg[0x01];
	i_data |= 0x10;
	status = qm_write(qm, bus, 0x01, i_data);
	if (status)
		return status;
	
	// ID check
	status = qm_read(qm, bus, 0x00, &i_data);
	if (status)
		return status;

	if ((bus == NULL) && (i_data != 0x48))
		return STATUS_INVALID_PARAM_ERROR;

	// LPF tuning on
	qm_sleep(qm, bus, 1);
	qm->reg[0x0c] |= 0x40;
	status = qm_write(qm, bus, 0x0c, qm->reg[0x0c]);
	if (status)
		return status;
	qm_sleep(qm, bus, qm->param.lpf_wait_time);

	for (i = 0; i < sizeof(flag); i++) {
		if (flag[i] == 1) {
			status = qm_write(qm, bus, i, qm->reg[i]);
			if (status)
				return status;
		}
	}

	status = qm_set_sleep_mode(qm, bus);
	if (status)
		return status;

	status = qm_set_search_mode(qm, bus);
	if (status)
		return status;

	return status;
}

STATUS
pt3_qm_get_locked(PT3_QM *qm, PT3_BUS *bus, int *locked)
{
	STATUS status;

	status = qm_read(qm, bus, 0x0d, &qm->reg[0x0d]);
	if (status)
		return status;
	
	if (qm->reg[0x0d] & 0x40)
		*locked = 1;
	else
		*locked = 0;
	
	return status;
}

STATUS
pt3_qm_set_frequency(PT3_QM *qm, __u32 channel, __s32 offset)
{
	STATUS status;
	int bs, locked;
	__u32 number, freq, freq_khz;
    ktime_t begin, now;

	status = pt3_tc_set_agc_s(qm->tc, NULL, PT3_TC_AGC_MANUAL);
	if (status)
		return status;
	
	pt3_qm_get_channel_freq(channel, &bs, &number, &freq);
	freq_khz = freq * 10 + offset;
	if (pt3_tc_index(qm->tc) == 0)
		freq_khz -= 500;
	else
		freq_khz += 500;
	qm->param.channel_freq = freq_khz;
	//PT3_PRINTK(7, KERN_DEBUG "frequency %d Khz\n", freq_khz);


	status = qm_local_lpf_tuning(qm, NULL, 1, channel);
	if (status)
		return status;

    begin = ktime_get();
	while (1) {
        now = ktime_get();

		status = pt3_qm_get_locked(qm, NULL, &locked);
		if (status)
			return status;
		if (locked)
			break;

		if (ktime_sub(now, begin) >= ktime_set(0, 100 * NSEC_PER_USEC))
			break;

		schedule_timeout_interruptible(msecs_to_jiffies(1));	
	}
	// PT3_PRINTK(7, KERN_DEBUG "qm_get_locked %d status=0x%x\n", locked, status);
	if (!locked)
		return STATUS_PLL_LOCK_TIMEOUT_ERROR;
	
	status = pt3_tc_set_agc_s(qm->tc, NULL, PT3_TC_AGC_AUTO);
	if (status)
		return status;

	qm->channel = channel;
	qm->offset = offset;

	return status;
}

PT3_QM *
create_pt3_qm(PT3_I2C *i2c, PT3_TC *tc)
{
	PT3_QM *qm;

	qm = NULL;

	qm = pt3_vzalloc(sizeof(PT3_QM));
	if (qm == NULL)
		goto fail;

	qm->i2c = i2c;
	qm->tc = tc;
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

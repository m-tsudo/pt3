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
#include "pt3_mx.h"

typedef struct _SHF_TYPE {
	__u32	freq;		// Channel center frequency
	__u32	freq_th;	// Offset frequency threshold
	__u8	shf_val;	// Spur shift value
	__u8	shf_dir;	// Spur shift direction
} SHF_TYPE;

static SHF_TYPE SHF_DVBT_TAB[] = {
	// { Freq(kHz),Offset(kHz), Val,	Dir,	type},
	{  64500, 500, 0x92, 0x07 },
	{ 191500, 300, 0xE2, 0x07 },
	{ 205500, 500, 0x2C, 0x04 },
	{ 212500, 500, 0x1E, 0x04 },
	{ 226500, 500, 0xD4, 0x07 },
	{  99143, 500, 0x9C, 0x07 },
	{ 173143, 500, 0xD4, 0x07 },
	{ 191143, 300, 0xD4, 0x07 },
	{ 207143, 500, 0xCE, 0x07 },
	{ 225143, 500, 0xCE, 0x07 },
	{ 243143, 500, 0xD4, 0x07 },
	{ 261143, 500, 0xD4, 0x07 },
	{ 291143, 500, 0xD4, 0x07 },
	{ 339143, 500, 0x2C, 0x04 },
	{ 117143, 500, 0x7A, 0x07 },
	{ 135143, 300, 0x7A, 0x07 },
	{ 153143, 500, 0x01, 0x07 }
};

static __u8 mx_address[MAX_TUNER] = { 0x62, 0x61 };

static void
mx_write(PT3_MX *mx, PT3_BUS *bus, __u8 *data, size_t size)
{
	pt3_tc_write_tuner_without_addr(mx->tc, bus, data, size);
}

static void
mx_read(PT3_MX *mx, PT3_BUS *bus, __u8 addr, __u8 *data)
{
	__u8 write[2];
	write[0] = 0xfb;
	write[1] = addr;

	pt3_tc_write_tuner_without_addr(mx->tc, bus, write, sizeof(write));
	pt3_tc_read_tuner_without_addr(mx->tc, bus, data, 1);
}

static void
mx_get_register(PT3_MX *mx, PT3_BUS *bus, __u8 addr, __u8 *data)
{
	mx_read(mx, bus, addr, data);
}

static void
mx_rftune(__u8 *data, __u32 *size, __u32 freq)
{
	__u32 dig_rf_freq ,temp ,frac_divider, khz, mhz, i;
	__u8 rf_data[] = {
		0x13, 0x00,		// abort tune
		0x3B, 0xC0,
		0x3B, 0x80,
		0x10, 0x95,		// BW
		0x1A, 0x05,
		0x61, 0x00,
		0x62, 0xA0,
		0x11, 0x40,		// 2 bytes to store RF frequency
		0x12, 0x0E,		// 2 bytes to store RF frequency
		0x13, 0x01		// start tune
	};

	dig_rf_freq = 0;
	temp = 0;
	frac_divider = 1000000;
	khz = 1000;
	mhz = 1000000;

	dig_rf_freq = freq / mhz;
	temp = freq % mhz;

	for (i = 0; i < 6; i++) {
		dig_rf_freq <<= 1;
		frac_divider /= 2;
		if (temp > frac_divider) {
			temp -= frac_divider;
			dig_rf_freq++;
		}
	}

	if (temp > 7812)
		dig_rf_freq++;

	rf_data[2 * (7) + 1] = (__u8)(dig_rf_freq);
	rf_data[2 * (8) + 1] = (__u8)(dig_rf_freq >> 8);

	for (i = 0; i < sizeof(SHF_DVBT_TAB)/sizeof(*SHF_DVBT_TAB); i++) {
		if ( (freq >= (SHF_DVBT_TAB[i].freq - SHF_DVBT_TAB[i].freq_th) * khz) &&
				(freq <= (SHF_DVBT_TAB[i].freq + SHF_DVBT_TAB[i].freq_th) * khz) ) {
			rf_data[2 * (5) + 1] = SHF_DVBT_TAB[i].shf_val;
			rf_data[2 * (6) + 1] = 0xa0 | SHF_DVBT_TAB[i].shf_dir;
			break;
		}
	}

	memcpy(data, rf_data, sizeof(rf_data));

	*size = sizeof(rf_data);
}

static void
mx_set_register(PT3_MX *mx, PT3_BUS *bus, __u8 addr, __u8 value)
{
	__u8 data[2];

	data[0] = addr;
	data[1] = value;
	
	mx_write(mx, bus, data, sizeof(data));
}

static void
mx_idac_setting(PT3_MX *mx, PT3_BUS *bus)
{
	__u8 data[] = {
		0x0D, 0x00,
		0x0C, 0x67,
		0x6F, 0x89,
		0x70, 0x0C,
		0x6F, 0x8A,
		0x70, 0x0E,
		0x6F, 0x8B,
		0x70, 0x10+12,
	};

	mx_write(mx, bus, data, sizeof(data));
}

static void
mx_tuner_rftune(PT3_MX *mx, PT3_BUS *bus, __u32 freq)
{
	__u8 data[100];
	__u32 size;

	size = 0;
	mx->freq = freq;

	mx_rftune(data, &size, freq);

	if (size != 20) {
		PT3_PRINTK(0, KERN_ERR, "fail mx_rftune size = %d\n", size);
		return;
	}

	mx_write(mx, bus, data, 14);

	schedule_timeout_interruptible(msecs_to_jiffies(1));	

	mx_write(mx, bus, data + 14, 6);

	schedule_timeout_interruptible(msecs_to_jiffies(1));	
	schedule_timeout_interruptible(msecs_to_jiffies(30));	

	mx_set_register(mx, bus, 0x1a, 0x0d);

	mx_idac_setting(mx, bus);
}

static void
mx_standby(PT3_MX *mx, PT3_BUS *bus)
{
	__u8 data[4];

	data[0] = 0x01;
	data[1] = 0x00;
	data[2] = 0x13;
	data[3] = 0x00;

	mx_write(mx, bus, data, sizeof(data));
}

static void
mx_wakeup(PT3_MX *mx, PT3_BUS *bus)
{
	__u8 data[2];

	data[0] = 0x01;
	data[1] = 0x01;

	mx_write(mx, bus, data, sizeof(data));

	mx_tuner_rftune(mx, bus, mx->freq);
}

static STATUS
mx_set_sleep_mode(PT3_MX *mx, PT3_BUS *bus, int sleep)
{
	STATUS status;
	status = 0;

	if (sleep) {
		mx_standby(mx, bus);
	} else {
		mx_wakeup(mx, bus);
	}

	return status;
}

static __u8 FREQ_TABLE[][3] = {
	{   2,  0,   3 },
	{  12,  1,  22 },
	{  21,  0,  12 },
	{  62,  1,  63 },
	{ 112,  0,  62 }
};

void
pt3_mx_get_channel_frequency(PT3_MX *mx, __u32 channel, int *catv, __u32 *number, __u32 *freq)
{
	__u32 i;
	__s32 freq_offset = 0;

	if (12 <= channel)
		freq_offset += 2;
	if (17 <= channel)
		freq_offset -= 2;
	if (63 <= channel)
		freq_offset += 2;
	*freq = 93 + channel * 6 + freq_offset;

	for (i = 0; i < sizeof(FREQ_TABLE) / sizeof(*FREQ_TABLE); i++) {
		if (channel <= FREQ_TABLE[i][0]) {
			*catv = FREQ_TABLE[i][1] ? 1: 0;
			*number = channel + FREQ_TABLE[i][2] - FREQ_TABLE[i][0];
			break;
		}
	}
}

static void
mx_set_frequency(PT3_MX *mx, PT3_BUS *bus, __u32 freq)
{
	mx_tuner_rftune(mx, bus, freq);
}

static void
mx_rfsynth_lock_status(PT3_MX *mx, PT3_BUS *bus, int *locked)
{
	__u8 data;

	*locked = 0;

	mx_get_register(mx, bus, 0x16, &data);

	data &= 0x0c;
	if (data == 0x0c)
		*locked = 1;
}

static void
mx_refsynth_lock_status(PT3_MX *mx, PT3_BUS *bus, int *locked)
{
	__u8 data;

	*locked = 0;

	mx_get_register(mx, bus, 0x16, &data);

	data &= 0x03;
	if (data == 0x03)
		*locked = 1;
}

__u8
pt3_mx_address(__u32 index)
{
	return mx_address[index];
}

STATUS
pt3_mx_set_sleep(PT3_MX *mx, int sleep)
{
	STATUS status;

	if (sleep) {
		status = pt3_tc_set_agc_t(mx->tc, NULL, PT3_TC_AGC_MANUAL);
		if (status)
			return status;
		mx_set_sleep_mode(mx, NULL, sleep);
		pt3_tc_write_slptim(mx->tc, NULL, sleep);
	} else {
		pt3_tc_write_slptim(mx->tc, NULL, sleep);
		mx_set_sleep_mode(mx, NULL, sleep);
	}
	
	mx->sleep = sleep;

	return STATUS_OK;
}

STATUS
pt3_mx_get_locked1(PT3_MX *mx, PT3_BUS *bus, int *locked)
{
	mx_rfsynth_lock_status(mx, bus, locked);

	return STATUS_OK;
}

STATUS
pt3_mx_get_locked2(PT3_MX *mx, PT3_BUS *bus, int *locked)
{
	mx_refsynth_lock_status(mx, bus, locked);

	return STATUS_OK;
}

static __u32 REAL_TABLE[112] = {
	0x58d3f49,0x5e8ccc9,0x6445a49,0x69fe7c9,0x6fb7549,
	0x75702c9,0x7b29049,0x80e1dc9,0x869ab49,0x8c538c9,
	0x920c649,0x97c53c9,0x9f665c9,0xa51f349,0xaad80c9,
	0xb090e49,0xb649bc9,0xba1a4c9,0xbfd3249,0xc58bfc9,
	0xcb44d49,0xd0fdac9,0xd6b6849,0xdc6f5c9,0xe228349,
	0xe7e10c9,0xed99e49,0xf352bc9,0xf90b949,0xfec46c9,
	0x1047d449,0x10a361c9,0x10feef49,0x115a7cc9,0x11b60a49,
	0x121197c9,0x126d2549,0x12c8b2c9,0x13244049,0x137fcdc9,
	0x13db5b49,0x1436e8c9,0x14927649,0x14ee03c9,0x15499149,
	0x15a51ec9,0x1600ac49,0x165c39c9,0x16b7c749,0x171354c9,
	0x176ee249,0x17ca6fc9,0x1825fd49,0x18818ac9,0x18dd1849,
	0x1938a5c9,0x19943349,0x19efc0c9,0x1a4b4e49,0x1aa6dbc9,
	0x1b026949,0x1b5df6c9,0x1bb98449,0x1c339649,0x1c8f23c9,
	0x1ceab149,0x1d463ec9,0x1da1cc49,0x1dfd59c9,0x1e58e749,
	0x1eb474c9,0x1f100249,0x1f6b8fc9,0x1fc71d49,0x2022aac9,
	0x207e3849,0x20d9c5c9,0x21355349,0x2190e0c9,0x21ec6e49,
	0x2247fbc9,0x22a38949,0x22ff16c9,0x235aa449,0x23b631c9,
	0x2411bf49,0x246d4cc9,0x24c8da49,0x252467c9,0x257ff549,
	0x25db82c9,0x26371049,0x26929dc9,0x26ee2b49,0x2749b8c9,
	0x27a54649,0x2800d3c9,0x285c6149,0x28b7eec9,0x29137c49,
	0x296f09c9,0x29ca9749,0x2a2624c9,0x2a81b249,0x2add3fc9,
	0x2b38cd49,0x2b945ac9,0x2befe849,0x2c4b75c9,0x2ca70349,
	0x2d0290c9,0x2d5e1e49,
};

STATUS
pt3_mx_set_frequency(PT3_MX *mx, __u32 channel, __s32 offset)
{
	STATUS status;
	int catv, locked1, locked2;
	__u32 number, freq;
	__u32 real_freq;
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,0,0)
	struct timeval begin, now;
#else
	struct timespec64 begin, now;
#endif

	status = pt3_tc_set_agc_t(mx->tc, NULL, PT3_TC_AGC_MANUAL);
	if (status)
		return status;
	
	pt3_mx_get_channel_frequency(mx, channel, &catv, &number, &freq);

	//real_freq = (7 * freq + 1 + offset) * 1000000.0 /7.0;
	real_freq = REAL_TABLE[channel];

	mx_set_frequency(mx, NULL, real_freq);

#if LINUX_VERSION_CODE < KERNEL_VERSION(5,0,0)
	do_gettimeofday(&begin);
#else
	ktime_get_real_ts64(&begin);
#endif
	locked1 = locked2 = 0;
	while (1) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,0,0)
		do_gettimeofday(&now);
#else
		ktime_get_real_ts64(&now);
#endif
		pt3_mx_get_locked1(mx, NULL, &locked1);
		pt3_mx_get_locked2(mx, NULL, &locked2);

		if (locked1 && locked2)
			break;
		if (time_diff(&begin, &now) > 1000)
			break;

		schedule_timeout_interruptible(msecs_to_jiffies(1));	
	}
#if 0
	PT3_PRINTK(7, KERN_DEBUG, "mx_get_locked1 %d locked2 %d\n", locked1, locked2);
#endif
	if (!(locked1 && locked2))
		return STATUS_PLL_LOCK_TIMEOUT_ERROR;
	
	status = pt3_tc_set_agc_t(mx->tc, NULL, PT3_TC_AGC_AUTO);
	if (status)
		return status;

	return status;
}

PT3_MX *
create_pt3_mx(PT3_I2C *i2c, PT3_TC *tc)
{
	PT3_MX *mx;

	mx = NULL;

	mx = pt3_vzalloc(sizeof(PT3_MX));
	if (mx == NULL)
		goto fail;

	mx->i2c = i2c;
	mx->tc = tc;
	mx->sleep = 1;
	
	return mx;
fail:
	if (mx != NULL)
		vfree(mx);
	return NULL;
}

void
free_pt3_mx(PT3_MX *mx)
{
	vfree(mx);
}


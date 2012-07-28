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

#ifndef _H_PT3_QM
#define _H_PT3_QM

#include "pt3_bus.h"
#include "pt3_i2c.h"
#include "pt3_tc.h"

typedef struct __PT3_QM_PARAM {
	__u32 channel_freq;
	__u32 crystal_freq;
	int fast_search_mode;
	int standby;
	__u32 lpf_wait_time;
	__u32 fast_search_wait_time;
	__u32 normal_search_wait_time;
} PT3_QM_PARAM;

typedef struct __PT3_QM {
	PT3_I2C *i2c;
	PT3_TC *tc;
	PT3_QM_PARAM param;
	__u8 reg[0x20];
	int sleep;
	__u32 channel;
	__s32 offset;
} PT3_QM;

STATUS pt3_qm_set_sleep(PT3_QM *qm, int sleep);
__u8 pt3_qm_address(__u32 index);
STATUS pt3_qm_set_frequency(PT3_QM *qm, __u32 channel, __s32 offset);
void pt3_qm_get_channel_freq(__u32 channel, int *bs, __u32 *number, __u32 *freq);
void pt3_qm_dummy_reset(PT3_QM *qm, PT3_BUS *bus);
void pt3_qm_init_reg_param(PT3_QM *qm);
STATUS pt3_qm_init(PT3_QM *qm, PT3_BUS *bus);
PT3_QM * create_pt3_qm(PT3_I2C *i2c, PT3_TC *tc);
void free_pt3_qm(PT3_QM *qm);

#endif

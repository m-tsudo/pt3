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

#ifndef _H_PT3_TC
#define _H_PT3_TC

#include "pt3_com.h"
#include "pt3_bus.h"
#include "pt3_i2c.h"
#include "pt3_ioctl.h"

#define TC_THROUGH 0xfe

typedef struct _PT3_TC {
	PT3_I2C *i2c;
	__u8 tc_addr;
	__u8 tuner_addr;
	__u8 master_clock_freq;	// MHz
} PT3_TC;

typedef enum {
	PT3_TC_AGC_AUTO,
	PT3_TC_AGC_MANUAL,
} PT3_TC_AGC;

__u8 pt3_tc_address(__u32 pin, int isdb, __u32 index);
STATUS pt3_tc_write(PT3_TC *tc, PT3_BUS *bus, __u8 addr, const __u8 *data, __u32 size);
STATUS pt3_tc_write_tuner(PT3_TC *tc, PT3_BUS *bus, __u8 addr, const __u8 *data, __u32 size);
STATUS pt3_tc_write_tuner_without_addr(PT3_TC *tc, PT3_BUS *bus, const __u8 *data, __u32 size);
STATUS pt3_tc_read_tuner(PT3_TC *tc, PT3_BUS *bus, __u8 addr, __u8 *data, __u32 size);
STATUS pt3_tc_read_tuner_without_addr(PT3_TC *tc, PT3_BUS *bus, __u8 *data, __u32 size);
STATUS pt3_tc_init_s(PT3_TC *tc, PT3_BUS *bus);
STATUS pt3_tc_init_t(PT3_TC *tc, PT3_BUS *bus);
STATUS pt3_tc_set_powers(PT3_TC *tc, PT3_BUS *bus, int tuner, int amp);
__u32 pt3_tc_index(PT3_TC *tc);
STATUS pt3_tc_write_slptim(PT3_TC *tc, PT3_BUS *bus, int sleep);
STATUS pt3_tc_set_agc_s(PT3_TC *tc, PT3_BUS *bus, PT3_TC_AGC agc);
STATUS pt3_tc_set_agc_t(PT3_TC *tc, PT3_BUS *bus, PT3_TC_AGC agc);
STATUS pt3_tc_set_sleep_s(PT3_TC *tc, PT3_BUS *bus, int sleep);
STATUS pt3_tc_set_ts_pins_mode_s(PT3_TC *tc, PT3_BUS *bus, PT3_TS_PINS_MODE *mode);
STATUS pt3_tc_set_ts_pins_mode_t(PT3_TC *tc, PT3_BUS *bus, PT3_TS_PINS_MODE *mode);
STATUS pt3_tc_read_retryov_tmunvld_fulock(PT3_TC *tc, PT3_BUS *bus, int *retryov, int *tmunvld, int *fulock);
STATUS pt3_tc_read_tmcc_s(PT3_TC *tc, PT3_BUS *bus, TMCC_S *tmcc);
STATUS pt3_tc_read_tmcc_t(PT3_TC *tc, PT3_BUS *bus, TMCC_T *tmcc);
STATUS pt3_tc_write_id_s(PT3_TC *tc, PT3_BUS *bus, __u16 id);
STATUS pt3_tc_read_id_s(PT3_TC *tc, PT3_BUS *bus, __u16 *id);
STATUS pt3_tc_read_agc_s(PT3_TC *tc, PT3_BUS *bus, __u8 *agc);
STATUS pt3_tc_read_ifagc_dt(PT3_TC *tc, PT3_BUS *bus, __u8 *ifagc_dt);
STATUS pt3_tc_read_cn_s(PT3_TC *tc, PT3_BUS *bus, __u32 *cn);
STATUS pt3_tc_read_cndat_t(PT3_TC *tc, PT3_BUS *bus, __u32 *cn);
PT3_TC * create_pt3_tc(PT3_I2C *i2c, __u8 tc_addr, __u8 qm_addr);
void free_pt3_tc(PT3_TC *tc);

#endif

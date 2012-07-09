#ifndef _H_PT3_QM
#define _H_PT3_QM

#include "pt3_i2c_bus.h"
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
	PT3_QM_PARAM param;
	__u8 reg[0x20];
} PT3_QM;

void pt3_qm_get_channel_freq(__u32 channel, int *bs, __u32 *number, __u32 *freq);
void pt3_qm_dummy_reset(PT3_I2C_BUS * bus, PT3_TC *tc, PT3_QM *qm);
void pt3_qm_init_reg_param(PT3_QM *qm);
int pt3_qm_init(PT3_I2C_BUS * bus, PT3_TC *tc, PT3_QM *qm);

#endif

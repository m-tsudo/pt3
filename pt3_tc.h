#ifndef _H_PT3_TC
#define _H_PT3_TC

#include "pt3_com.h"
#include "pt3_i2c_bus.h"

#define TC_THROUGH 0xfe

typedef struct _PT3_TC {
	PT3_I2C_BUS *bus;
	__u8 tc_addr;
	__u8 tuner_addr;
	double master_clock_freq;	// MHz
} PT3_TC;

typedef enum {
	PT3_TC_AGC_AUTO,
	PT3_TC_AGC_MANUAL,
} PT3_TC_AGC;

__u8 pt3_tc_address(__u32 pin, int isdb, __u32 index);
int pt3_tc_write(PT3_TC *tc, __u8 addr, const __u8 *data, __u32 size);
int pt3_tc_read_tuner(PT3_TC *tc, __u8 addr, __u8 *data, __u32 size);
int pt3_tc_write_tuner(PT3_TC *tc, __u8 addr, const __u8 *data, __u32 size);
STATUS pt3_tc_write_tuner_without_addr(PT3_TC *tc, const __u8 *data, __u32 size);
int pt3_tc_init_s(PT3_TC *tc);
int pt3_tc_init_t(PT3_TC *tc);
int pt3_tc_set_powers(PT3_TC *tc, int tuner, int amp);
__u32 pt3_tc_index(PT3_TC *tc);
STATUS pt3_tc_write_slptim(PT3_TC *tc, int sleep);
STATUS pt3_tc_set_agc_s(PT3_TC *tc, PT3_TC_AGC agc);
STATUS pt3_tc_set_agc_t(PT3_TC *tc, PT3_TC_AGC agc);
STATUS pt3_tc_set_sleep_s(PT3_TC *tc, int sleep);
STATUS pt3_tc_set_ts_pins_mode_s(PT3_TC *tc, PT3_TS_PINS_MODE *mode);
STATUS pt3_tc_set_ts_pins_mode_t(PT3_TC *tc, PT3_TS_PINS_MODE *mode);
PT3_TC * create_pt3_tc(PT3_I2C_BUS *bus, __u8 tc_addr, __u8 qm_addr);
void free_pt3_tc(PT3_TC *tc);

#endif

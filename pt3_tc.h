#ifndef _H_PT3_TC
#define _H_PT3_TC

#include "pt3_i2c_bus.h"

#define TC_THROUGH 0xfe

enum {
	PT3_ISDB_S,
	PT3_ISDB_T,
};

typedef struct _PT3_TC {
	__u8 tc_addr;
	__u8 qm_addr;
	double master_clock_freq;	// MHz
} PT3_TC;

__u8 pt3_tc_address(__u32 pin, int isdb, __u32 index);
int pt3_tc_write(PT3_I2C_BUS *bus, PT3_TC *tc, __u8 addr, const __u8 *data, __u32 size);
int pt3_tc_read_tuner(PT3_I2C_BUS *bus, PT3_TC *tc, __u8 addr, __u8 *data, __u32 size);
int pt3_tc_write_tuner(PT3_I2C_BUS *bus, PT3_TC *tc, __u8 addr, const __u8 *data, __u32 size);
int pt3_tc_init_s(PT3_I2C_BUS *bus, PT3_TC *tc);
int pt3_tc_init_t(PT3_I2C_BUS *bus, PT3_TC *tc);

#endif

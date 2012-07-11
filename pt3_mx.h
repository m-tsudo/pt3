#ifndef _H_PT3_MX
#define _H_PT3_MX

#include "pt3_i2c_bus.h"
#include "pt3_tc.h"

typedef struct _PT3_MX {
	PT3_I2C_BUS *bus;
	PT3_TC *tc;
	__u32 freq;
	int sleep;
	__u32 channel;
	__s32 offset;
} PT3_MX;

__u8 pt3_mx_address(__u32 index);
STATUS pt3_mx_set_sleep(PT3_MX *mx, int sleep);
PT3_MX * create_pt3_mx(PT3_I2C_BUS *bus, PT3_TC *tc);
void free_pt3_mx(PT3_MX *mx);

#endif

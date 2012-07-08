#ifndef		__PT3_I2C_H__
#define		__PT3_I2C_H__
#include <linux/mutex.h>

#define PT3_I2C_INSTRUCTION_MAX 4096

typedef struct __PT3_I2C_CONTROL {
	__u8		*instructions;
	__u32		count;
	struct mutex	lock ;
} PT3_I2C_CONTROL;

#endif

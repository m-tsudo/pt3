/***************************************************************************/
/* I2C¾ðÊóºîÀ®                                                             */
/***************************************************************************/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/version.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,4,0)
#include <asm/system.h>
#endif
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>

#include	"pt3_com.h"
#include	"pt3_i2c.h"
#include	"pt3_pci.h"
#include	"pt3_tuner.h"

#define I2C_OFFSET1	(4096 + (   0));
#define I2C_OFFSET2	(4096 + (2042));

enum {
	L,
	H,
};

enum {
	I_END,
	I_ADDRESS,
	I_CLOCK_L,
	I_CLOCK_H,
	I_DATA_L,
	I_DATA_H,
	I_RESET,
	I_SLEEP,
	I_DATA_L_NOP = 0x08,
	I_DATA_H_NOP = 0x0C,
	I_DATA_H_READ = 0x0D,
	I_DATA_H_ACK0 = 0x0E,
	I_DATA_H_ACK1 = 0x0F,
};

static __u8 tmp_instruction;
static int
add_instruction(PT3_I2C_CONTROL *ctl, __u32 instruction)
{
	if (ctl->count >= PT3_I2C_INSTRUCTION_MAX)
		return -1;

	if ((ctl->count % 2) == 0)
		tmp_instruction = instruction;
	else {
		tmp_instruction |= (instruction << 4);
		memcpy(ctl->instructions + ((ctl->count - 1) / 2),
			&tmp_instruction, sizeof(tmp_instruction));
	}
	ctl->count++;
	return 0;
}

static void
set_read_address(PT3_I2C_CONTROL *ctl, __u32 addr)
{
	add_instruction(ctl, I_ADDRESS);
	add_instruction(ctl, BIT_SHIFT_MASK(addr, 4*(0), 4));
	add_instruction(ctl, BIT_SHIFT_MASK(addr, 4*(1), 4));
	add_instruction(ctl, BIT_SHIFT_MASK(addr, 4*(2), 3));
}

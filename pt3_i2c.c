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

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,4,0)
#include <asm/system.h>
#endif
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>

#include "pt3_com.h"
#include "pt3_pci.h"
#include "pt3_i2c.h"

#define DATA_OFFSET 0

static void
wait(PT3_I2C *i2c, __u32 *data)
{
	__u32 val;
	
	while (1) {
		val = readl(i2c->bar[0].regs + REGS_I2C_R);
		if (!BIT_SHIFT_MASK(val, 0, 1))
			break;
		schedule_timeout_interruptible(msecs_to_jiffies(1));	
	}

	if (data != NULL)
		*data = val;
}

static STATUS
run_code(PT3_I2C *i2c, __u32 start_addr, __u32 *ack)
{
	__u32 data, a;

	wait(i2c, &data);

	if (start_addr >= (1 << 13))
		printk(KERN_DEBUG "start address is over.");
	
	writel(1 << 16 | start_addr, i2c->bar[0].regs + REGS_I2C_W);

	wait(i2c, &data);

	printk(KERN_DEBUG "i2c status 0x%lx", data);

	a = BIT_SHIFT_MASK(data, 1, 2);
	if (ack != NULL)
		*ack = a;

	return BIT_SHIFT_MASK(data, 1, 2) ? STATUS_I2C_ERROR : STATUS_OK;
}

void
pt3_i2c_copy(PT3_I2C *i2c, PT3_BUS *bus)
{
	void __iomem *dst;
	__u8 *src;

	src = &bus->insts[0];
	dst = i2c->bar[1].regs + DATA_OFFSET + (bus->inst_addr / 2);

#if 0
	printk(KERN_DEBUG "PT3 : i2c_copy. base=%p dst=%p src=%p size=%d",
						i2c->bar[1].regs, dst, src, bus->inst_pos);
#endif
	memcpy(dst, src, bus->inst_pos);
}

STATUS
pt3_i2c_run(PT3_I2C *i2c, PT3_BUS *bus, __u32 *ack, int copy)
{
	STATUS status;
	__u32 rsize, i;

	mutex_lock(&i2c->lock);

	if (copy) {
		pt3_i2c_copy(i2c, bus);
	}

	status = run_code(i2c, bus->inst_addr, ack);

	rsize = bus->read_addr;

	for (i = 0; i < rsize; i++) {
		pt3_bus_push_read_data(bus, readl(i2c->bar[2].regs + DATA_OFFSET));
	}

	mutex_unlock(&i2c->lock);

	return status;
}

int
pt3_i2c_is_clean(PT3_I2C *i2c)
{
	__u32 val;

	val = readl(i2c->bar[0].regs + REGS_I2C_R);

	return BIT_SHIFT_MASK(val, 3, 1);
}

void
pt3_i2c_reset(PT3_I2C *i2c)
{
	writel(1 << 17, i2c->bar[0].regs + REGS_I2C_W);
}

PT3_I2C *
create_pt3_i2c(BAR *bar)
{
	PT3_I2C *i2c;

	i2c = vzalloc(sizeof(PT3_I2C));
	if (i2c == NULL)
		goto fail;

	mutex_init(&i2c->lock);
	i2c->bar = bar;

	return i2c;
fail:
	if (i2c != NULL)
		free_pt3_i2c(i2c);
	return NULL;
}

void
free_pt3_i2c(PT3_I2C *i2c)
{
	vfree(i2c);
}

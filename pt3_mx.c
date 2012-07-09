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
#include "pt3_mx.h"

static __u8 mx_address[MAX_TUNER] = { 0x62, 0x61 };

__u8
pt3_mx_address(__u32 index)
{
	return mx_address[index];
}

PT3_MX *
create_pt3_mx()
{
	PT3_MX *mx;

	mx = NULL;

	mx = vzalloc(sizeof(PT3_MX));
	if (mx == NULL)
		goto fail;
	
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


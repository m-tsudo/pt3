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

#define DRV_NAME "PT3-pci"
#include "version.h"

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/version.h>
#include <linux/mutex.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,4,0)
#include <asm/system.h>
#endif
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,11)
typedef struct pm_message {
        int event;
} pm_message_t;
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,23)
#include <linux/freezer.h>
#else
#define set_freezable()
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
#include <linux/sched.h>
#endif
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,38)
#include <linux/smp_lock.h>
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,8,0)
#define __devinitdata
#define __devinit
#define __devexit
#define __devexit_p
#endif
#include <linux/kthread.h>
#include <linux/dma-mapping.h>

#include <linux/fs.h>
#include <linux/cdev.h>

#include <linux/ioctl.h>

#include	"pt3_com.h"
#include	"pt3_ioctl.h"
#include	"pt3_pci.h"
#include	"pt3_bus.h"
#include	"pt3_i2c.h"
#include	"pt3_tc.h"
#include	"pt3_qm.h"
#include	"pt3_mx.h"
#include	"pt3_dma.h"

#ifndef pt3_vzalloc
void *
pt3_vzalloc(unsigned long size)
{
	void *p = vmalloc(size);
	if (p)
		memset(p, 0, size);
	return p;
}
#endif

char pt3_driver_name[] = DRV_NAME;

/* These identify the driver base version and may not be removed. */
static char version[] __devinitdata =
DRV_NAME ".c: " DRV_VERSION " " DRV_RELDATE " \n";

MODULE_AUTHOR("anyone");
#define	DRIVER_DESC		"PCI earthsoft PT3 driver"
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

int debug = 0;				/* 1 normal messages, 0 quiet .. 7 verbose. */
static int lnb = 0;			/* LNB OFF:0 +11V:1 +15V:2 */

module_param(debug, int, S_IRUGO | S_IWUSR);
module_param(lnb, int, 0);
MODULE_PARM_DESC(debug, "debug level (0-7)");
MODULE_PARM_DESC(lnb, "LNB level (0:OFF 1:+11V 2:+15V)");

#define VENDOR_ALTERA 0x1172
#define PCI_PT3_ID 0x4c15

static struct pci_device_id pt3_pci_tbl[] = {
	{ VENDOR_ALTERA, PCI_PT3_ID, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ 0, }
};
MODULE_DEVICE_TABLE(pci, pt3_pci_tbl);
#define		DEV_NAME	"pt3video"
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,39)
static	DEFINE_MUTEX(pt3_biglock);
#endif

#define		MAX_PCI_DEVICE		128		// 最大64枚

typedef struct _PT3_VERSION {
	__u8		ptn;
	__u8 		regs;
	__u8		fpga;
} PT3_VERSION;

typedef struct _PT3_SYSTEM {
	__u8		dma_descriptor_page_size;
	__u8		can_transport_ts;
} PT3_SYSTEM;

typedef struct _PT3_TUNER {
	int tuner_no;
	PT3_TC *tc_s;
	PT3_TC *tc_t;
	PT3_QM *qm;
	PT3_MX *mx;
} PT3_TUNER;

typedef struct _PT3_CHANNEL PT3_CHANNEL;

typedef	struct	_pt3_device{
	int bars;
	__u8 __iomem* hw_addr[2];
	struct mutex		lock ;
	dev_t			dev ;
	int			card_number;
	__u32			base_minor ;
	struct	cdev	cdev[MAX_CHANNEL];
	PT3_VERSION		version;
	PT3_SYSTEM		system;
	PT3_I2C			*i2c;
	PT3_TUNER       tuner[MAX_TUNER];
	PT3_CHANNEL		*channel[MAX_CHANNEL];
} PT3_DEVICE;

struct _PT3_CHANNEL {
	__u32			valid ;
	__u32			minor;
	PT3_TUNER		*tuner;
	int				type ;
	struct mutex	lock ;
	PT3_DEVICE		*ptr ;
	PT3_I2C			*i2c;
	PT3_DMA			*dma;
};

static int real_channel[MAX_CHANNEL] = {0, 1, 2, 3};
static int channel_type[MAX_CHANNEL] = {PT3_ISDB_S, PT3_ISDB_S,
										PT3_ISDB_T, PT3_ISDB_T};

static	PT3_DEVICE	*device[MAX_PCI_DEVICE];
static struct class	*pt3video_class;

#define		DRIVERNAME	"pt3video"

static int
check_fpga_version(PT3_DEVICE *dev_conf)
{
	__u32	val;
	
	val = readl(dev_conf->hw_addr[0] + REGS_VERSION);

	dev_conf->version.ptn = ((val >> 24) & 0xFF);
	dev_conf->version.regs = ((val >> 16) & 0xFF);
	dev_conf->version.fpga = ((val >> 8) & 0xFF);
	if (dev_conf->version.ptn != 3) {
		PT3_PRINTK(0, KERN_ERR, "PTn needs 3.\n");
		return -1;
	}
	PT3_PRINTK(7, KERN_INFO, "Check PTn is passed. n=%d\n", dev_conf->version.ptn);

	if (dev_conf->version.fpga != 0x04) {
		PT3_PRINTK(0, KERN_ERR, "this FPGA version is not supported. version=0x%x\n",
				dev_conf->version.fpga);
		return -1;
	}

	val = readl(dev_conf->hw_addr[0] + REGS_SYSTEM_R);
	dev_conf->system.can_transport_ts = ((val >> 5) & 0x01);
	dev_conf->system.dma_descriptor_page_size = (val & 0x1F);

	return 0;
}

#if 0
static int
get_tuner_status(int isdb, PT3_TUNER *tuner)
{
	int sleep;

	sleep = 1;
	switch (isdb) {
	case PT3_ISDB_S :
		sleep = tuner->qm->sleep;
		break;
	case PT3_ISDB_T :
		sleep = tuner->mx->sleep;
		break;
	}
	return sleep ? 1 : 0;
}
#endif

static STATUS
set_id_s(PT3_TUNER *tuner, __u32 id)
{
	STATUS status;

	status = pt3_tc_write_id_s(tuner->tc_s, NULL, (__u16)id);

	return status;
}

static STATUS
get_id_s(PT3_TUNER *tuner, __u32 *id)
{
	STATUS status;
	__u16 short_id;

	if (unlikely(id == NULL))
		return STATUS_INVALID_PARAM_ERROR;
	
	status = pt3_tc_read_id_s(tuner->tc_s, NULL, &short_id);
	if (status)
		return status;

	*id = short_id;

	return status;
}

static STATUS
get_tmcc_s(PT3_TUNER *tuner, TMCC_S *tmcc)
{
	if (unlikely(tmcc == NULL))
		return STATUS_INVALID_PARAM_ERROR;

	return pt3_tc_read_tmcc_s(tuner->tc_s, NULL, tmcc);
}

static STATUS
get_tmcc_t(PT3_TUNER *tuner, TMCC_T *tmcc)
{
	int b, retryov, tmunvld, fulock;

	if (unlikely(tmcc == NULL))
		return STATUS_INVALID_PARAM_ERROR;

	b = 0;
	while (1) {
		pt3_tc_read_retryov_tmunvld_fulock(tuner->tc_t, NULL, &retryov, &tmunvld, &fulock);
		if (!fulock) {
			b = 1;
			break;
		} else {
			if (retryov)
				break;
		}
		schedule_timeout_interruptible(msecs_to_jiffies(1));	
	}

	if (likely(b))
		pt3_tc_read_tmcc_t(tuner->tc_t, NULL, tmcc);

	return b ? STATUS_OK : STATUS_GENERAL_ERROR;
}

static __u32 LNB_SETTINGS[] = {
	(1 << 3 | 0 << 1) | (1 << 2 | 0 << 9),	// 0v
	(1 << 3 | 0 << 1) | (1 << 2 | 1 << 0),	// 12v
	(1 << 3 | 1 << 1) | (1 << 2 | 1 << 0),	// 15v
};

static STATUS
set_lnb(PT3_DEVICE *dev_conf, int lnb)
{
	if (unlikely(lnb < 0 || 2 < lnb))
		return STATUS_INVALID_PARAM_ERROR;
	writel(LNB_SETTINGS[lnb], dev_conf->hw_addr[0] + REGS_SYSTEM_W);
	return STATUS_OK;
}

static STATUS
set_frequency(int isdb, PT3_TUNER *tuner, __u32 channel, __s32 offset)
{
	STATUS status;

	PT3_PRINTK(7, KERN_DEBUG, "set_freq isdb=%d tuner_no=%d channel=%d offset=%d\n",
			isdb, tuner->tuner_no, channel, offset);

	switch (isdb) {
	case PT3_ISDB_S :
		status = pt3_qm_set_frequency(tuner->qm, channel, 0);
		break;
	case PT3_ISDB_T :
		status = pt3_mx_set_frequency(tuner->mx, channel, offset);
		break;
	default :
		status = STATUS_INVALID_PARAM_ERROR;
	}

	return status;
}

static STATUS
set_tuner_sleep(int isdb, PT3_TUNER *tuner, int sleep)
{
	STATUS status;

	switch (isdb) {
	case PT3_ISDB_S :
		PT3_PRINTK(1, KERN_INFO, "TUNER %p ISDB_S %s\n", tuner,
					(sleep) ? "Sleep" : "Wakeup");
		status = pt3_qm_set_sleep(tuner->qm, sleep);
		break;
	case PT3_ISDB_T :
		PT3_PRINTK(1, KERN_INFO, "TUNER %p ISDB_T %s\n", tuner,
					(sleep) ? "Sleep" : "Wakeup");
		status = pt3_mx_set_sleep(tuner->mx, sleep);
		break;
	default :
		status = STATUS_INVALID_PARAM_ERROR;
	}
	schedule_timeout_interruptible(msecs_to_jiffies(50));

	return status;
}

static STATUS
init_tuner(PT3_I2C *i2c, PT3_TUNER *tuner)
{
	STATUS status;
	PT3_BUS *bus;

	pt3_qm_init_reg_param(tuner->qm);
	{
		bus = create_pt3_bus();
		if (bus == NULL)
			return STATUS_OUT_OF_MEMORY_ERROR;
		pt3_qm_dummy_reset(tuner->qm, bus);
		pt3_bus_end(bus);
		status = pt3_i2c_run(i2c, bus, NULL, 1);
		free_pt3_bus(bus);
		if (status) {
			PT3_PRINTK(7, KERN_DEBUG, "fail init_tuner dummy reset. status=0x%x\n", status);
			return status;
		}
	}

	{
		bus = create_pt3_bus();
		if (bus == NULL)
			return STATUS_OUT_OF_MEMORY_ERROR;
		status = pt3_qm_init(tuner->qm, bus);
		if (status) {
			free_pt3_bus(bus);
			return status;
		}
		pt3_bus_end(bus);
		status = pt3_i2c_run(i2c, bus, NULL, 1);
		free_pt3_bus(bus);
		if (status) {
			PT3_PRINTK(7, KERN_DEBUG, "fail init_tuner qm init. status=0x%x\n", status);
			return status;
		}
	}

	return status;
}

static STATUS
tuner_power_on(PT3_DEVICE *dev_conf, PT3_BUS *bus)
{
	STATUS status;
	int i, j;
	PT3_TS_PINS_MODE pins;

	PT3_TUNER *tuner;

	for (i = 0; i < MAX_TUNER; i++) {
		tuner = &dev_conf->tuner[i];
		status = pt3_tc_init_s(tuner->tc_s, NULL);
		if (status)
			PT3_PRINTK(1, KERN_INFO, "tc_init_s[%d] status=0x%x\n", i, status);
	}
	for (i = 0; i < MAX_TUNER; i++) {
		tuner = &dev_conf->tuner[i];
		status = pt3_tc_init_t(tuner->tc_t, NULL);
		if (status)
			PT3_PRINTK(1, KERN_INFO, "tc_init_t[%d] status=0x%x\n", i, status);
	}

	tuner = &dev_conf->tuner[1];
	status = pt3_tc_set_powers(tuner->tc_t, NULL, 1, 0);
	if (status) {
		PT3_PRINTK(7, KERN_DEBUG, "fail set powers.\n");
		goto last;
	}

	pins.clock_data = PT3_TS_PIN_MODE_NORMAL;
	pins.byte = PT3_TS_PIN_MODE_NORMAL;
	pins.valid = PT3_TS_PIN_MODE_NORMAL;

	for (i = 0; i < MAX_TUNER; i++) {
		tuner = &dev_conf->tuner[i];
		status = pt3_tc_set_ts_pins_mode_s(tuner->tc_s, NULL, &pins);
		if (status)
			PT3_PRINTK(1, KERN_INFO, "fail set ts pins mode s [%d] status=0x%x\n", i, status);
	}
	for (i = 0; i < MAX_TUNER; i++) {
		tuner = &dev_conf->tuner[i];
		status = pt3_tc_set_ts_pins_mode_t(tuner->tc_t, NULL, &pins);
		if (status)
			PT3_PRINTK(1, KERN_INFO, "fail set ts pins mode t [%d] status=0x%x\n", i, status);
	}

	schedule_timeout_interruptible(msecs_to_jiffies(1));	

	for (i = 0; i < MAX_TUNER; i++) {
		for (j = 0; j < 10; j++) {
			if (j != 0)
				PT3_PRINTK(0, KERN_INFO, "retry init_tuner\n");
			status = init_tuner(dev_conf->i2c, &dev_conf->tuner[i]);
			if (!status)
				break;
			schedule_timeout_interruptible(msecs_to_jiffies(1));
		}
		if (status) {
			PT3_PRINTK(7, KERN_INFO, "fail init_tuner %d status=0x%x\n", i, status);
			goto last;
		}
	}

	if (unlikely(bus->inst_addr < 4096))
		pt3_i2c_copy(dev_conf->i2c, bus);

	bus->inst_addr = PT3_BUS_INST_ADDR1;
	status = pt3_i2c_run(dev_conf->i2c, bus, NULL, 0);
	if (status) {
		PT3_PRINTK(7, KERN_INFO, "failed inst_addr=0x%x status=0x%x\n",
				PT3_BUS_INST_ADDR1, status);
		goto last;
	}

	tuner = &dev_conf->tuner[1];
	status = pt3_tc_set_powers(tuner->tc_t, NULL, 1, 1);
	if (status) {
		 PT3_PRINTK(7, KERN_INFO, "fail tc_set_powers,\n");
		goto last;
	}

last:
	return status;
}

static STATUS
init_all_tuner(PT3_DEVICE *dev_conf)
{
	STATUS status;
	int i, j, channel;
	PT3_I2C *i2c = dev_conf->i2c;
	PT3_BUS *bus = create_pt3_bus();

	if (bus == NULL)
		return STATUS_OUT_OF_MEMORY_ERROR;

	pt3_bus_end(bus);
	bus->inst_addr = PT3_BUS_INST_ADDR0;

	if (!pt3_i2c_is_clean(i2c)) {
		PT3_PRINTK(0, KERN_INFO, "cleanup I2C bus.\n");
		status = pt3_i2c_run(i2c, bus, NULL, 0);
		if (status)
			goto last;
		schedule_timeout_interruptible(msecs_to_jiffies(10));
	}

	status = tuner_power_on(dev_conf, bus);
	if (status)
		goto last;
	PT3_PRINTK(7, KERN_DEBUG, "tuner_power_on\n");
	
	for (i = 0; i < MAX_TUNER; i++) {
		for (j = 0; j < PT3_ISDB_MAX; j++) {
			if (j == PT3_ISDB_S)
				channel = 0;
			else
				channel = (i == 0) ? 70 : 71;
			status = set_tuner_sleep(j, &dev_conf->tuner[i], 0);
			if (status)
				goto last;
			status = set_frequency(j, &dev_conf->tuner[i], channel, 0);
			if (status) {
				PT3_PRINTK(0, KERN_DEBUG, "fail set_frequency. status=0x%x\n", status);
			}
			status = set_tuner_sleep(j, &dev_conf->tuner[i], 1);
			if (status)
				goto last;
		}
	}
last:
	free_pt3_bus(bus);
	return status;
}

static STATUS
get_cn_agc(PT3_CHANNEL *channel, __u32 *cn, __u32 *curr_agc, __u32 *max_agc)
{
	STATUS status;
	PT3_TUNER *tuner = channel->tuner;
	__u8 byte_agc;

	switch (channel->type) {
	case PT3_ISDB_S:
		status = pt3_tc_read_cn_s(tuner->tc_s, NULL, cn);
		if (status)
			return status;
		status = pt3_tc_read_agc_s(tuner->tc_s, NULL, &byte_agc);
		if (status)
			return status;
		*curr_agc = byte_agc;
		*max_agc = 127;
		break;
	case PT3_ISDB_T:
		status = pt3_tc_read_cndat_t(tuner->tc_t, NULL, cn);
		if (status)
			return status;
		status = pt3_tc_read_ifagc_dt(tuner->tc_t, NULL, &byte_agc);
		if (status)
			return status;
		*curr_agc = byte_agc;
		*max_agc = 255;
		break;
	default:
		*cn = 0;
		*curr_agc = 0;
		*max_agc = 0;
	}
	PT3_PRINTK(7, KERN_INFO, "cn=0x%x\n", *cn);
	PT3_PRINTK(7, KERN_INFO, "agc=0x%x\n", *curr_agc);

	return STATUS_OK;
}

static STATUS
SetChannel(PT3_CHANNEL *channel, FREQUENCY *freq)
{
	TMCC_S tmcc_s;
	TMCC_T tmcc_t;
	STATUS status;
	__u32 i, tsid;

	status = set_frequency(channel->type, channel->tuner, freq->frequencyno, freq->slot);
	if (status)
		return status;

	switch (channel->type) {
	case PT3_ISDB_S :
		for (i = 0; i < 1000; i++) {
			schedule_timeout_interruptible(msecs_to_jiffies(1));
			status = get_tmcc_s(channel->tuner, &tmcc_s);
			if (!status)
				break;
		}
		if (status) {
			PT3_PRINTK(1, KERN_ERR, "fail get_tmcc_s status=0x%x\n", status);
			return status;
		}
		PT3_PRINTK(7, KERN_DEBUG, "tmcc_s.id = 0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x\n",
				tmcc_s.id[0], tmcc_s.id[1], tmcc_s.id[2], tmcc_s.id[3],
				tmcc_s.id[4], tmcc_s.id[5], tmcc_s.id[6], tmcc_s.id[7]);
		status = set_id_s(channel->tuner, tmcc_s.id[freq->slot]);
		if (status) {
			PT3_PRINTK(1, KERN_ERR, "fail set_tmcc_s status=0x%x\n", status);
			return status;
		}
		for (i = 0; i < 1000; i++) {
			status = get_id_s(channel->tuner, &tsid);
			if (status) {
				PT3_PRINTK(1, KERN_ERR, "fail get_id_s status=0x%x\n", status);
				return status;
			}
			PT3_PRINTK(7, KERN_DEBUG, "tsid=0x%x\n", tsid);
			if ((tsid & 0xffff) == tmcc_s.id[freq->slot]) {
				// wait for fill buffer
				schedule_timeout_interruptible(msecs_to_jiffies(100));
				// reset_error_count
				pt3_dma_set_test_mode(channel->dma, 0, 0, 0, 1);
				return STATUS_OK;
			}
			schedule_timeout_interruptible(msecs_to_jiffies(2));
		}
		break;
	case PT3_ISDB_T :
		for (i = 0; i < 1000; i++) {
			status = get_tmcc_t(channel->tuner, &tmcc_t);
			if (!status)
				break;
			schedule_timeout_interruptible(msecs_to_jiffies(2));
		}
		if (status) {
			PT3_PRINTK(1, KERN_ERR, "fail get_tmcc_t status=0x%x\n", status);
			return status;
		}
		// wait for fill buffer
		schedule_timeout_interruptible(msecs_to_jiffies(200));
		// reset_error_count
		pt3_dma_set_test_mode(channel->dma, 0, 0, 0, 1);
		return status;
		break;
	}

	return STATUS_INVALID_PARAM_ERROR;
}

static int
pt3_open(struct inode *inode, struct file *file)
{
	int major = imajor(inode);
	int minor = iminor(inode);
	int lp, lp2;
	PT3_CHANNEL *channel;

	for (lp = 0; lp < MAX_PCI_DEVICE; lp++) {
		if (device[lp] == NULL) {
			PT3_PRINTK(1, KERN_DEBUG, "device is not exists\n");
			return -EIO;
		}

		if (MAJOR(device[lp]->dev) == major &&
			device[lp]->base_minor <= minor &&
			device[lp]->base_minor + MAX_CHANNEL > minor) {

			mutex_lock(&device[lp]->lock);
			for (lp2 = 0; lp2 < MAX_CHANNEL; lp2++) {
				channel = device[lp]->channel[lp2];
				if (channel->minor == minor) {
					if (channel->valid) {
						mutex_unlock(&device[lp]->lock);
						PT3_PRINTK(1, KERN_DEBUG, "device is already used.\n");
						return -EIO;
					}
					PT3_PRINTK(7, KERN_DEBUG, "selected tuner_no=%d type=%d\n",
							channel->tuner->tuner_no, channel->type);

					set_tuner_sleep(channel->type, channel->tuner, 0);
					schedule_timeout_interruptible(msecs_to_jiffies(100));
	
					channel->valid = 1;
					file->private_data = channel;

					mutex_unlock(&device[lp]->lock);

					return 0;
				}
			}
			mutex_unlock(&device[lp]->lock);
		}
	}

	return -EIO;
}

static int
pt3_release(struct inode *inode, struct file *file)
{
	PT3_CHANNEL *channel = file->private_data;

	mutex_lock(&channel->ptr->lock);
	channel->valid = 0;
	pt3_dma_set_enabled(channel->dma, 0);
	mutex_unlock(&channel->ptr->lock);

	if (debug > 0)
		PT3_PRINTK(0, KERN_INFO, "(%d:%d) error count %d\n",
				imajor(inode), iminor(inode),
				pt3_dma_get_ts_error_packet_count(channel->dma));
	set_tuner_sleep(channel->type, channel->tuner, 1);
	schedule_timeout_interruptible(msecs_to_jiffies(50));

	return 0;
}

static int dma_look_ready[MAX_CHANNEL] = {1, 1, 1, 1};
static ssize_t
pt3_read(struct file *file, char __user *buf, size_t cnt, loff_t * ppos)
{
	size_t rcnt;
	PT3_CHANNEL *channel;

	channel = file->private_data;

	rcnt = pt3_dma_copy(channel->dma, buf, cnt, ppos,
						dma_look_ready[channel->dma->real_index]);
	if (rcnt < 0) {
		PT3_PRINTK(1, KERN_INFO, "fail copy_to_user.\n");
		return -EFAULT;
	}

	return rcnt;
}

static int
count_used_bs_tuners(PT3_DEVICE *device)
{
	int count, i;
	count = 0;

	for (i = 0; i < MAX_CHANNEL; i++) {
		if (device && device->channel[i] &&
			device->channel[i]->type == PT3_ISDB_S &&
			device->channel[i]->valid)
			count++;
	}

	PT3_PRINTK(1, KERN_INFO, "used bs tuners on %p = %d\n", device, count);

	return count;
}

static long
pt3_do_ioctl(struct file  *file, unsigned int cmd, unsigned long arg0)
{
	PT3_CHANNEL *channel;
	FREQUENCY freq;
	int status, signal, curr_agc, max_agc, lnb_eff, lnb_usr;
	unsigned int count;
	unsigned long dummy;
	char *voltage[] = {"0V", "11V", "15V"};
	void *arg;

	channel = file->private_data;
	arg = (void *)arg0;

	switch (cmd) {
	case SET_CHANNEL:
		dummy = copy_from_user(&freq, arg, sizeof(FREQUENCY));
		status = SetChannel(channel, &freq);
		return -status;
	case START_REC:
		pt3_dma_set_enabled(channel->dma, 1);
		return 0;
	case STOP_REC:
		pt3_dma_set_enabled(channel->dma, 0);
		return 0;
	case GET_SIGNAL_STRENGTH:
		status = get_cn_agc(channel, &signal, &curr_agc, &max_agc);
		if (status)
			PT3_PRINTK(1, KERN_INFO, "fail get signal strength status=0x%x\n", status);
		dummy = copy_to_user(arg, &signal, sizeof(int));
		return 0;
	case LNB_ENABLE:
		count = count_used_bs_tuners(channel->ptr);
		if (count <= 1) {
			lnb_usr = (int)arg0;
			lnb_eff = lnb_usr ? lnb_usr : lnb;
			set_lnb(channel->ptr, lnb_eff);
			PT3_PRINTK(1, KERN_INFO, "LNB on %s\n", voltage[lnb_eff]);
		}
		return 0;
	case LNB_DISABLE:
		count = count_used_bs_tuners(channel->ptr);
		if (count <= 1) {
			set_lnb(channel->ptr, 0);
			PT3_PRINTK(1, KERN_INFO, "LNB off\n");
		}
		return 0;
	case GET_STATUS:
		status = (int)pt3_dma_get_status(channel->dma);
		dummy = copy_to_user(arg, &status, sizeof(int));
		return 0;
	case SET_TEST_MODE_ON:
		pt3_dma_build_page_descriptor(channel->dma, 0);
		PT3_PRINTK(7, KERN_DEBUG, "rebuild dma descriptor.\n");
		status = (1 + channel->dma->real_index) * 12345;
		pt3_dma_set_test_mode(channel->dma, 1, (__u16)status, 0, 0);
		PT3_PRINTK(7, KERN_DEBUG, "set test mode.\n");
		schedule_timeout_interruptible(msecs_to_jiffies(10));	
		pt3_dma_set_enabled(channel->dma, 1);
		schedule_timeout_interruptible(msecs_to_jiffies(10));	
		while (1) {
			status = (int)pt3_dma_get_status(channel->dma);
			PT3_PRINTK(7, KERN_DEBUG, "status = 0x%x\n", status);
			if ((status & 0x01) == 0)
				break;
			if ((status >> 24) != 0x47)
				break;
			schedule_timeout_interruptible(msecs_to_jiffies(1));	
		}
		dma_look_ready[channel->dma->real_index] = 0;
		return 0;
	case SET_TEST_MODE_OFF:
		dma_look_ready[channel->dma->real_index] = 1;
		pt3_dma_set_enabled(channel->dma, 0);
		pt3_dma_set_test_mode(channel->dma, 0, 0, 0, 1);
		pt3_dma_build_page_descriptor(channel->dma, 1);
		return 0;
	case GET_TS_ERROR_PACKET_COUNT:
		count = (int)pt3_dma_get_ts_error_packet_count(channel->dma);
		dummy = copy_to_user(arg, &count, sizeof(unsigned int));
		return 0;
	}
	return -EINVAL;
}

static long
pt3_unlocked_ioctl(struct file  *file, unsigned int cmd, unsigned long arg0)
{
	long ret;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,39)
	if(mutex_lock_interruptible(&pt3_biglock))
		return -EINTR ;
#elif LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,38)
	lock_kernel();
#endif

	ret = pt3_do_ioctl(file, cmd, arg0);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,39)
	mutex_unlock(&pt3_biglock);
#elif LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,38)
	unlock_kernel();
#endif

	return ret;
}

static long
pt3_compat_ioctl(struct file  *file, unsigned int cmd, unsigned long arg0)
{
	long ret;
	/* should do 32bit <-> 64bit conversion here? --yaz */
	ret = pt3_unlocked_ioctl(file, cmd, arg0);

	return ret;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36)
static int
pt3_ioctl(struct inode *inode, struct file  *file, unsigned int cmd, unsigned long arg0)
{
	int ret;
	ret = (int)pt3_do_ioctl(file, cmd, arg0);
	return ret;
}
#endif

/*
*/
static const struct file_operations pt3_fops = {
	.owner		=	THIS_MODULE,
	.open		=	pt3_open,
	.release	=	pt3_release,
	.read		=	pt3_read,
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36)
	.ioctl		=	pt3_ioctl,
#else
	.unlocked_ioctl		=	pt3_unlocked_ioctl,
	.compat_ioctl		=	pt3_compat_ioctl,
#endif
	.llseek		=	no_llseek,
};

static int __devinit
pt3_pci_init_one (struct pci_dev *pdev, const struct pci_device_id *ent)
{
	int			rc ;
	int			lp ;
	int			minor ;
	int			bars;
	u32			class_revision ;
	PT3_DEVICE	*dev_conf ;
	PT3_TUNER *tuner;
	PT3_CHANNEL *channel;

	bars = pci_select_bars(pdev, IORESOURCE_MEM);
	rc = pci_enable_device(pdev);
	if (rc)
		return rc;

	rc =pci_request_selected_regions(pdev, bars, pt3_driver_name);
	if (rc)
		goto out_err_pci;

	pci_set_master(pdev);
	PT3_PRINTK(0, KERN_INFO, "Bus Mastering Enabled.\n");
	rc = pci_save_state(pdev);
	if (rc)
		goto out_err_reg;

	pci_read_config_dword(pdev, PCI_CLASS_REVISION, &class_revision);
	if ((class_revision & 0xFF) != 1) {
		PT3_PRINTK(0, KERN_ERR, "Revision %x is not supported\n",
				(class_revision & 0xFF));
		goto out_err_reg;
	}
	PT3_PRINTK(7, KERN_DEBUG, "Revision check passed. revision=0x%x\n", class_revision & 0xff);

	dev_conf = kzalloc(sizeof(PT3_DEVICE), GFP_KERNEL);
	if(!dev_conf){
		PT3_PRINTK(0, KERN_ERR, "out of memory !\n");
		goto out_err_reg;
	}
	PT3_PRINTK(7, KERN_DEBUG, "Allocate PT3_DEVICE.\n");

	// PCIアドレスをマップする
	dev_conf->bars = bars;
	dev_conf->hw_addr[0] = pci_ioremap_bar(pdev, 0);
	if (!dev_conf->hw_addr[0])
		goto out_err_fpga;
	dev_conf->hw_addr[1] = pci_ioremap_bar(pdev, 2);
	if (!dev_conf->hw_addr[1])
		goto out_err_fpga;

	rc = pci_set_dma_mask(pdev, DMA_BIT_MASK(64));
	if (!rc) {
		rc = pci_set_consistent_dma_mask(pdev, DMA_BIT_MASK(64));
	} else {
		PT3_PRINTK(0, KERN_ERR, "DMA MASK ERROR\n");
		goto out_err_fpga;
	}

	if(check_fpga_version(dev_conf)){
		goto out_err_fpga;
	}
	mutex_init(&dev_conf->lock);
	dev_conf->i2c = create_pt3_i2c(dev_conf->hw_addr);
	if (dev_conf->i2c == NULL) {
		PT3_PRINTK(0, KERN_ERR, "cannot allocate i2c.\n");
		goto out_err_fpga;
	}
	PT3_PRINTK(7, KERN_DEBUG, "Allocate PT3_I2C.\n");

	set_lnb(dev_conf, 0);
	// Tuner
	for (lp = 0; lp < MAX_TUNER; lp++) {
		__u8 tc_addr, tuner_addr;
		__u32 pin;

		tuner = &dev_conf->tuner[lp];
		tuner->tuner_no = lp;
		pin = 0;
		tc_addr = pt3_tc_address(pin, PT3_ISDB_S, lp);
		tuner_addr = pt3_qm_address(lp);

		tuner->tc_s = create_pt3_tc(dev_conf->i2c, tc_addr, tuner_addr);
		tuner->qm   = create_pt3_qm(dev_conf->i2c, tuner->tc_s);

		tc_addr = pt3_tc_address(pin, PT3_ISDB_T, lp);
		tuner_addr = pt3_mx_address(lp);

		tuner->tc_t = create_pt3_tc(dev_conf->i2c, tc_addr, tuner_addr);
		tuner->mx   = create_pt3_mx(dev_conf->i2c, tuner->tc_t);
	}
	PT3_PRINTK(7, KERN_DEBUG, "Allocate tuners.\n");

	rc = init_all_tuner(dev_conf);
	if (rc) {
		PT3_PRINTK(0, KERN_ERR, "fail init_all_tuner. 0x%x\n", rc);
		goto out_err_i2c;
	}

	for(lp = 0 ; lp < MAX_PCI_DEVICE ; lp++){
		PT3_PRINTK(0, KERN_INFO, "device[%d]=%p\n", lp, device[lp]);
		if(device[lp] == NULL){
			device[lp] = dev_conf ;
			dev_conf->card_number = lp;
			break ;
		}
	}

	rc =alloc_chrdev_region(&dev_conf->dev, 0, MAX_CHANNEL, DEV_NAME);
	if (rc < 0)
		goto out_err_i2c;
	minor = MINOR(dev_conf->dev) ;
	dev_conf->base_minor = minor ;
	for (lp = 0; lp < MAX_CHANNEL; lp++) {
		cdev_init(&dev_conf->cdev[lp], &pt3_fops);
		dev_conf->cdev[lp].owner = THIS_MODULE;
		rc = cdev_add(&dev_conf->cdev[lp],
			MKDEV(MAJOR(dev_conf->dev), (MINOR(dev_conf->dev) + lp)), 1);
		if (rc < 0) {
			PT3_PRINTK(0, KERN_ERR, "fail cdev_add.\n");
		}

		channel = kzalloc(sizeof(PT3_CHANNEL), GFP_KERNEL);
		if (channel == NULL) {
			PT3_PRINTK(0, KERN_ERR, "out of memory !\n");
			goto out_err_dma;
		}

		channel->dma = create_pt3_dma(pdev, dev_conf->i2c, real_channel[lp]);
		if (channel->dma == NULL) {
			PT3_PRINTK(0, KERN_ERR, "fail create dma.\n");
			kfree(channel);
			goto out_err_dma;
		}

		mutex_init(&channel->lock);
		channel->minor = MINOR(dev_conf->dev) + lp;
		channel->tuner = &dev_conf->tuner[real_channel[lp] & 1];
		channel->type = channel_type[lp];
		channel->ptr = dev_conf;
		channel->i2c = dev_conf->i2c;

		dev_conf->channel[lp] = channel;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
		PT3_PRINTK(0, KERN_INFO, "card_number = %d channel=%d\n",
					dev_conf->card_number, real_channel[lp]);
		device_create(pt3video_class,
					NULL,
					MKDEV(MAJOR(dev_conf->dev), (MINOR(dev_conf->dev) + lp)),
					NULL,
					"pt3video%u",
					MINOR(dev_conf->dev) + lp + dev_conf->card_number * MAX_CHANNEL);
#else
		device_create(pt3video_class,
					NULL,
					MKDEV(MAJOR(dev_conf->dev), (MINOR(dev_conf->dev) + lp)),
					"pt3video%u",
					MINOR(dev_conf->dev) + lp + dev_conf->card_number * MAX_CHANNEL);
#endif
	}

	pci_set_drvdata(pdev, dev_conf);
	return 0;

out_err_dma:
	for (lp = 0; lp < MAX_CHANNEL; lp++) {
		if (dev_conf->channel[lp] != NULL) {
			if (dev_conf->channel[lp]->dma != NULL)
				free_pt3_dma(pdev, dev_conf->channel[lp]->dma);
			kfree(dev_conf->channel[lp]);
			device_destroy(pt3video_class,
					MKDEV(MAJOR(dev_conf->dev), (MINOR(dev_conf->dev) + lp)));
		}
	}
out_err_i2c:
	for (lp = 0; lp < MAX_TUNER; lp++) {
		tuner = &dev_conf->tuner[lp];
		free_pt3_tc(tuner->tc_s);
		free_pt3_qm(tuner->qm);
		free_pt3_tc(tuner->tc_t);
		free_pt3_mx(tuner->mx);
	}
	free_pt3_i2c(dev_conf->i2c);
out_err_fpga:
	if (dev_conf->hw_addr[0])
		iounmap(dev_conf->hw_addr[0]);
	if (dev_conf->hw_addr[1])
		iounmap(dev_conf->hw_addr[1]);
	kfree(dev_conf);
out_err_reg:
	pci_release_selected_regions(pdev, bars);
out_err_pci:
	pci_disable_device(pdev);
	return -EIO;
}

static void __devexit
pt3_pci_remove_one(struct pci_dev *pdev)
{
	__u32 lp;
	PT3_TUNER *tuner;
	PT3_CHANNEL *channel;
	PT3_DEVICE	*dev_conf = (PT3_DEVICE *)pci_get_drvdata(pdev);

	if(dev_conf){
		for (lp = 0; lp < MAX_CHANNEL; lp++) {
			channel = dev_conf->channel[lp];
			if (channel->dma->enabled)
				pt3_dma_set_enabled(channel->dma, 0);
			set_tuner_sleep(channel->type, channel->tuner, 1);
		}
		set_lnb(dev_conf, 0);
		for (lp = 0; lp < MAX_TUNER; lp++) {
			tuner = &dev_conf->tuner[lp];

			if (tuner->tc_s != NULL) {
				free_pt3_tc(tuner->tc_s);
			}
			if (tuner->tc_t != NULL) {
				pt3_tc_set_powers(tuner->tc_t, NULL, 0, 0);
				free_pt3_tc(tuner->tc_t);
			}
			if (tuner->qm != NULL)
				free_pt3_qm(tuner->qm);
			if (tuner->mx != NULL)
				free_pt3_mx(tuner->mx);
		}
		for (lp = 0; lp < MAX_CHANNEL; lp++) {
			if (dev_conf->channel[lp] != NULL) {
				cdev_del(&dev_conf->cdev[lp]);
				if (dev_conf->channel[lp]->dma != NULL)
					free_pt3_dma(pdev, dev_conf->channel[lp]->dma);
				kfree(dev_conf->channel[lp]);
			}
			device_destroy(pt3video_class,
						MKDEV(MAJOR(dev_conf->dev), (MINOR(dev_conf->dev) + lp)));
		}
		pt3_i2c_reset(dev_conf->i2c);
		free_pt3_i2c(dev_conf->i2c);

		unregister_chrdev_region(dev_conf->dev, MAX_CHANNEL);
		if (dev_conf->hw_addr[0])
			iounmap(dev_conf->hw_addr[0]);
		if (dev_conf->hw_addr[1])
			iounmap(dev_conf->hw_addr[1]);
		pci_release_selected_regions(pdev, dev_conf->bars);
		device[dev_conf->card_number] = NULL;
		kfree(dev_conf);
		PT3_PRINTK(0, KERN_DEBUG, "free PT3 DEVICE.\n");
	}
	pci_disable_device(pdev);
	pci_set_drvdata(pdev, NULL);
}

#ifdef CONFIG_PM

static int
pt3_pci_suspend (struct pci_dev *pdev, pm_message_t state)
{
	return 0;
}

static int
pt3_pci_resume (struct pci_dev *pdev)
{
	return 0;
}

#endif /* CONFIG_PM */


static struct pci_driver pt3_driver = {
	.name		= pt3_driver_name,
	.probe		= pt3_pci_init_one,
	.remove		= __devexit_p(pt3_pci_remove_one),
	.id_table	= pt3_pci_tbl,
#ifdef CONFIG_PM
	.suspend	= pt3_pci_suspend,
	.resume		= pt3_pci_resume,
#endif /* CONFIG_PM */

};


static int __init
pt3_pci_init(void)
{
	PT3_PRINTK(0, KERN_INFO, "%s", version);
	pt3video_class = class_create(THIS_MODULE, DRIVERNAME);
	if (IS_ERR(pt3video_class))
		return PTR_ERR(pt3video_class);
	return pci_register_driver(&pt3_driver);
}


static void __exit
pt3_pci_cleanup(void)
{
	pci_unregister_driver(&pt3_driver);
	class_destroy(pt3video_class);
}

module_init(pt3_pci_init);
module_exit(pt3_pci_cleanup);

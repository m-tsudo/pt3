#define DRV_NAME	"pt3-pci"
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

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,23)
#include <linux/freezer.h>
#else
#define set_freezable()
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,11)
typedef struct pm_message {
        int event;
} pm_message_t;
#endif
#endif
#include <linux/kthread.h>
#include <linux/dma-mapping.h>

#include <linux/fs.h>
#include <linux/cdev.h>

#include <linux/ioctl.h>

#include	"pt3_com.h"
#include	"pt3_ioctl.h"
#include	"pt3_pci.h"
#include	"pt3_i2c_bus.h"
#include	"pt3_tc.h"
#include	"pt3_qm.h"
#include	"pt3_mx.h"
#include	"pt3_dma.h"

/* These identify the driver base version and may not be removed. */
static char version[] __devinitdata =
DRV_NAME ".c: " DRV_VERSION " " DRV_RELDATE " \n";

MODULE_AUTHOR("anyone");
#define	DRIVER_DESC		"PCI earthsoft PT3 driver"
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

static int debug = 7;			/* 1 normal messages, 0 quiet .. 7 verbose. */
static int lnb = 0;			/* LNB OFF:0 +11V:1 +15V:2 */

module_param(debug, int, 0);
module_param(lnb, int, 0);
MODULE_PARM_DESC(debug, "debug level (1-2)");
MODULE_PARM_DESC(debug, "LNB level (0:OFF 1:+11V 2:+15V)");

#define VENDOR_ALTERA 0x1172
#define PCI_PT3_ID 0x4c15

static struct pci_device_id pt3_pci_tbl[] = {
	{ VENDOR_ALTERA, PCI_PT3_ID, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ 0, }
};
MODULE_DEVICE_TABLE(pci, pt3_pci_tbl);
#define		DEV_NAME	"pt3video"

#define		PACKET_SIZE			188		// 1パケット長
#define		MAX_READ_BLOCK	4			// 1度に読み出す最大DMAバッファ数
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
	BAR		bar[2];
	struct mutex		lock ;
	dev_t			dev ;
	int			card_number;
	__u32			base_minor ;
	struct	cdev	cdev[MAX_CHANNEL];
	PT3_VERSION		version;
	PT3_SYSTEM		system;
	PT3_I2C_BUS		*bus;
	PT3_TUNER       tuner[MAX_TUNER];
	PT3_CHANNEL		*channel[MAX_CHANNEL];
} PT3_DEVICE;

struct _PT3_CHANNEL {
	__u32			valid ;
	__u32			minor;
	PT3_TUNER		*tuner;
	int				type ;
	struct mutex	lock ;
	struct mutex	biglock ;
	PT3_DEVICE		*ptr ;
	PT3_I2C_BUS		*bus;
	PT3_DMA			*dma;
};

static int real_channel[MAX_CHANNEL] = {0, 2, 1, 3};
static int channel_type[MAX_CHANNEL] = {PT3_ISDB_S, PT3_ISDB_S,
										PT3_ISDB_T, PT3_ISDB_T};

static	PT3_DEVICE	*device[MAX_PCI_DEVICE];
static struct class	*pt3video_class;

#define		PT3MAJOR	251
#define		DRIVERNAME	"pt3video"

static int
setup_bar(struct pci_dev *pdev, BAR *bar, int index)
{
	struct resource *dummy;

	bar->mmio_start = pci_resource_start(pdev, index);
	bar->mmio_len = pci_resource_len(pdev, index);
	dummy = request_mem_region(bar->mmio_start, bar->mmio_len, DEV_NAME);
	if (!dummy) {
		printk(KERN_ERR "PT3:cannot request iomem  (0x%llx).\n", (unsigned long long) bar->mmio_start);
		goto out_err_regbase;
	}
	printk(KERN_DEBUG "request_mem_resion success. mmio_start=%lu mmio_len=%u",
						bar->mmio_start, bar->mmio_len);

	bar->regs = ioremap(bar->mmio_start, bar->mmio_len);
	if (!bar->regs){
		printk(KERN_ERR "pt3:Can't remap register area.\n");
		goto out_err_regbase;
	}
	printk(KERN_DEBUG "io_remap success. %p", bar->regs);

	return 0;
out_err_regbase:
	return -1;
}

static int
ep4c_init(PT3_DEVICE *dev_conf)
{
	__u32	val;
	
	val = readl(dev_conf->bar[0].regs + REGS_VERSION);

	dev_conf->version.ptn = ((val >> 24) & 0xFF);
	dev_conf->version.regs = ((val >> 16) & 0xFF);
	dev_conf->version.fpga = ((val >> 8) & 0xFF);
	if (dev_conf->version.ptn != 3) {
		printk(KERN_ERR "PTn needs 3.\n");
		return -1;
	}
	printk(KERN_DEBUG "Check PTn is passed.\n");

	if (dev_conf->version.fpga != 0x04) {
		printk(KERN_ERR "this FPGA version not supported\n");
		return -1;
	}
	printk(KERN_DEBUG "Check FPGA version is passed.");

	val = readl(dev_conf->bar[0].regs + REGS_SYSTEM_R);
	dev_conf->system.can_transport_ts = ((val >> 5) & 0x01);
	printk(KERN_DEBUG "can_transport_ts = %d\n",
						dev_conf->system.can_transport_ts);
	dev_conf->system.dma_descriptor_page_size = (val & 0x1F);
	printk(KERN_DEBUG "dma_descriptor_page_size = %d\n",
						dev_conf->system.dma_descriptor_page_size);

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

static __u32 LNB_SETTINGS[] = {
	(1 << 3 | 0 << 1) | (1 << 2 | 0 << 9),	// 0v
	(1 << 3 | 0 << 1) | (1 << 2 | 1 << 0),	// 12v
	(1 << 3 | 1 << 1) | (1 << 2 | 1 << 0),	// 15v
};

static STATUS
set_lnb(PT3_DEVICE *dev_conf, int lnb)
{
	if (lnb < 0 || 2 < lnb)
		return STATUS_INVALID_PARAM_ERROR;
	writel(LNB_SETTINGS[lnb], dev_conf->bar[0].regs + REGS_SYSTEM_W);
	return STATUS_OK;
}

static STATUS
set_frequency(int isdb, PT3_TUNER *tuner, __u32 channel, __s32 offset)
{
	STATUS status;

	switch (isdb) {
	case PT3_ISDB_S :
		status = pt3_qm_set_frequency(tuner->qm, channel, offset);
		break;
	case PT3_ISDB_T :
		status = pt3_mx_set_frequency(tuner->mx, channel, offset);
		break;
	default :
		status = STATUS_INVALID_PARAM_ERROR;
	}

	return status;
}

static int
set_tuner_sleep(int isdb, PT3_TUNER *tuner, int sleep)
{
	STATUS status;

	switch (isdb) {
	case PT3_ISDB_S :
		status = pt3_qm_set_sleep(tuner->qm, sleep);
		break;
	case PT3_ISDB_T :
		status = pt3_mx_set_sleep(tuner->mx, sleep);
		break;
	default :
		status = STATUS_INVALID_PARAM_ERROR;
	}

	return status;
}

static int
init_tuner(PT3_I2C_BUS *bus, PT3_TUNER *tuner)
{
	int status;

	pt3_qm_init_reg_param(tuner->qm);
	{
		pt3_qm_dummy_reset(tuner->qm);
		pt3_i2c_bus_end(bus);
		pt3_i2c_bus_run(bus, NULL, 1);
	}

	{
		status = pt3_qm_init(tuner->qm);
		if (status)
			return status;
		pt3_i2c_bus_end(bus);
		pt3_i2c_bus_run(bus, NULL, 1);
	}

	return status;
}

static STATUS
tuner_power_on(PT3_DEVICE *dev_conf)
{
	int status, i;
	PT3_TS_PINS_MODE pins;

	PT3_I2C_BUS *bus = dev_conf->bus;
	PT3_TUNER *tuner;

	for (i = 0; i < MAX_TUNER; i++) {
		tuner = &dev_conf->tuner[i];
		pt3_tc_init_s(tuner->tc_s);
		pt3_tc_init_t(tuner->tc_t);
		printk(KERN_DEBUG "tc_init %d", i);
	}

	schedule_timeout_interruptible(msecs_to_jiffies(1));	

	tuner = &dev_conf->tuner[1];
	status = pt3_tc_set_powers(tuner->tc_t, 1, 0);
	if (status)
		return status;

	pins.clock_data = PT3_TS_PIN_MODE_NORMAL;
	pins.byte = PT3_TS_PIN_MODE_NORMAL;
	pins.valid = PT3_TS_PIN_MODE_NORMAL;

	for (i = 0; i < MAX_TUNER; i++) {
		tuner = &dev_conf->tuner[i];
		pt3_tc_set_ts_pins_mode_s(tuner->tc_s, &pins);
		pt3_tc_set_ts_pins_mode_t(tuner->tc_t, &pins);
	}

	schedule_timeout_interruptible(msecs_to_jiffies(1));	

	for (i = 0; i < MAX_TUNER; i++) {
		status = init_tuner(bus, &dev_conf->tuner[i]);
		if (status)
			return status;
		printk(KERN_DEBUG "init_tuner %d", i);
	}

	bus->inst_addr = PT3_I2C_INST_ADDR1;

	status = pt3_i2c_bus_run(bus, NULL, 0);
	if (status)
		return status;

	status = pt3_tc_set_powers(tuner->tc_t, 1, 1);
	if (status)
		return status;

	return status;
}

static int
init_all_tuner(PT3_DEVICE *dev_conf)
{
	STATUS status;
	int i, j, channel;
	PT3_I2C_BUS *bus = dev_conf->bus;

	pt3_i2c_bus_end(bus);
	bus->inst_addr = PT3_I2C_INST_ADDR0;

	if (!pt3_i2c_bus_is_clean(bus)) {
		printk(KERN_INFO "I2C bus is dirty.");
		status = pt3_i2c_bus_run(bus, NULL, 0);
		if (status)
			return status;
	}

	status = tuner_power_on(dev_conf);
	if (status)
		return status;
	
	for (i = 0; i < MAX_TUNER; i++) {
		for (j = 0; i < PT3_ISDB_MAX; j++) {
			if (j == PT3_ISDB_S)
				channel = 0;
			else
				channel = (i == 0) ? 70 : 71;
			status = set_tuner_sleep(j, &dev_conf->tuner[i], 0);
			if (status)
				return status;
			status = set_frequency(j, &dev_conf->tuner[i], channel, 0);
			if (status)
				return status;
			status = set_tuner_sleep(j, &dev_conf->tuner[i], 1);
			if (status)
				return status;
		}
	}

	return status;
}

static int pt3_open(struct inode *inode, struct file *file)
{
	int major = imajor(inode);
	int minor = iminor(inode);
	int lp, lp2;
	PT3_CHANNEL *channel;

	for (lp = 0; lp < MAX_PCI_DEVICE; lp++) {
		if (device[lp] == NULL) {
			printk(KERN_DEBUG "device is not exists");
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
						printk(KERN_DEBUG "device is already used.");
						return -EIO;
					}
					printk(KERN_DEBUG "PT3: selected tuner_no=%d type=%d",
							channel->tuner->tuner_no, channel->type);

					set_tuner_sleep(channel->type, channel->tuner, 0);
					schedule_timeout_interruptible(msecs_to_jiffies(50));
	
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

static int pt3_release(struct inode *inode, struct file *file)
{
	PT3_CHANNEL *channel = file->private_data;

	mutex_lock(&channel->ptr->lock);
	channel->valid = 0;
	pt3_dma_set_enabled(channel->dma, 0);
	mutex_unlock(&channel->ptr->lock);

	set_lnb(channel->ptr, 0);
	set_tuner_sleep(channel->type, channel->tuner, 1);
	schedule_timeout_interruptible(msecs_to_jiffies(50));

	return 0;
}

static ssize_t pt3_read(struct file *file, char __user *buf, size_t cnt, loff_t * ppos)
{
	size_t rcnt;
	PT3_CHANNEL *channel;

	channel = file->private_data;

	rcnt = pt3_dma_copy(channel->dma, buf, cnt);
	if (rcnt < 0) {
		printk(KERN_INFO "PT3: fail copy_to_user.");
		return -EFAULT;
	}
	*ppos += rcnt;

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

	printk(KERN_INFO "used bs tuners on %p = %d", device, count);

	return count;
}

static long pt3_do_ioctl(struct file  *file, unsigned int cmd, unsigned long arg0)
{
	PT3_CHANNEL *channel;
	int status, count, lnb_eff, lnb_usr;
	unsigned long dummy;
	char *voltage[] = {"0V", "11V", "15V"};
	void *arg;

	channel = file->private_data;
	arg = (void *)arg0;

	switch (cmd) {
	case START_REC:
		pt3_dma_set_enabled(channel->dma, 1);
		return 0;
	case STOP_REC:
		pt3_dma_set_enabled(channel->dma, 0);
		return 0;
	case LNB_ENABLE:
		count = count_used_bs_tuners(channel->ptr);
		if (count <= 1) {
			lnb_usr = (int)arg0;
			lnb_eff = lnb_usr ? lnb_usr : lnb;
			set_lnb(channel->ptr, lnb_eff);
			printk(KERN_INFO "PT3: LNB on %s", voltage[lnb_eff]);
		}
		return 0;
	case LNB_DISABLE:
		count = count_used_bs_tuners(channel->ptr);
		if (count <= 1) {
			set_lnb(channel->ptr, 0);
			printk(KERN_INFO "PT3: LNB off");
		}
		return 0;
	case GET_STATUS:
		status = (int)pt3_dma_get_status(channel->dma);
		dummy = copy_to_user(arg, &status, sizeof(int));
		return 0;
	case SET_TEST_MODE:
		pt3_dma_set_test_mode(channel->dma, 1, 0, 0, 0);
		return 0;
	}

	return -EINVAL;
}

static long pt3_unlocked_ioctl(struct file  *file, unsigned int cmd, unsigned long arg0)
{
	long ret;
	PT3_CHANNEL *channel = file->private_data;

	mutex_lock(&channel->biglock);
	ret = pt3_do_ioctl(file, cmd, arg0);
	mutex_unlock(&channel->biglock);

	return ret;
}

static long pt3_compat_ioctl(struct file  *file, unsigned int cmd, unsigned long arg0)
{
	long ret;
	/* should do 32bit <-> 64bit conversion here? --yaz */
	ret = pt3_unlocked_ioctl(file, cmd, arg0);

	return ret;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36)
static int pt3_ioctl(struct inode *inode, struct file  *file, unsigned int cmd, unsigned long arg0)
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

static int __devinit pt3_pci_init_one (struct pci_dev *pdev,
				     const struct pci_device_id *ent)
{
	int			rc ;
	int			lp ;
	int			minor ;
	u16			cmd ;
	u32			class_revision ;
	PT3_DEVICE	*dev_conf ;
	PT3_CHANNEL *channel;

	rc = pci_enable_device(pdev);
	if (rc)
		return rc;

	rc = pci_set_dma_mask(pdev, DMA_BIT_MASK(64));
	if (rc) {
		printk(KERN_ERR "PT3:DMA MASK ERROR");
		return rc;
	}

	pci_read_config_dword(pdev, PCI_CLASS_REVISION, &class_revision);
	if ((class_revision & 0xFF) != 1) {
		printk(KERN_ERR "Revision %x is not supported\n",
				(class_revision & 0xFF));
		return -EIO;
	}
	printk(KERN_DEBUG "Revision check passwd.");

	pci_read_config_word(pdev, PCI_COMMAND, &cmd);
	if (!(cmd & PCI_COMMAND_MASTER)) {
		printk(KERN_INFO "Attempting to enable Bus Mastering\n");
		pci_set_master(pdev);
		pci_read_config_word(pdev, PCI_COMMAND, &cmd);
		if (!(cmd & PCI_COMMAND_MASTER)) {
			printk(KERN_ERR "Bus Mastering is not enabled\n");
			return -EIO;
		}
	}
	printk(KERN_INFO "Bus Mastering Enabled.\n");

	dev_conf = kzalloc(sizeof(PT3_DEVICE), GFP_KERNEL);
	if(!dev_conf){
		printk(KERN_ERR "PT3:out of memory !");
		return -ENOMEM ;
	}
	printk(KERN_DEBUG "Allocate PT3_DEVICE.");

	// PCIアドレスをマップする
	if (setup_bar(pdev, &dev_conf->bar[0], 0))
		goto out_err_regbase;
	if (setup_bar(pdev, &dev_conf->bar[1], 2))
		goto out_err_regbase;

	// 初期化処理
	if(ep4c_init(dev_conf)){
		printk(KERN_ERR "Error ep4c_init\n");
		goto out_err_fpga;
	}
	mutex_init(&dev_conf->lock);
	dev_conf->bus = create_pt3_i2c_bus(&dev_conf->bar[0]);
	if (dev_conf->bus == NULL) {
		printk(KERN_ERR "PT3: cannot allocate bus.");
		goto out_err_i2c_bus;
	}
	printk(KERN_DEBUG "Allocate PT3_I2C_BUS.");

	// Tuner
	for (lp = 0; lp < MAX_TUNER; lp++) {
		__u8 tc_addr, tuner_addr;
		__u32 pin;
		PT3_TUNER *tuner;

		tuner = &dev_conf->tuner[lp];
		tuner->tuner_no = lp;
		pin = 0;
		tc_addr = pt3_tc_address(pin, PT3_ISDB_S, lp);
		tuner_addr = pt3_qm_address(lp);

		tuner->tc_s = create_pt3_tc(dev_conf->bus, tc_addr, tuner_addr);
		tuner->qm   = create_pt3_qm(dev_conf->bus, tuner->tc_s);

		tc_addr = pt3_tc_address(pin, PT3_ISDB_T, lp);
		tuner_addr = pt3_mx_address(lp);

		tuner->tc_t = create_pt3_tc(dev_conf->bus, tc_addr, tuner_addr);
		tuner->mx   = create_pt3_mx(dev_conf->bus, tuner->tc_t);
	}
	printk(KERN_DEBUG "Allocate tuners.");

	init_all_tuner(dev_conf);

	minor = MINOR(dev_conf->dev) ;
	dev_conf->base_minor = minor ;
	for(lp = 0 ; lp < MAX_PCI_DEVICE ; lp++){
		printk(KERN_INFO "PT3:device[%d]=%p\n", lp, device[lp]);
		if(device[lp] == NULL){
			device[lp] = dev_conf ;
			dev_conf->card_number = lp;
			break ;
		}
	}

	rc =alloc_chrdev_region(&dev_conf->dev, 0, MAX_CHANNEL, DEV_NAME);
	if (rc < 0)
		goto out_err_fpga;
	minor = MINOR(dev_conf->dev);
	dev_conf->base_minor = minor;
	for (lp = 0; lp < MAX_CHANNEL; lp++) {
		cdev_init(&dev_conf->cdev[lp], &pt3_fops);
		dev_conf->cdev[lp].owner = THIS_MODULE;
		rc = cdev_add(&dev_conf->cdev[lp],
			MKDEV(MAJOR(dev_conf->dev), (MINOR(dev_conf->dev) + lp)), 1);
		if (rc < 0) {
			printk(KERN_ERR "fail cdev_add.");
		}

		channel = kzalloc(sizeof(PT3_CHANNEL), GFP_KERNEL);
		if (channel == NULL) {
			printk(KERN_ERR "PT3:out of memory !");
			goto out_err_v4l;
		}

		channel->dma = create_pt3_dma(pdev, dev_conf->bus, real_channel[lp]);
		if (channel->dma == NULL) {
			printk(KERN_ERR "PT3: fail create dma.");
			kfree(channel);
			goto out_err_v4l;
		}

		mutex_init(&channel->lock);
		mutex_init(&channel->biglock);
		channel->minor = MINOR(dev_conf->dev) + lp;
		channel->tuner = &dev_conf->tuner[lp & 0x01];
		channel->type = channel_type[lp];
		channel->ptr = dev_conf;
		channel->bus = dev_conf->bus;

		dev_conf->channel[lp] = channel;

		switch (channel->type) {
		case PT3_ISDB_S :
			break;
		case PT3_ISDB_T :
			break;
		}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
		printk(KERN_INFO "PT3:card_number = %d\n", dev_conf->card_number);
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

out_err_v4l:
	for (lp = 0; lp < MAX_CHANNEL; lp++) {
		if (dev_conf->channel[lp] != NULL) {
			if (dev_conf->channel[lp]->dma != NULL)
				free_pt3_dma(pdev, dev_conf->channel[lp]->dma);
			kfree(dev_conf->channel[lp]);
			device_destroy(pt3video_class,
					MKDEV(MAJOR(dev_conf->dev), (MINOR(dev_conf->dev) + lp)));
		}
	}
out_err_i2c_bus:
	free_pt3_i2c_bus(dev_conf->bus);
out_err_fpga:
	iounmap(dev_conf->bar[0].regs);
	release_mem_region(dev_conf->bar[0].mmio_start, dev_conf->bar[0].mmio_len);
	iounmap(dev_conf->bar[1].regs);
	release_mem_region(dev_conf->bar[1].mmio_start, dev_conf->bar[1].mmio_len);
out_err_regbase:
	kfree(dev_conf);
	return -EIO;
}

static void __devexit pt3_pci_remove_one(struct pci_dev *pdev)
{

	__u32 lp;
	PT3_DEVICE	*dev_conf = (PT3_DEVICE *)pci_get_drvdata(pdev);

	if(dev_conf){
		for (lp = 0; lp < MAX_TUNER; lp++) {
			PT3_TUNER *tuner = &dev_conf->tuner[lp];

			if (tuner->tc_s != NULL) {
				pt3_tc_set_powers(tuner->tc_s, 0, 0);
				free_pt3_tc(tuner->tc_s);
			}
			if (tuner->tc_t != NULL) {
				pt3_tc_set_powers(tuner->tc_t, 0, 0);
				free_pt3_tc(tuner->tc_t);
			}
			if (tuner->qm != NULL)
				free_pt3_qm(tuner->qm);
			if (tuner->mx != NULL)
				free_pt3_mx(tuner->mx);
		}
		if (dev_conf->bus)
			free_pt3_i2c_bus(dev_conf->bus);

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
		
		unregister_chrdev_region(dev_conf->dev, MAX_CHANNEL);
		release_mem_region(dev_conf->bar[0].mmio_start, dev_conf->bar[0].mmio_len);
		release_mem_region(dev_conf->bar[1].mmio_start, dev_conf->bar[1].mmio_len);
		iounmap(dev_conf->bar[0].regs);
		iounmap(dev_conf->bar[1].regs);
		device[dev_conf->card_number] = NULL;
		kfree(dev_conf);
		printk(KERN_DEBUG "free PT3 DEVICE.");
	}
	pci_set_drvdata(pdev, NULL);
}

#ifdef CONFIG_PM

static int pt3_pci_suspend (struct pci_dev *pdev, pm_message_t state)
{
	return 0;
}

static int pt3_pci_resume (struct pci_dev *pdev)
{
	return 0;
}

#endif /* CONFIG_PM */


static struct pci_driver pt3_driver = {
	.name		= DRV_NAME,
	.probe		= pt3_pci_init_one,
	.remove		= __devexit_p(pt3_pci_remove_one),
	.id_table	= pt3_pci_tbl,
#ifdef CONFIG_PM
	.suspend	= pt3_pci_suspend,
	.resume		= pt3_pci_resume,
#endif /* CONFIG_PM */

};


static int __init pt3_pci_init(void)
{
	printk(KERN_INFO "%s", version);
	pt3video_class = class_create(THIS_MODULE, DRIVERNAME);
	if (IS_ERR(pt3video_class))
		return PTR_ERR(pt3video_class);
	return pci_register_driver(&pt3_driver);
}


static void __exit pt3_pci_cleanup(void)
{
	pci_unregister_driver(&pt3_driver);
	class_destroy(pt3video_class);
}

module_init(pt3_pci_init);
module_exit(pt3_pci_cleanup);

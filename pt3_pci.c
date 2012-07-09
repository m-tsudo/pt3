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
#include	"pt3_pci.h"
#include	"pt3_i2c_bus.h"

/* These identify the driver base version and may not be removed. */
static char version[] __devinitdata =
DRV_NAME ".c: " DRV_VERSION " " DRV_RELDATE " \n";

MODULE_AUTHOR("Tomoaki Ishikawa tomy@users.sourceforge.jp and Yoshiki Yazawa yaz@honeyplanet.jp");
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

typedef	struct	_pt3_device{
	unsigned long	mmio_start ;
	__u32			mmio_len ;
	void __iomem		*regs;
	struct mutex		lock ;
	struct	task_struct	*kthread;
	dev_t			dev ;
	int			card_number;
	__u32			base_minor ;
	struct	cdev	cdev[MAX_CHANNEL];
	wait_queue_head_t	dma_wait_q ;// for poll on reading
	PT3_VERSION		version;
	PT3_SYSTEM		system;
	PT3_I2C_BUS		*i2c_bus;
} PT3_DEVICE;

static	PT3_DEVICE	*device[MAX_PCI_DEVICE];
static struct class	*pt3video_class;

#define		PT3MAJOR	251
#define		DRIVERNAME	"pt3video"

static int
ep4c_init(PT3_DEVICE *dev_conf)
{
	__u32	val;
	
	val = readl(dev_conf->regs + REGS_VERSION);

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

	val = readl(dev_conf->regs + REGS_SYSTEM_R);
	dev_conf->system.can_transport_ts = ((val >> 5) & 0x01);
	printk(KERN_DEBUG "can_transport_ts = %d\n",
						dev_conf->system.can_transport_ts);
	dev_conf->system.dma_descriptor_page_size = (val & 0x1F);
	printk(KERN_DEBUG "dma_descriptor_page_size = %d\n",
						dev_conf->system.dma_descriptor_page_size);

	return 0;
}

static int pt3_open(struct inode *inode, struct file *file)
{
	return -EIO;
}

static int pt3_release(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t pt3_read(struct file *file, char __user *buf, size_t cnt, loff_t * ppos)
{
	return 0;
}

static long pt3_do_ioctl(struct file  *file, unsigned int cmd, unsigned long arg0)
{
	return -EINVAL;
}

static long pt3_unlocked_ioctl(struct file  *file, unsigned int cmd, unsigned long arg0)
{
	return 0;
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
	struct resource *dummy;

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
	dev_conf->mmio_start = pci_resource_start(pdev, 0);
	dev_conf->mmio_len = pci_resource_len(pdev, 0);
	dummy = request_mem_region(dev_conf->mmio_start, dev_conf->mmio_len, DEV_NAME);
	if (!dummy) {
		printk(KERN_ERR "PT3:cannot request iomem  (0x%llx).\n", (unsigned long long) dev_conf->mmio_start);
		goto out_err_regbase;
	}
	printk(KERN_DEBUG "request_mem_resion success.");

	dev_conf->regs = ioremap(dev_conf->mmio_start, dev_conf->mmio_len);
	if (!dev_conf->regs){
		printk(KERN_ERR "pt3:Can't remap register area.\n");
		goto out_err_regbase;
	}
	printk(KERN_DEBUG "io_remap success.");

	// 初期化処理
	if(ep4c_init(dev_conf)){
		printk(KERN_ERR "Error ep4c_init\n");
		goto out_err_fpga;
	}
	mutex_init(&dev_conf->lock);
	dev_conf->i2c_bus = create_pt3_i2c_bus(dev_conf->regs);
	if (dev_conf->i2c_bus == NULL) {
		printk(KERN_ERR "PT3: cannot allocate i2c_bus.");
		goto out_err_i2c_bus;
	}
	printk(KERN_DEBUG "Allocate PT3_I2C_BUS.");

	// 初期化
	init_waitqueue_head(&dev_conf->dma_wait_q);

	// Tuner

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

	pci_set_drvdata(pdev, dev_conf);
	return 0;

out_err_i2c_bus:
	free_pt3_i2c_bus(dev_conf->i2c_bus);
out_err_fpga:
	iounmap(dev_conf->regs);
	release_mem_region(dev_conf->mmio_start, dev_conf->mmio_len);
out_err_regbase:
	kfree(dev_conf);
	return -EIO;
}

static void __devexit pt3_pci_remove_one(struct pci_dev *pdev)
{

	PT3_DEVICE	*dev_conf = (PT3_DEVICE *)pci_get_drvdata(pdev);

	if(dev_conf){
		if(dev_conf->kthread) {
			kthread_stop(dev_conf->kthread);
			dev_conf->kthread = NULL;
		}

		if (dev_conf->i2c_bus)
			free_pt3_i2c_bus(dev_conf->i2c_bus);
		
		unregister_chrdev_region(dev_conf->dev, MAX_CHANNEL);
		release_mem_region(dev_conf->mmio_start, dev_conf->mmio_len);
		iounmap(dev_conf->regs);
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

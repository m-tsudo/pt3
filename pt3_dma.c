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
#include "pt3_bus.h"
#include "pt3_dma.h"

#define DMA_PAGE_SIZE			4096
#define MAX_DESCS			204		/* 4096 / 20 */
#define BLOCK_COUNT		32
#define BLOCK_SIZE			(DMA_PAGE_SIZE * 47 * 8)
#define DMA_TS_BUF_SIZE		(BLOCK_SIZE * BLOCK_COUNT)
#define NOT_SYNC_BYTE		0x74

static void
dma_write_descriptor(__u64 ts_addr, __u32 size, __u64 next_addr, PT3_DMA_DESC *desc)
{
	desc->page_addr = ts_addr;
	desc->page_size = size;
	desc->next_addr = next_addr;
#if 0
	printk(KERN_DEBUG "descriptor : page_addr=%llx page_size=%08d next_addr=%llx",
				desc->page_addr, desc->page_size, desc->next_addr);
#endif
}

static void
dma_build_page_descriptor(PT3_DMA *dma)
{
	PT3_DMA_PAGE *desc_info, *ts_info;
	__u64 ts_addr, desc_addr;
	__u32 i, j, ts_size, desc_remain;
	PT3_DMA_DESC *prev, *curr;

	desc_info = dma->desc_info;
	ts_info = dma->ts_info;
	desc_addr = desc_info->addr;
	desc_remain = desc_info->size;
	curr = (PT3_DMA_DESC *)desc_info->data;
	prev = NULL;
	desc_info++;
	for (i = 0; i < dma->ts_count; i++) {
		ts_addr = ts_info->addr;
		ts_size = ts_info->size;
		ts_info++;
		for (j = 0; j < ts_size / DMA_PAGE_SIZE; j++) {
			if (desc_remain < sizeof(*curr)) {
				curr = (PT3_DMA_DESC *)desc_info->data;
				desc_addr = desc_info->addr;
				desc_remain = desc_info->size;
				desc_info++;
			}
			if (prev != NULL)
				prev->next_addr = desc_addr;
			dma_write_descriptor(ts_addr, DMA_PAGE_SIZE, 0, curr);
			ts_addr += DMA_PAGE_SIZE;

			prev = curr;
			curr++;
			desc_addr += sizeof(*curr);
			desc_remain -= sizeof(*curr);
		}
	}

	if (prev != NULL)
		prev->next_addr = dma->desc_info->addr;
}

#if 0
static void
dma_check_page_descriptor(PT3_DMA *dma)
{
	__u32 remain;
	__u32 i;
	PT3_DMA_PAGE *page;
	PT3_DMA_DESC *desc;

	for (i = 0; i < dma->desc_count; i++) {
		page = &dma->desc_info[i];
		remain = page->size;
		desc = (PT3_DMA_DESC*)&page->data[0];
		while (0 < remain) {
			if (remain < sizeof(*desc)) {
				printk(KERN_DEBUG "remain = %d", remain);
				break;
			}
			if (desc->page_addr == 0) {
				printk(KERN_DEBUG "page_addr = 0 remain = %d", remain);
				break;
			}
			printk(KERN_DEBUG "descriptor : page_addr=%llx page_size=%08d next_addr=%llx",
				desc->page_addr, desc->page_size, desc->next_addr);
			desc++;
			remain -= sizeof(*desc);
		}
	}
}
#endif

void __iomem *
get_base_addr(PT3_DMA *dma)
{
	return dma->i2c->bar[0].regs + REGS_DMA_DESC_L + 0x18 * dma->tuner_index;
}

void
pt3_dma_set_test_mode(PT3_DMA *dma, int test, __u16 init, int not, int reset)
{
	void __iomem *base;
	__u32 data;

	base = get_base_addr(dma);
	data = (reset ? 1: 0) << 18 | (not ? 1 : 0) << 17 | (test ? 1 : 0) << 16 | init;

#if 0
	printk(KERN_DEBUG "set_test_mode base=%p offset=0x%02x data=0x%04d",
			base, base - dma->i2c->bar[0].regs, data);
#endif

	writel(data, base + 0x0c);
}

void
pt3_dma_set_enabled(PT3_DMA *dma, int enabled)
{
	void __iomem *base;
	__u32 data;
	__u64 start_addr;

	base = get_base_addr(dma);
	start_addr = dma->desc_info->addr;

	if (enabled) {
		printk(KERN_DEBUG "enable dma tuner_index=%d start_addr=%llu offset=%d",
				dma->tuner_index, start_addr, base - dma->i2c->bar[0].regs);
		pt3_dma_reset(dma);
		writel( 1 << 1, base + 0x08);
		writel(BIT_SHIFT_MASK(start_addr,  0, 32), base + 0x0);
		printk(KERN_DEBUG "set dma address low %llx",
				BIT_SHIFT_MASK(start_addr,  0, 32));
		writel(BIT_SHIFT_MASK(start_addr, 32, 32), base + 0x4);
		printk(KERN_DEBUG "set dma address heigh %llx",
				BIT_SHIFT_MASK(start_addr, 32, 32));
		writel( 1 << 0, base + 0x08);
	} else {
		printk(KERN_DEBUG "disable dma tuner_index=%d", dma->tuner_index);
		writel(1 << 1, base + 0x08);
		while (1) {
			data = readl(base + 0x10);
			if (!BIT_SHIFT_MASK(data, 0, 1))
				break;
			schedule_timeout_interruptible(msecs_to_jiffies(1));
		}
	}
	dma->enabled = enabled;
}

ssize_t
pt3_dma_copy(PT3_DMA *dma, char __user *buf, size_t size)
{
	int ready;
	PT3_DMA_PAGE *page;
	size_t rsize, remain;
	__u8 *p;

	mutex_lock(&dma->lock);
	remain = size;
	while (1) {
		if (remain <= 0)
			break;
		ready = pt3_dma_ready(dma);
		if (!ready)
			break;

		page = &dma->ts_info[dma->ts_pos];
		if ((page->size - page->data_pos) > remain) {
			rsize = remain;
		} else {
			rsize = (page->size - page->data_pos);
		}
		if (copy_to_user(buf, page->data, rsize)) {
			mutex_unlock(&dma->lock);
			return -EFAULT;
		}
		/*
		printk(KERN_DEBUG "copy_to_user size=%d ts_pos=%d data_size = %d data_pos=%d",
				rsize, dma->ts_pos, page->size, page->data_pos);
		*/
		remain -= rsize;
		page->data_pos += rsize;
		if (page->data_pos >= page->size) {
			page->data_pos = 0;
			p = &page->data[page->data_pos];
			*p = NOT_SYNC_BYTE;
			dma->ts_pos++;
			if (dma->ts_pos >= dma->ts_count)
				dma->ts_pos = 0;
		}
		schedule_timeout_interruptible(msecs_to_jiffies(1));
	}
	mutex_unlock(&dma->lock);

	return size - remain;
}

int
pt3_dma_ready(PT3_DMA *dma)
{
	PT3_DMA_PAGE *page;
	__u8 *p;

	page = &dma->ts_info[dma->ts_pos];
	p = &page->data[page->data_pos];

	if (*p == 0x47)
		return 1;
	if (*p == NOT_SYNC_BYTE)
		return 0;

	printk(KERN_DEBUG "sync byte is not 0x47 and also NOT_SYNC_BYTE.");

	return 0;
}

void
pt3_dma_reset(PT3_DMA *dma)
{
	PT3_DMA_PAGE *page;
	__u32 i;

	for (i = 0; i < dma->ts_count; i++) {
		page = &dma->ts_info[i];
		memset(page->data, 0, page->size);
		page->data_pos = 0;
		*page->data = NOT_SYNC_BYTE;
	}
	dma->ts_pos = 0;
}

__u32
pt3_dma_get_status(PT3_DMA *dma)
{
	void __iomem *base;
	__u32 status;

	base = get_base_addr(dma);

	status = readl(base + 0x10);

	return status;
}


PT3_DMA *
create_pt3_dma(struct pci_dev *hwdev, PT3_I2C *i2c, int tuner_index)
{
	PT3_DMA *dma;
	PT3_DMA_PAGE *page;
	__u32 i;

	dma = kzalloc(sizeof(PT3_DMA), GFP_KERNEL);
	if (dma == NULL) {
		printk(KERN_ERR "fail allocate PT3_DMA");
		goto fail;
	}

	dma->enabled = 0;
	dma->i2c = i2c;
	dma->tuner_index = tuner_index;
	mutex_init(&dma->lock);
	
	dma->ts_count = BLOCK_COUNT;
	dma->ts_info = kzalloc(sizeof(PT3_DMA_PAGE) * dma->ts_count, GFP_KERNEL);
	if (dma->ts_info == NULL) {
		printk(KERN_ERR "fail allocate PT3_DMA_PAGE");
		goto fail;
	}
	for (i = 0; i < dma->ts_count; i++) {
		page = &dma->ts_info[i];
		page->size = BLOCK_SIZE;
		page->data = pci_alloc_consistent(hwdev, page->size, &page->addr);
		if (page->data == NULL) {
			printk(KERN_ERR "fail allocate consistent. %d", i);
			goto fail;
		}
	}
	printk(KERN_DEBUG "Allocate TS buffer.");

	dma->desc_count = (DMA_TS_BUF_SIZE / (PAGE_SIZE) + 203) / 204;
	dma->desc_info = kzalloc(sizeof(PT3_DMA_PAGE) * dma->desc_count, GFP_KERNEL);
	if (dma->desc_info == NULL) {
		printk(KERN_ERR "fail allocate PT3_DMA_PAGE");
		goto fail;
	}
	for (i = 0; i < dma->desc_count; i++) {
		page = &dma->desc_info[i];
		page->size = DMA_PAGE_SIZE;
		page->data = pci_alloc_consistent(hwdev, page->size, &page->addr);
		if (page->data == NULL) {
			printk(KERN_ERR "fail allocate consistent. %d", i);
			goto fail;
		}
	}
	printk(KERN_DEBUG "Allocate Descriptor buffer.");
	
	dma_build_page_descriptor(dma);
	printk(KERN_DEBUG "set page descriptor.");
#if 0
	dma_check_page_descriptor(dma);
#endif

	return dma;
fail:
	if (dma != NULL)
		free_pt3_dma(hwdev, dma);
	return NULL;
}

void
free_pt3_dma(struct pci_dev *hwdev, PT3_DMA *dma)
{
	PT3_DMA_PAGE *page;
	__u32 i;
	if (dma->ts_info != NULL) {
		for (i = 0; i < dma->ts_count; i++) {
			page = &dma->ts_info[i];
			if (page->size != 0)
				pci_free_consistent(hwdev, page->size, page->data, page->addr);
		}
		kfree(dma->ts_info);
	}
	if (dma->desc_info != NULL) {
		for (i = 0; i < dma->desc_count; i++) {
			page = &dma->desc_info[i];
			if (page->size != 0)
				pci_free_consistent(hwdev, page->size, page->data, page->addr);
		}
		kfree(dma->desc_info);
	}
	kfree(dma);
}

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
#include "pt3_i2c_bus.h"
#include "pt3_dma.h"

#define DMA_PAGE_SIZE			4096
#define MAX_DESCS			204		/* 4096 / 20 */
#define BUFF_PER_WRITE		32
#define WRITE_SIZE			(DMA_PAGE_SIZE * 47 * 8)
#define DMA_TS_BUF_SIZE		(WRITE_SIZE * BUFF_PER_WRITE)
#define NOT_SYNC_BYTE		0x74

static void
dma_write_descriptor(__u64 ts_addr, __u32 size, __u64 next_addr, PT3_DMA_DESC *page)
{
	page->page_addr = ts_addr | 7;
	page->page_size = size | 7;
	page->next_addr = next_addr | 2;
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
			if (desc_remain < 20) {
				curr = (PT3_DMA_DESC *)desc_info->data;
				desc_addr = desc_info->addr;
				desc_remain = desc_info->size;
				desc_info++;
			}
			if (prev != NULL)
				prev->next_addr = desc_addr | 2;
			dma_write_descriptor(ts_addr, DMA_PAGE_SIZE, 0, curr);
			ts_addr += DMA_PAGE_SIZE;

			prev = curr;
			curr++;
			desc_addr += sizeof(*curr);
			desc_remain -= sizeof(*curr);
		}
	}

	if (prev != NULL)
		prev->next_addr = dma->desc_info->addr | 2;
}

void __iomem *
get_base_addr(PT3_DMA *dma)
{
	return dma->bus->bar[0].regs + REGS_DMA_DESC_L + 0x18 * dma->tuner_index;
}

void
pt3_dma_set_test_mode(PT3_DMA *dma, int test, __u16 init, int not, int reset)
{
	void __iomem *base;
	__u32 data;

	base = get_base_addr(dma);
	data = (reset ? 1: 0) << 18 | (not ? 1 : 0) << 17 | (test ? 1 : 0) << 16 | init;

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
		printk(KERN_DEBUG "enable dma tuner_index=%d", dma->tuner_index);
		pt3_dma_reset(dma);
		writel( 1 << 1, base + 0x08);
		writel(BIT_SHIFT_MASK(start_addr,  0, 32), base + 0x0);
		writel(BIT_SHIFT_MASK(start_addr, 32, 32), base + 0x4);
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
		/*
		ready = pt3_dma_ready(dma);
		if (!ready)
			break;
		*/

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
		printk(KERN_DEBUG "copy_to_user size=%d ts_pos=%d data_size = %d data_pos=%d",
				rsize, dma->ts_pos, page->size, page->data_pos);
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

PT3_DMA *
create_pt3_dma(struct pci_dev *hwdev, PT3_I2C_BUS *bus, int tuner_index)
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
	dma->bus = bus;
	dma->tuner_index = tuner_index;
	mutex_init(&dma->lock);
	
	dma->ts_count = DMA_TS_BUF_SIZE / (DMA_PAGE_SIZE * BUFF_PER_WRITE);
	dma->ts_info = kzalloc(sizeof(PT3_DMA_PAGE) * dma->ts_count, GFP_KERNEL);
	if (dma->ts_info == NULL) {
		printk(KERN_ERR "fail allocate PT3_DMA_PAGE");
		goto fail;
	}
	for (i = 0; i < dma->ts_count; i++) {
		page = &dma->ts_info[i];
		page->size = DMA_PAGE_SIZE * BUFF_PER_WRITE;
		page->data = pci_alloc_consistent(hwdev, page->size, &page->addr);
		if (page->data == NULL) {
			printk(KERN_ERR "fail allocate consistent. %d", i);
			goto fail;
		}
	}
	printk(KERN_DEBUG "Allocate TS buffer.");

	dma->desc_count = (DMA_TS_BUF_SIZE / (PAGE_SIZE * BUFF_PER_WRITE) + 203) / 204;
	dma->desc_info = kzalloc(sizeof(PT3_DMA_PAGE) * dma->desc_count, GFP_KERNEL);
	if (dma->desc_info == NULL) {
		printk(KERN_ERR "fail allocate PT3_DMA_PAGE");
		goto fail;
	}
	for (i = 0; i < dma->desc_count; i++) {
		page = &dma->desc_info[i];
		page->size = DMA_PAGE_SIZE * BUFF_PER_WRITE;
		page->data = pci_alloc_consistent(hwdev, page->size, &page->addr);
		if (page->data == NULL) {
			printk(KERN_ERR "fail allocate consistent. %d", i);
			goto fail;
		}
	}
	printk(KERN_DEBUG "Allocate Descriptor buffer.");
	
	dma_build_page_descriptor(dma);

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

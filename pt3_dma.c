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
	curr = desc_info->data;
	prev = NULL;
	desc_info++;
	for (i = 0; i < dma->ts_count; i++) {
		ts_addr = ts_info->addr;
		ts_size = ts_info->size;
		ts_info++;
		for (j = 0; j < ts_size / DMA_PAGE_SIZE; j++) {
			if (desc_remain < 20) {
				curr = desc_info->data;
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

PT3_DMA *
create_pt3_dma(struct pci_dev *hwdev)
{
	PT3_DMA *dma;
	PT3_DMA_PAGE *page;
	__u32 i;

	dma = kzalloc(sizeof(PT3_DMA), GFP_KERNEL);
	if (dma == NULL) {
		printk(KERN_ERR "fail allocate PT3_DMA");
		goto fail;
	}
	
	dma->ts_count = DMA_TS_BUF_SIZE / (DMA_PAGE_SIZE * 4);
	dma->ts_info = kzalloc(sizeof(PT3_DMA_PAGE) * dma->ts_count, GFP_KERNEL);
	if (dma->ts_info == NULL) {
		printk(KERN_ERR "fail allocate PT3_DMA_PAGE");
		goto fail;
	}
	for (i = 0; i < dma->ts_count; i++) {
		page = &dma->ts_info[i];
		page->size = DMA_PAGE_SIZE * 4;
		page->data = pci_alloc_consistent(hwdev, page->size, &page->addr);
		if (page->data == NULL) {
			printk(KERN_ERR "fail allocate consistent. %d", i);
			goto fail;
		}
	}
	printk(KERN_DEBUG "Allocate TS buffer.");

	dma->desc_count = (DMA_TS_BUF_SIZE / (PAGE_SIZE * 4) + 203) / 204;
	dma->desc_info = kzalloc(sizeof(PT3_DMA_PAGE) * dma->desc_count, GFP_KERNEL);
	if (dma->desc_info == NULL) {
		printk(KERN_ERR "fail allocate PT3_DMA_PAGE");
		goto fail;
	}
	for (i = 0; i < dma->desc_count; i++) {
		page = &dma->desc_info[i];
		page->size = DMA_PAGE_SIZE * 4;
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

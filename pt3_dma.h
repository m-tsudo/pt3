#ifndef _H_PT3_DMA
#define _H_PT3_DMA

#include "pt3_com.h"
#include "pt3_bus.h"
#include "pt3_i2c.h"

typedef struct _PT3_DMA_PAGE {
	dma_addr_t addr;
	__u8 *data;
	__u32 size;
	__u32 data_pos;
} PT3_DMA_PAGE;

typedef struct __PT3_DMA {
	PT3_I2C *i2c;
	int real_index;
	int enabled;
	__u32 desc_count;
	PT3_DMA_PAGE *desc_info;
	__u32 ts_count;
	PT3_DMA_PAGE *ts_info;
	__u32 ts_pos;
	struct mutex lock;
} PT3_DMA;

void pt3_dma_build_page_descriptor(PT3_DMA *dma, int loop);
void pt3_dma_set_test_mode(PT3_DMA *dma, int test, __u16 init, int not, int reset);
void pt3_dma_set_enabled(PT3_DMA *dma, int enabled);
ssize_t pt3_dma_copy(PT3_DMA *dma, char __user *buf, size_t size, loff_t *ppos, int look_ready);
int pt3_dma_ready(PT3_DMA *dma);
void pt3_dma_reset(PT3_DMA *dma);
__u32 pt3_dma_get_status(PT3_DMA *dma);
__u32 pt3_dma_get_ts_error_packet_count(PT3_DMA *dma);
PT3_DMA * create_pt3_dma(struct pci_dev *hwdev, PT3_I2C *i2c, int tuner_no);
void free_pt3_dma(struct pci_dev *hwdev, PT3_DMA *dma);

#endif

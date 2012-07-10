#ifndef _H_PT3_DMA
#define _H_PT3_DMA

typedef struct _PT3_DMA_DESC {
	__u64 page_addr;
	__u32 page_size;
	__u64 next_addr;
} PT3_DMA_DESC;

typedef struct _PT3_DMA_PAGE {
	dma_addr_t addr;
	void *data;
	__u32 size;
} PT3_DMA_PAGE;

typedef struct __PT3_DMA {
	__u32 desc_count;
	PT3_DMA_PAGE *desc;
	__u32 ts_count;
	PT3_DMA_PAGE *ts;
} PT3_DMA;

PT3_DMA * create_pt3_dma(struct pci_dev *hwdev);
void free_pt3_dma(struct pci_dev *hwdev, PT3_DMA *dma);

#endif

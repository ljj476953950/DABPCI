#ifndef _DABPCI_H_
#define _DABPCI_H_

#define PCI_VENDOR_ID_DAB 0x1172
#define PCI_DEVICE_ID_DAB 0x0004

#define DAB_NAME "Tongshi_DAB"
#define DAB_CLASS_NAME "Tongshi_DAB_BOARD"

#define DMA_BLOCK	256
#define DMA_BLOCK_SIZE	(4*1024)
#define DMA_LENGTH 	(DMA_BLOCK*DMA_BLOCK_SIZE)			// the circular buffer

#define TSDAB_IOC_MAGIC  'k'          // magic is 'k'
#define DmaSet  		  _IO(TSDAB_IOC_MAGIC,1)
#define SETQamFrequency  _IO(TSDAB_IOC_MAGIC,2)
#define SoftReset        _IO(TSDAB_IOC_MAGIC,3)
#define DmaEnable        _IO(TSDAB_IOC_MAGIC,4)
#define TSDAB_IOC_MAXNR	4            // the maximum of definition command word, do nothing if more than it.

#define RD_DMA_CTL	0x00<<2
#define RD_DMA_ADR 0x01<<2
#define RD_DMA_WR_P	0x07<<2

struct dab_dev {

    struct pci_dev *pci_dev;
    void *data;
	
	int dma_read_done;
	
	dma_addr_t dma_handle;			//	physical address of DMA
	char *dma_addr;					//	virtual address of DMA

    void __iomem *mmio_bar0; 
    unsigned long mmio_start;
    unsigned long mmio_end;
    unsigned long mmio_flags;
    unsigned long mmio_len;
	
	// the circular buffer by BLOCK
	int write_ptr;					//	pointer to write memory
	int read_ptr;					//	pointer to read memory
	 							//	
    
    int m_irq;
	wait_queue_head_t wq;
	int flag;						// 1:empty;	0:full

};

#endif


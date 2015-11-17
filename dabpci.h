#ifndef _DABPCI_H_
#define _DABPCI_H_

#define PCI_VENDOR_ID_DAB 0x1172
#define PCI_DEVICE_ID_DAB 0x4258

#define DAB_NAME "Tongshi_DAB"
#define DAB_CLASS_NAME "Tongshi_DAB_BOARD"

#define TSDAB_IOC_MAGIC  'k'          // 定义的幻数
#define DmaSet  		  _IO(TSDAB_IOC_MAGIC,1)
#define SETQamFrequency  _IO(TSDAB_IOC_MAGIC,2)
#define SoftReset        _IO(TSDAB_IOC_MAGIC,3)
#define DmaEnable        _IO(TSDAB_IOC_MAGIC,4)
#define TSDAB_IOC_MAXNR	4            // 定义命令字的最大值 超过就不处理

struct dab_dev {

    struct pci_dev *pci_dev;
    void *data;

    void __iomem *mmio_addr; 
    unsigned long mmio_start;
    unsigned long mmio_end;
    unsigned long mmio_flags;
    unsigned long mmio_len;
    
    int m_irq;

};

#endif


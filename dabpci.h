#ifndef _DABPCI_H_
#define _DABPCI_H_

#define PCI_VENDOR_ID_DAB 0x1100
#define PCI_DEVICE_ID_DAB 0x4258

#define DAB_NAME "Tongshi_DAB"
#define DAB_CLASS_NAME "Tongshi_DAB_BOARD"

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


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/ioctl.h>
#include <linux/dma-mapping.h>
#include "dabpci.h"


int dabpci_minor = 0;
int dabpci_nr_devs = 1;
int dabpci_major = 0;

static struct dab_dev *DABDrv;

static int DABDrv_w32(int addr, int data)
{
	iowrite32(data,DABDrv->mmio_bar0 + addr);
	return 0;
}

static int DABDrv_Open(struct inode *inode,struct file *filp)
{
//	iowrite32(0x00000001,DABDrv->mmio_bar0 +29);
    return 0;
}

static int DABDrv_Release(struct inode *inode, struct file *filp)
{
    return 0;
}

static long  DABDrv_IOControl(struct file* filp, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	int i = 0;
	unsigned int arg1 = arg;

	unsigned int dwData;
         printk(KERN_INFO "in IOCTL\n");
         printk(KERN_INFO "in IOCTL\n");

	if (_IOC_TYPE(cmd) != TSDAB_IOC_MAGIC)                 
		return -ENOTTY;
	if (_IOC_NR(cmd) > TSDAB_IOC_MAXNR)
		return -ENOTTY;

   if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)

	if (err)
		return -EFAULT;					

	switch(cmd)
	{
	case DmaSet:
//		iowrite32(0x0000000f,DABDrv->mmio_bar0+arg);	
	
	}		

		
    return 0;
}




static ssize_t DABDrv_Write(struct file* filp, const char __user *buf, size_t count, loff_t *f_pos)
{
	int err = -EINVAL;
	void * virt_addr = NULL;
	dma_addr_t dma_write_addr;
	
	if(count>DMA_LENGTH)
		return 0;
	
	if(unlikely(copy_from_user(DABDrv->dma_addr,buf,count)))
		return err;
		
//	dma_write_addr = pci_map_single(DABDrv->pci_dev,virt_addr,count,PCI_DMA_TODEVICE);
	
	DABDrv_w32(RD_DMA_ADR,DABDrv->dma_handle);
	DABDrv_w32(RD_DMA_SIZE,count>>12);
	DABDrv_w32(RD_DMA_CTL,0x1);


    return count;
}

static ssize_t DABDrv_Read(struct file* filp, char __user *buf, size_t count, loff_t *f_pos)
{
    return 0;
}

struct file_operations DABDrv_fops = {
    .owner = THIS_MODULE,
    .open = DABDrv_Open,
    .release = DABDrv_Release,
    .unlocked_ioctl = DABDrv_IOControl,
    .read = DABDrv_Read,
    .write = DABDrv_Write,
};

static struct pci_device_id dabpci_ids[] = {
    {PCI_DEVICE(PCI_VENDOR_ID_DAB,PCI_DEVICE_ID_DAB)},
    {0,},
};
MODULE_DEVICE_TABLE(pci,dabpci_ids);

irqreturn_t DABDrv_interrupt(int irq,void *dev)
{
	printk(KERN_INFO "interrupt DAB PCIe.");
    return IRQ_HANDLED;
}

static int dabpci_probe(struct pci_dev *dev, const struct pci_device_id *id)
{
    int result;
    printk(KERN_INFO "initialize DAB PCIe.");
    printk(KERN_INFO "the vendor:0x%x.\n the device:0x%x.\n",dev->vendor,dev->device);

    /*adapt the space for private*/
    DABDrv = kmalloc(sizeof(struct dab_dev),GFP_KERNEL);

    if(unlikely(!DABDrv))
        return -ENOMEM;
    
    DABDrv->pci_dev = dev;

    /* Enable PCI*/
    result = pci_enable_device(dev);
    
    if(unlikely(result))
        goto disable_pci;
    /*set the PCI DMA mode*/
    pci_set_master(dev);
    result = pci_set_dma_mask(dev, DMA_BIT_MASK(32));
    if(unlikely(result))
    {
        printk( KERN_ERR "DABPCI: 32-bits PCI DMA address not supported!\n");
        goto disable_pci;
    }
    else
        printk( KERN_INFO "DABPCI: set DMA mask successfully!\n");

    /*request I/O resource */

    DABDrv->mmio_start = pci_resource_start(dev,0);
    DABDrv->mmio_end = pci_resource_end(dev,0);
    DABDrv->mmio_flags = pci_resource_flags(dev,0);
    DABDrv->mmio_len = pci_resource_len(dev,0);

    if(unlikely(!(DABDrv->mmio_flags & IORESOURCE_MEM)))
    {
        printk(KERN_ERR "DABPCI: Failed to allocate IO memory!\n");
    }


    result = pci_request_regions(dev, DAB_NAME);
    if(unlikely(result))
    {
        printk(KERN_ERR "DABPCI: Failed to request IO space!\n");
        goto disable_pci;
    }
    
    DABDrv->mmio_bar0 = ioremap(DABDrv->mmio_start,DABDrv->mmio_len);
    if(unlikely(!DABDrv->mmio_bar0))
    {
        result = -EIO;
        printk(KERN_ERR "DABPCI: Failed to do IO remap!\n");
    }

    /*enable MSI*/
    result = pci_enable_msi(dev);
    if(unlikely(result))
    {
        printk(KERN_ERR "DABPCI: Failed to enable MSI!\n");
        return result;
    }

    DABDrv->m_irq = dev->irq;
    result = request_irq(DABDrv->m_irq,DABDrv_interrupt,IRQF_DISABLED,DAB_NAME,NULL);
    if(unlikely(result))
    {
        printk(KERN_ERR "DABPCI: Failed to request interrupt.\n");
        goto disable_pci;
    }

    dabpci_major = register_chrdev(0,DAB_NAME,&DABDrv_fops);
    if(dabpci_major<0)
    {
        printk(KERN_ERR "DABPCI: Failed to get the major device number:%d;\n",dabpci_major);
        goto disable_pci;
    }
    else
    {
        printk(KERN_INFO "DABPCI: Get the major device number successfully,major = %d\n",dabpci_major);
    }
    
	DABDrv->dma_addr =
	    dma_alloc_coherent(NULL, DMA_LENGTH, &(DABDrv->dma_handle), GFP_KERNEL);

	printk(KERN_INFO "The virtual address is %x, the physical address is %x\n",dabpci_major);

    return 0;


    disable_pci:
        pci_disable_device(dev);
        return result;
}

static void dabpci_remove(struct pci_dev *dev)
{

    unregister_chrdev(dabpci_major,DAB_NAME);
    pci_disable_device(dev);
    kfree(DABDrv);
    printk(KERN_INFO "DAB PCIe remove done.\n");
}

static struct pci_driver dabpci_driver =
{
    .name = DAB_NAME,
    .id_table = dabpci_ids,
    .probe = dabpci_probe,
    .remove = dabpci_remove,
};

static int __init dabpci_init(void)
{
    return pci_register_driver( &dabpci_driver);
}

static void __exit  dabpci_exit(void)
{
    pci_unregister_driver(&dabpci_driver);
}

module_init( dabpci_init );
module_exit( dabpci_exit );

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Li JiaJie, Xi'an TongShi");
MODULE_VERSION("V1.0");
MODULE_DESCRIPTION("TongShi 4-Channel DAB Broadcast card");




/*************************************************************************
	> File Name: ue1000.h
	> Author: 
	> Mail: 
	> Created Time: Tue 25 Sep 2018 06:19:45 PM +08
 ************************************************************************/

#ifndef _UE1000_H
#define _UE1000_H

#include <linux/miscdevice.h>
#include "ue1000_cmd.h"

#define LOGD(fmt, ...)                                      \
    do {                                                    \
        printk("ue1000[%-20s:%-4d] "fmt,        \
                __FILE__, __LINE__, ##__VA_ARGS__);     \
    }while ( 0 )


struct udev {
    struct list_head entry;
    struct miscdevice misc;
    struct e1000_adapter *adapter;

    phys_addr_t    io_addr_phys;
    void *         io_addr_kerl;
    unsigned int   io_addr_size;

    dma_addr_t     rx_ring_phys;
    void *         rx_ring_kerl;
    unsigned long  rx_ring_size;

    dma_addr_t     tx_ring_phys;
    void *         tx_ring_kerl;
    unsigned long  tx_ring_size;

    dma_addr_t     buffers_addr_phys;
    void *         buffers_addr_kerl;
    unsigned long  buffers_addr_size;
    
};



struct list_head udevs = LIST_HEAD_INIT(udevs);

static long misc_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    struct udev *ud = filp->private_data;
    struct e1000_adapter *adapter = ud->adapter;

    LOGD("misc_ioctl...cmd: %d arg: %lx", cmd, arg);
    switch ( cmd ) {
        case IOCTL_GET_IO:
        {
            u64 *pout = (u64 *)arg;
            if ( copy_to_user(pout, &ud->io_addr_size, sizeof(ud->io_addr_size)) ) {
                return -EINVAL;
            }
        }
        return 0;
        case IOCTL_GET_RX:
        {
            u64 *pout = (u64 *)arg;
            u64 size = 0;
            if ( copy_from_user(&size, pout, sizeof(size)) ) {
                return -EINVAL;
            }

            if ( ud->rx_ring_size != size ) {
                if ( ud->rx_ring_size && ud->rx_ring_kerl) {
                    dma_free_coherent(&adapter->pdev->dev, ud->rx_ring_size,
                                      ud->rx_ring_kerl, ud->rx_ring_phys);
                    ud->rx_ring_kerl = NULL;
                    ud->rx_ring_phys = 0;
                    ud->rx_ring_size = 0;
                }
                ud->rx_ring_kerl = dma_alloc_coherent(&adapter->pdev->dev, size, 
                                                     &ud->rx_ring_phys, GFP_KERNEL);
                if ( ud->rx_ring_kerl == NULL ) {
                    LOGD("dma_alloc_coherent %lld failed.", size);
                    return -ENOMEM;
                }
                ud->rx_ring_size = size;
                LOGD("dma erea alloced: %llx", ud->rx_ring_phys);
                LOGD("dma erea phys addr from virt: %llx", virt_to_phys(ud->rx_ring_kerl));

            }
            if ( copy_to_user(pout, &ud->rx_ring_phys, sizeof(ud->rx_ring_phys)) ) {
                return -EINVAL;
            }
            return 0;
        }
        break;
        case IOCTL_GET_BUFFER:
        {
            u64 *pout = (u64 *)arg;
            u64 size = 0;
            if ( copy_from_user(&size, pout, sizeof(size)) ) {
                return -EINVAL;
            }

            if ( ud->buffers_addr_size != size ) {
                if ( ud->buffers_addr_size && ud->buffers_addr_kerl ) {
                    dma_free_coherent(&adapter->pdev->dev, ud->buffers_addr_size,
                                      ud->buffers_addr_kerl, ud->buffers_addr_phys);
                    ud->buffers_addr_kerl = NULL;
                    ud->buffers_addr_phys = 0;
                    ud->buffers_addr_size = 0;
                }

                ud->buffers_addr_kerl = dma_alloc_coherent(&adapter->pdev->dev, size,
                                                          &ud->buffers_addr_phys, GFP_KERNEL);
                if ( ud->buffers_addr_kerl == NULL ) {
                    LOGD("dma_alloc_coherent %lld failed.", size);
                    return -ENOMEM;
                }
                ud->buffers_addr_size = size;
                LOGD("dma erea alloced: %llx", ud->buffers_addr_phys);
                LOGD("dma erea phys addr from virt: %llx", virt_to_phys(ud->buffers_addr_kerl));
            }
            if ( copy_to_user(pout, &ud->buffers_addr_phys, sizeof(ud->buffers_addr_phys)) ) {
                return -EINVAL;
            }
            return 0;
        }
    }
    return -EINVAL;
}

static int misc_open(struct inode *inode, struct file *filp)
{
    struct list_head *pos = NULL;
    struct udev *ud = NULL;
    LOGD("misc_open... inode->i_cdev: %p, minor: %d", inode->i_cdev, iminor(inode));
    list_for_each(pos, &udevs) {
        ud = list_entry(pos, struct udev, entry);
        if ( ud->misc.minor == iminor(inode) ) {
            LOGD("udev finded, addr: %p", ud);
            filp->private_data = ud;
            e1000_down(ud->adapter);
            return 0;
        }
    }
    return -EINVAL;
}

static int misc_release(struct inode* inode, struct file *filp)
{
    struct udev *ud = filp->private_data;
    LOGD("misc_close...");
    e1000_up(ud->adapter);
    return 0;
}

static void mmap_vm_open(struct vm_area_struct *vma) {
    
}
static void mmap_vm_close(struct vm_area_struct *vma) {
    
}

static struct vm_operations_struct mmap_ops = {
    .open = mmap_vm_open,
    .close = mmap_vm_close,
};

static int misc_mmap(struct file *filp, struct vm_area_struct *vma) 
{

    struct udev *ud = filp->private_data;
    unsigned long start = vma->vm_start;
    unsigned long size  = vma->vm_end - vma->vm_start;
    unsigned long pgoff = vma->vm_pgoff;
    unsigned long pfn;

    LOGD("misc_mmap...");
    
    if ( ud == NULL ) {
        LOGD("ud == 0");
        return -EINVAL;
    }
    LOGD("pgoff: %lx", pgoff);

    if ( pgoff*SIZE_OF_PAGE == IO_ADDR_OFFSET ) {
        pfn = pci_resource_start(ud->adapter->pdev, BAR_0);
        pfn = pfn >> PAGE_SHIFT;
        vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
    }
    else if ( pgoff*SIZE_OF_PAGE == RX_RING_OFFSET ) {
        if ( !ud->rx_ring_kerl ) {
            return -EAGAIN;
        }
        pfn = page_to_pfn(virt_to_page(ud->rx_ring_kerl));
    }
    else if ( pgoff*SIZE_OF_PAGE == BUFS_ADDR_OFFSET ) {
        if ( !ud->buffers_addr_kerl  ) {
            return -EAGAIN;
        }
        pfn = page_to_pfn(virt_to_page(ud->buffers_addr_kerl));
    }
    else {
        return -EINVAL;
    }

    if ( remap_pfn_range(vma, start, pfn, size, vma->vm_page_prot) )
    {
        LOGD("remap_pfn_range failed.");
        return -EAGAIN;
    }

    vma->vm_ops = &mmap_ops;   

    return 0;
}

struct file_operations misc_fops = {
    .owner = THIS_MODULE,
    .unlocked_ioctl = misc_ioctl,
    .open  = misc_open,
    .release = misc_release,
    .mmap = misc_mmap,
};

void ue1000_print_buffer(unsigned char *buffer, int len)
{
    int ix = 0;
    for ( ix = 0; ix < len; ix +=4 ) {
        LOGD("%08x", *(unsigned int *)(buffer + ix));
    }
    return;
}

void ue1000_print_info(struct e1000_adapter *adapter) 
{
    struct e1000_hw *hw = &adapter->hw;
    LOGD("name: %s", adapter->netdev->name);
    if ( adapter->netdev->ifalias ) {
        LOGD("ifalias: %s", adapter->netdev->ifalias);
    }
    LOGD("ifindex: %d", adapter->netdev->ifindex);
    LOGD("adapter->hw.hw_addr: 0x%p", adapter->hw.hw_addr);
    LOGD("RCTL: %08x", er32(RCTL));
    LOGD("adapter->hd.hd_addr[0]: 0x%08x", *(unsigned int *)(adapter->hw.hw_addr));
    LOGD("bar0 size: %llu", pci_resource_len(adapter->pdev, BAR_0));
    LOGD("bar0 base: %llx", pci_resource_start(adapter->pdev, BAR_0));
    LOGD("pfn of base: %lx", page_to_pfn(virt_to_page(adapter->hw.hw_addr)));

    ue1000_print_buffer(adapter->hw.hw_addr, 16);
}

struct udev* ue1000_dev_alloc(struct e1000_adapter *adapter)
{
    int ret = 0;
    int ifindex = adapter->netdev->ifindex;
    struct udev *ud = NULL;

    ud = kmalloc(sizeof(*ud), GFP_KERNEL);
    if ( ud == NULL ) {
        LOGD("kmalloc failed.");
        return NULL;
    }
    memset(ud, 0, sizeof(*ud));
    ud->adapter = adapter;

    ud->misc.minor = MISC_DYNAMIC_MINOR;
    ud->misc.name = kmalloc(256, GFP_KERNEL);
    snprintf((char *)ud->misc.name, 256, "ue1000_%d", ifindex);
    ud->misc.fops = &misc_fops;

    ud->io_addr_phys = pci_resource_start(adapter->pdev, BAR_0);
    ud->io_addr_size = pci_resource_len(adapter->pdev, BAR_0);
    ud->io_addr_kerl = adapter->hw.hw_addr;

    ret = misc_register(&ud->misc);
    if ( ret != 0 ) {
        LOGD("misc_regitster failed.");
    }
    LOGD("misc_register succeed, addr: %p, minor: %d", &ud->misc, ud->misc.minor);

    list_add(&ud->entry, &udevs);

    return ud;
}

int ue1000_dev_free(struct e1000_adapter *adapter)
{
    struct list_head *pos = NULL;
    struct list_head *nxt = NULL;
    struct udev *ud = NULL;

    list_for_each_safe(pos, nxt, &udevs) {
        ud = list_entry(pos, struct udev, entry);
        if ( ud->adapter == adapter ) {
            misc_deregister(&ud->misc);
            kfree(ud->misc.name);
            list_del(&ud->entry);
            kfree(ud);
            return 0;
        }
    }
    return -1;
}

int ue1000_init(struct e1000_adapter *adapter)
{
    ue1000_print_info(adapter);
    ue1000_dev_alloc(adapter);
    return 0;
}

int ue1000_deinit(struct e1000_adapter *adapter) {
    ue1000_dev_free(adapter);
    return 0;
}

#endif

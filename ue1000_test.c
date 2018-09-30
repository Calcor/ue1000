/*************************************************************************
	> File Name: test.c
	> Author: 
	> Mail: 
	> Created Time: Thu 27 Sep 2018 12:14:25 AM +08
 ************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <errno.h>

#include "ue1000_cmd.h"
#include "ue1000_hw.h"
#include "ue1000_buf.h"


#define BUFFERS_SIZE (4*1024*1024)
#define BUF_SIZE 2048


struct instance {
    int fd;
    e1000_mac_type mac_type;

    void *        hw_addr;
    unsigned long hw_size;

    void *             rx_ring_virt;
    unsigned long long rx_ring_phys;
    unsigned long      rx_ring_size;

    unsigned long      rx_desc_num;

    unsigned long      head;
    unsigned long      tail;

    void *             buffers_virt;
    unsigned long long buffers_phys;
    unsigned long      buffers_size;
};

struct instance inst = {0};

struct buf_stack *pstack = NULL;

int __init_rx_ring()
{
    unsigned long ix = 0;
    inst.rx_desc_num = inst.rx_ring_size/sizeof(struct e1000_rx_desc);
    struct e1000_rx_desc *pdesc = (struct e1000_rx_desc *)inst.rx_ring_virt;
    printf("%llx", pstack->phys_base);
    for ( ix = 0; ix < inst.rx_desc_num; ix ++ ) {
        void *buffer = buf_stack_pop(pstack);
        //pdesc[ix].buffer_addr = pstack->phys_base;
        pdesc[ix].buffer_addr = buf_stack_virt_to_phys(pstack, buffer);
    }
    return 0;
}

int enable_recv()
{
    unsigned long rctl;

    rctl = E1000_READ_REG(&inst, RCTL);
    rctl &= ~(E1000_RCTL_EN | E1000_RCTL_UPE |E1000_RCTL_MPE);
    E1000_WRITE_REG(&inst, RCTL, rctl);

    E1000_WRITE_REG(&inst, RDBAL, inst.rx_ring_phys & 0xffffffff);
    E1000_WRITE_REG(&inst, RDBAH, inst.rx_ring_phys >>32 );

    E1000_WRITE_REG(&inst, RDLEN, inst.rx_ring_size);

    E1000_WRITE_REG(&inst, RDT, 0);
    E1000_WRITE_REG(&inst, RDH, 0);

    inst.head = 0;
    inst.tail = 254;
    
    rctl &= ~(E1000_RCTL_LPE);

    rctl &= ~E1000_RCTL_SZ_4096;
    rctl &= ~E1000_RCTL_BSEX;
    rctl |=  E1000_RCTL_SZ_2048;

    rctl |= (E1000_RCTL_EN | E1000_RCTL_BAM | E1000_RCTL_LBM_NO | 
             E1000_RCTL_RDMTS_HALF );

    E1000_WRITE_REG(&inst, RCTL, rctl);

    E1000_WRITE_REG(&inst, RDT, 254);
    E1000_WRITE_FLUSH(&inst);
    return 0;
}

size_t try_recv(void **pbuffer)
{
    struct e1000_rx_desc *pdesc = (struct e1000_rx_desc *)inst.rx_ring_virt;
    if ( pdesc[inst.head].status & E1000_RXD_STAT_DD ) {
        size_t len = pdesc[inst.head].length;
        *pbuffer = (void *)buf_stack_phys_to_virt(pstack, pdesc[inst.head].buffer_addr);

        void *buffer = buf_stack_pop(pstack);
        pdesc[inst.head].buffer_addr = buf_stack_virt_to_phys(pstack, buffer);
        pdesc[inst.head].status = 0;

        inst.head = (inst.head + 1) %inst.rx_desc_num;
        inst.tail = (inst.tail + 1) %inst.rx_desc_num;

        E1000_WRITE_REG(&inst, RDH, inst.head);
        E1000_WRITE_REG(&inst, RDT, inst.tail);
        return len;
    }
    return 0;
}

void print_regs()
{
    printf("\n");
    printf("e1000_ctrl: 0x%08x\n", *(unsigned int *)inst.hw_addr);
    printf("e1000_RCTL:  0x%08x\n", *(unsigned int *)(inst.hw_addr+0x100));
    printf("e1000_RDBAL: 0x%08x\n", *(unsigned int *)(inst.hw_addr+0x2800));
    printf("e1000_RDBAH: 0x%08x\n", *(unsigned int *)(inst.hw_addr+0x2804));
    printf("e1000_rdlen: 0x%08x\n", *(unsigned int *)(inst.hw_addr+0x02808));
    printf("e1000_rdh:   0x%08x\n", *(unsigned int *)(inst.hw_addr+0x02810));
    printf("e1000_rdt:   0x%08x\n", *(unsigned int *)(inst.hw_addr+0x02818));
}

void print_buffer(unsigned char * buffer, size_t len)
{
    int ix = 0;
    for ( ix = 0; ix < len; ix ++ ) {
        if ( ix%16 == 0 ) {
            printf("\n");
        }
        if ( ix%8 == 0 ) {
            printf(" ");
        }
        printf("%02x ", buffer[ix]);
    }
}

int main(int argc, char *argv[])
{

    if ( argc != 2 ) {
        printf("Usage: %s /dev/ue1000_%%d\n", argv[0]);
        return -1;
    }
    int fd = open(argv[1], O_RDWR);
    if ( inst.fd < 0 ) {
        perror("open failed.");
        return -1;
    }
    inst.fd = fd;

    unsigned long long data = 0;

    int ret = ioctl(inst.fd, IOCTL_GET_IO, &data);
    if ( ret != 0 ) {
        perror("ioctl GET_IO failed.\n");
        return -1;
    }
    printf("io size: %lld.\n", data);
    inst.hw_size = data;

    inst.hw_addr = mmap(NULL, 131072, PROT_READ|PROT_WRITE, MAP_SHARED,
                  fd, IO_ADDR_OFFSET);
    if ( inst.hw_addr == MAP_FAILED )
    {
        perror("mmap failed.");
        return -1;
    }

    while ( argc > 2 ) {
        sleep(1);
        print_regs();
    }

    data = 0x1000;
    inst.rx_ring_size = data;
    ret = ioctl(fd, IOCTL_GET_RX, &data);
    if ( ret != 0 ) {
        perror("ioctl GET_RX failed.\n");
        return -1;
    }
    printf("rx buffer phys addr: %llx\n", data);
    inst.rx_ring_phys = data;


    data = BUFFERS_SIZE;
    inst.buffers_size = data;
    ret = ioctl(fd, IOCTL_GET_BUFFER, &data);
    if ( ret != 0 ) {
        perror("ioctl GET_BUFFER failed.\n");
        return -1;
    }
    printf("buffers phys addr :%llx\n", data);
    inst.buffers_phys = data;

    

    inst.rx_ring_virt = mmap(NULL, inst.rx_ring_size, PROT_READ|PROT_WRITE, MAP_SHARED,
                  fd, RX_RING_OFFSET);
    if ( inst.rx_ring_virt == MAP_FAILED ) {
        perror("mmap ring failed.");
        return -1;
    }

    inst.buffers_virt = mmap(NULL, inst.buffers_size, PROT_READ|PROT_WRITE, MAP_SHARED,
                  fd, BUFS_ADDR_OFFSET);
    if ( inst.buffers_virt== MAP_FAILED ) {
        perror("mmap buffers failed");
        return -1;
    }
    memset(inst.buffers_virt, 0, inst.buffers_size);

    inst.mac_type = e1000_82545;

    pstack = buf_stack_alloc(inst.buffers_phys, inst.buffers_virt, inst.buffers_size,
                            BUF_SIZE);

    __init_rx_ring();
    enable_recv();

    while ( 1 )
    {
        void *buffer = NULL;
        size_t len = try_recv(&buffer);
        if ( !len ) {
            continue;
        }
        //sleep(1);
        printf("get a packet..len: %d\n", len);
        print_buffer(buffer, len);
        print_regs();
        buf_stack_push(pstack, buffer);
    }

    return 0;
}

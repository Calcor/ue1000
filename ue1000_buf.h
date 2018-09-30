/*************************************************************************
	> File Name: buffer.h
	> Author: 
	> Mail: 
	> Created Time: Fri 28 Sep 2018 01:40:23 AM +08
 ************************************************************************/

#ifndef _BUFFER_H
#define _BUFFER_H

#define UBUF_HEADROOM_SIZE 300

struct ubuf {
    char headroom[UBUF_HEADROOM_SIZE];
    char data[0];
};

struct buf_stack {
    unsigned long long phys_base;
    void * buffers_base;
    unsigned long buffers_size;
    unsigned long self_size;

    unsigned long buffer_size;
    unsigned long buffer_num;

    unsigned long stack_top;
    void *stack_base[0];
};


#define __buf_stack_push(pstack, buf) \
do {    \
    pstack->stack_base[pstack->stack_top] = buf;    \
    pstack->stack_top ++;    \
} while ( 0 )

#define __buf_stack_pop(pstack)    \
do {    \
    void *ret = pstack->stack_base[pstack->stack_top];    \
    pstack->stack_top --;    \
    return ret;    \
} while ( 0 )


#define buf_stack_virt_to_phys(pstack, addr) (addr - pstack->buffers_base + pstack->phys_base)

#define buf_stack_phys_to_virt(pstack, addr) (addr - pstack->phys_base + pstack->buffers_base)

struct buf_stack* buf_stack_alloc(unsigned long long phys_base, void * buffers_base, unsigned long buffers_size,
        unsigned long buffer_size)
{
    unsigned long ix = 0;
    unsigned long buffer_num = buffers_size/buffer_size;
    unsigned long self_size = buffer_num*sizeof(void *) + sizeof(struct buf_stack);
    struct buf_stack *pstack = malloc(self_size);
    memset(pstack, 0, self_size);

    pstack->phys_base = phys_base;
    pstack->buffers_base = buffers_base;
    pstack->buffers_size = buffers_size;
    pstack->self_size = self_size;


    pstack->buffer_size = buffer_size;
    pstack->buffer_num = buffer_num;
    pstack->stack_top = 0;


    for ( ix = 0; ix < buffer_num; ix ++ ) {
        __buf_stack_push(pstack, pstack->buffers_base + ix*pstack->buffer_size);
    }

    return pstack;
}

int buf_stack_push(struct buf_stack *pstack, void *buf)
{
    __buf_stack_push(pstack, buf);
    return 0;
}

void *buf_stack_pop(struct buf_stack *pstack)
{
    if ( pstack->stack_top == 0 ) {
        return NULL;
    }

    __buf_stack_pop(pstack);
}



int buf_stack_free(struct buf_stack *pstack) {
    free(pstack);
    return 0;
}

#endif

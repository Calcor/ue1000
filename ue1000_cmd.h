/*************************************************************************
	> File Name: ue1000_cmd.h
	> Author: 
	> Mail: 
	> Created Time: Thu 27 Sep 2018 11:45:11 PM +08
 ************************************************************************/

#ifndef _UE1000_CMD_H
#define _UE1000_CMD_H

#define SIZE_OF_PAGE 4096

#define IO_ADDR_OFFSET   ( 0 * SIZE_OF_PAGE )
#define RX_RING_OFFSET   ( 1 * SIZE_OF_PAGE )
#define TX_RING_OFFSET   ( 2 * SIZE_OF_PAGE )
#define BUFS_ADDR_OFFSET ( 3 * SIZE_OF_PAGE )


#define IOCTL_GET_IO      0 
#define IOCTL_GET_RX      1
#define IOCTL_GET_TX      2
#define IOCTL_GET_BUFFER  3

#endif

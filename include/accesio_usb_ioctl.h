#ifndef _ACCESIO_USB_IOCTL_H_
#define _ACCESIO_USB_IOCTL_H_

#include <linux/ioctl.h>
#include <linux/types.h>
#define ACCESIO_MAGIC_NUM 0xE0


struct  accesio_usb_control_transfer
{
        __u8 request;
        __u16 value;
        __u16 index;
        __u16  size;
        void * data;
        int read;
};

#define ACCESIO_USB_CONTROL_XFER        _IOWR(ACCESIO_MAGIC_NUM, 1, struct accesio_usb_control_transfer *)

struct accesio_usb_bulk_transfer
{
        unsigned int pipe_index;
        void *data;
        int size;
        int *transferred;
        int read;
};

#define ACCESIO_USB_BULK_XFER        _IOWR(ACCESIO_MAGIC_NUM, 2, struct accesio_usb_bulk_transfer *)

struct accesio_usb_aiousb_info
{
        int pid;
};

/******************************************************************************
 * Used by the aiousb library to get any information needed when opening a device
 * ***************************************************************************/
#define ACCESIO_USB_AIOUSB_INFO         _IOR(ACCESIO_MAGIC_NUM, 3, struct accesio_usb_aiousb_info *)

#define ACCESIO_USB_ABORT_PIPE          _IO(ACCESIO_MAGIC_NUM, 4)

#endif
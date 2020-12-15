#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <pthread.h>

#include <stdint.h>

#include "accesio_usb_ioctl.h"

void *abort_transfer (void *descriptor)
{
        int fd = (uintptr_t) descriptor;

        sleep(3);

        ioctl(fd, ACCESIO_USB_ABORT_PIPE);

        return NULL;
}

int generic_vendor_read(int fd, unsigned char request, unsigned short value,
                        unsigned short index, unsigned long size, void *data)
{
        struct  accesio_usb_control_transfer context = {0};
        int status;

        context.request = request;
        context.value = value;
        context.index = index;
        context.size = size;
        context.data = data;
        context.read = 1;

        status = ioctl (fd, ACCESIO_USB_CONTROL_XFER, &context);

        printf("ioctl returned %d\n", status);
        return status;
}

int generic_vendor_write(int fd, unsigned char request, unsigned short value,
                        unsigned short index, unsigned long size, void *data)
{
        struct accesio_usb_control_transfer context = {0};
        int status;

        context.request = request;
        context.value = value;
        context.index = index;
        context.size = size;
        context.data = data;
        context.read = 0;

        status = ioctl (fd, ACCESIO_USB_CONTROL_XFER, &context);

        printf("ioctl returned %d\n", status);

        return status;
}

int generic_bulk_in (int fd, unsigned int pipe_index, void *data, unsigned int size,
                        int *transferred)
{
        struct accesio_usb_bulk_transfer context = {0};
        int status;

        context.pipe_index = pipe_index;
        context.data = data;
        context.size = size;
        context.transferred = transferred;
        context.read = 1;

        status = ioctl(fd, ACCESIO_USB_BULK_XFER, &context);

        printf("%s: status = %d\n", __FUNCTION__, status);

        return status;
}

#define NUM_SAMPLES 0xb0

int main (int argc, char **argv)
{
        unsigned char config_block[20];
        int fd = open("/dev/accesio/usb_aio16_16f_3", O_RDWR);
        unsigned short counts[NUM_SAMPLES];
        int transferred = 0;
        int i;
        pthread_t abort_thread;

        printf ("fd = %d\n", fd);

        if (fd < 0)
        {
                exit(1);
        }


        config_block[0] = 0x0;
        config_block[1] = 0x0;
        config_block[2] = 0x0;
        config_block[3] = 0x0;
        config_block[4] = 0x0;
        config_block[5] = 0x0;
        config_block[6] = 0x0;
        config_block[7] = 0x0;
        config_block[8] = 0x0;
        config_block[9] = 0x0;
        config_block[10] = 0x0;
        config_block[11] = 0x0;
        config_block[12] = 0x0;
        config_block[13] = 0x0;
        config_block[14] = 0x0;
        config_block[15] = 0x0;
        config_block[16] = 0x0;
        config_block[17] = 0x4;
        config_block[18] = 0xf0;
        config_block[19] = 0xa;


        printf("writing config block to device\n");
        generic_vendor_write(fd, 0xbe, 0, 0, sizeof(config_block), config_block);

        printf("spawning thread\n");
        pthread_create(&abort_thread, NULL, abort_transfer, (void *)(intptr_t)fd);

        printf("calling bulk in\n");
        generic_bulk_in(fd, 0, counts, sizeof(counts), &transferred);

        printf("transferred = %d\n", transferred);

        for ( i = 0 ; i < transferred ; i++)
        {
                if ((i != 0) && ( ! (i % 22)))
                {
                        printf("\n");
                }
                printf("0x%x  ", counts[i]);
        }

        pthread_join(abort_thread, NULL);

        return 0;
}
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include "accesio_usb_ioctl.h"


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

        if (status)
        {
                printf("ioctl returned %d\n", status);
        }
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

        if (status)
        {
                printf("ioctl returned %d\n", status);
        }

        return status;
}


int main (int *argc, char **argv)
{
        int fd = open ("/dev/accesio/usb_dio_32i_3", O_RDWR);
        long long serial_num = 0;
        unsigned long size = sizeof(serial_num);
        int status;

        //Result := GenericVendorRead(DeviceIndex, $A2, $1DF8, 0, L, @Content);

        if (fd < 0)
        {
                printf("fd = %d\n", fd);
                exit(1);
        }

        //read the serial number
        {
        status = generic_vendor_read(fd, 0xa2, 0x1df8, 0, size, &serial_num);

        printf("status = %d, serial_num = %llx\n", status, serial_num);
        }

        //configure all bits as output and set them to one
        {
        unsigned char data[6] = {0};
        data[0] = 0xff;
        data[4] = 0xff;
        //memset(data, 0xff, sizeof(data));
        status = generic_vendor_write (fd, 0x12, 0, 0, sizeof(data), data);
        printf("status after write = %d\n", status);
        }

        

        close(fd);
        

        return 0;
}
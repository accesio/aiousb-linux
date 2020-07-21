#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include "aiousb.h"

#define err_printf(fmt, ...) \
        do { printf ("%s:%d:%s(): \n" fmt, __FILE__, \
                                __LINE__, __func__, ##__VA_ARGS__); } while (0)

int main (int arg, char **argv)
{
    int status;
    aiousb_device_handle device;

    aiousb_init();

    status = aiousb_device_handle_by_path("/dev/accesio/usb_dio_32i_3", &device);

    if (status)
    {
        err_printf("Unable to open device\n");
    }

    printf("device = %p\n", device);

    {
        unsigned char out_mask[4] = {0};
        unsigned char dio_bytes[4] = {0};

        memset(out_mask, 0xff, sizeof(out_mask));
        memset(dio_bytes, 0xff, sizeof(dio_bytes));

        status = aiousb_dio_configure(device, 0, out_mask, dio_bytes);

        status = aiousb_dio_write1(device, 0, 0);
        sleep(3);
        status = aiousb_dio_write1(device, 0, 1);
        sleep(3);
        status = aiousb_dio_write1(device, 0, 0);
        sleep(3);
        status = aiousb_dio_write1(device, 0, 1);
        sleep(3);
    }

    {
        unsigned char dio_bytes[4] = {0};
        unsigned char a_byte = 0x0;

        status = aiousb_dio_read_all(device, dio_bytes);

        printf("break\n");

        status = aiousb_dio_read_8(device, 0, &a_byte);

        printf("break\n");
    }
}
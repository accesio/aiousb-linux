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
    
    status = aiousb_device_open("/dev/accesio/usb_dio_32i_3", &device);

    if (status)
    {
        err_printf("Unable to open device\n");
    }

    printf("device = %p\n", device);

    // {
    //     int i;
    //     unsigned char out_mask[4] = {0};
    //     unsigned char dio_bytes[4] = {0};
    //     memset(out_mask, 0xff, sizeof (out_mask));
    //     printf("Testing aiousb_dio_configure\n");
    //     for ( i = 0 ; i < 10 ; i++)
    //     {

    //         memset(dio_bytes, (i % 2) ? 0x0 : 0xff, sizeof(dio_bytes));

    //         status = aiousb_dio_configure(device, 0, out_mask, dio_bytes);
    //         sleep(1);

    //     } 
    //  }



    // {
    //     unsigned char outs[4] = {0};
    //     unsigned char out_mask[4] = {0};
    //     unsigned char data[4] = {0};
    //     unsigned char data_mask[4] = {0};
    //     unsigned char tristates[4] = {0};
    //     unsigned char tristates_mask[4] = {0};

    //     memset(outs, 0xdd, sizeof(outs));
    //     memset(out_mask, 0xee, sizeof(out_mask));
    //     memset(data, 0xaa, sizeof(data));
    //     memset(data_mask, 0xbb, sizeof(data_mask));
    //     memset(tristates, 0xff, sizeof(tristates));
    //     memset(tristates_mask, 0xcc, sizeof(tristates_mask));

    //     printf("Testing aiousb_dio_configure_masked\n");


    //     aiousb_dio_configure_masked(device,
    //                                 outs,
    //                                 out_mask,
    //                                 data,
    //                                 data_mask,
    //                                 tristates,
    //                                 tristates_mask);
        
    // }

    // {
    //     memset(out_mask, 0xaa, sizeof(out_mask));
    //     memset(dio_mask, 0x55, sizeof(dio_bytes));
    //     memset(tristate, 0xba, sizeof(tristate));

    //     status = aiousb_dio_configure_masked(device

    // {
    //     unsigned char test_bytes[6] = {0};
    //     test_bytes[0] = 0xff;
    //     test_bytes[2] = 0xff;
    //     test_bytes[3] = 0xff;
    //     test_bytes[4] = 0xff;
    //     status = aiousb_generic_vendor_write(device, 0x12, 0, 0, 6, test_bytes);
    // }

    {
        unsigned char out_mask[4] = {0};
        unsigned char dio_bytes[4] = {0};

        memset(out_mask, 0xff, sizeof(out_mask));
        memset(dio_bytes, 0xff, sizeof(dio_bytes));

        status = aiousb_dio_configure(device, 0, out_mask, dio_bytes);
        
        status = aiousb_dio_write1(device, 0, 0);
    }

    {
        unsigned char dio_bytes[4] = {0};
        unsigned char a_byte = 0;

        status = aiousb_dio_read_all(device, dio_bytes);

        printf("break\n");

        status = aiousb_dio_read_8(device, 0, &a_byte);

        printf("break\n");
    }
    aiousb_device_close(device);
}
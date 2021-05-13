#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <map>

#include "aiousb.h"

#include "AiousbSamples.inc"

//24 bit cards are in 32 group
enum BitGroup {BitGroup12, BitGroup16, BitGroup32, BitGroup32i, BitGroup48_96};

std::map<uint16_t, BitGroup> PidToGroup =
{
    { 0x8001, BitGroup32 },
    { 0x0004, BitGroup32i },
    { 0x8005, BitGroup32 },
    { 0x8002, BitGroup48_96 },
    { 0x8003, BitGroup48_96 },
    { 0x8006, BitGroup32 },
    { 0x800c, BitGroup16 },
    { 0x800f, BitGroup16 },
    { 0x8032, BitGroup16 }, //Probably shouldn't be here because it's an RO
    { 0x804c, BitGroup32 },
    { 0x803c, BitGroup32 },
    { 0x8030, BitGroup12 },
    { 0x803E, BitGroup12 },
};

#define err_printf(fmt, ...) \
        do { printf ("%s:%d:%s(): \n" fmt, __FILE__, \
                                __LINE__, __func__, ##__VA_ARGS__); } while (0)

int main (int argc, char **argv)
{
    int Status;
    AIOUSB::aiousb_device_handle Device;

    AIOUSB::AiousbInit();

    Status = SampleGetDeviceHandle(argc, argv, &Device);

    if (Status)
    {
        err_printf("Unable to open device\n");
    }

    {
        unsigned char out_mask[4] = {0};
        unsigned char dio_bytes[4] = {0};

        memset(out_mask, 0xff, sizeof(out_mask));
        memset(dio_bytes, 0xff, sizeof(dio_bytes));

        Status = AIOUSB::DIO_Configure(Device, 0, out_mask, dio_bytes);

        Status = AIOUSB::DIO_Write1(Device, 0, 0);
        sleep(3);
        Status = AIOUSB::DIO_Write1(Device, 0, 1);
        sleep(3);
        Status = AIOUSB::DIO_Write1(Device, 0, 0);
        sleep(3);
        Status = AIOUSB::DIO_Write1(Device, 0, 1);
        sleep(3);
    }

    {
        unsigned char dio_bytes[4] = {0};
        unsigned char a_byte = 0x0;

        Status = AIOUSB::DIO_ReadAll(Device, dio_bytes);

        printf("break\n");

        Status = AIOUSB::DIO_Read8(Device, 0, &a_byte);

        printf("break\n");
    }
}
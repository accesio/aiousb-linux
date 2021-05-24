#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <iomanip>

#include <map>

#include "aiousb.h"

#include "AiousbSamples.inc"

//24 bit cards are in 32 group
enum TestGroup {TestGroup32, TestGroup32i, TestGroup48_96, TestGroupNone};

std::map<std::string, TestGroup> TestToGroup =
{
    { "./aiousb-dio32-test", TestGroup32 },
    { "./aiousb-dio32i-test", TestGroup32i },
    { "./aiousb-dio96-test", TestGroup48_96 },
    { "./aiousb-dio-test", TestGroupNone },
};

#define err_printf(fmt, ...) \
        do { printf ("%s:%d:%s(): \n" fmt, __FILE__, \
                                __LINE__, __func__, ##__VA_ARGS__); } while (0)

int main (int argc, char **argv)
{
    int Status;
    AIOUSB::aiousb_device_handle Device;
    int DioBytes;

    AIOUSB::AiousbInit();

    std::map<std::string, TestGroup>::iterator Group = TestToGroup.find(argv[0]);

    switch (Group->second)
    {
        case TestGroup32:
        std::cout << "TestGroup32" << std::endl;
        DioBytes = 4;
        break;

        case TestGroup32i:
        std::cout << "TestGroup32i" << std::endl;
        DioBytes = 4;
        break;

        case TestGroup48_96:
        std::cout << "TestGroup58_96" << std::endl;
        DioBytes = 8;
        break;

        case TestGroupNone:
        std::cout << "Run as bit specific test eg aiousb-dio-test aiousb-dio32-test" << std::endl;
        exit(-1);
        break;
    };



    Status = SampleGetDeviceHandle(argc, argv, &Device);

    if (Status)
    {
        err_printf("Unable to open device\n");
    }

    uint8_t *OutMask= new uint8_t[DioBytes];
    uint8_t *DioData = new uint8_t[DioBytes];
    memset(OutMask, 0xff, DioBytes);
    memset(DioData, 0xff, DioBytes);
    Status = AIOUSB::DIO_Configure(Device, 0, OutMask, DioData);

    std::cout << "Toggling bits with DIO_write1()" << std::endl;
    for (int i = 0 ; i < 3 ; i++)
    {
        for (int j = 0 ; j < DioBytes * 8 ; j++)
        {
            Status = AIOUSB::DIO_Write1(Device, j, 1);
        }
        sleep (3);
        for (int j = 0 ; j < DioBytes * 8 ; j++)
        {
            Status = AIOUSB::DIO_Write1(Device, j, 0);
        }
        sleep(3);
    }

    std::cout << "Toggling bits with DIO_Write8()" << std::endl;
    for (int i = 0 ; i < 3 ; i++)
    {
        for (int j = 0 ; j < DioBytes; j++)
        {
           Status = AIOUSB::DIO_Write8(Device, j, 0xff);
        }
        sleep (3);
        for (int j = 0 ; j < DioBytes * 8 ; j++)
        {
            Status = AIOUSB::DIO_Write8(Device, j, 0x0);
        }
        sleep(3);
    }

     std::cout << "Toggling bits with DIO_WriteAll()" << std::endl;
    for (int i = 0 ; i < 3 ; i++)
    {
        memset(DioData, 0xff, DioBytes);
        for (int j = 0 ; j < DioBytes; j++)
        Status = AIOUSB::DIO_WriteAll(Device, DioData);
        sleep (3);
        memset(DioData, 0xff, DioBytes);
        Status = AIOUSB::DIO_WriteAll(Device, DioData);
        sleep(3);
    }

    memset(OutMask, 0x00, DioBytes);
    Status = AIOUSB::DIO_Configure(Device, 0, OutMask, DioData);

    std::cout << "Reading inputs with DIO_Read1()" << std::endl;
    for (int i = 0; i < 3; i++)
    {
        memset(DioData, 0, DioBytes);
        for (int j = 0 ; j < DioBytes * 8 ; j++)
        {
            uint8_t Reading;
            AIOUSB::DIO_Read1(Device, j, &Reading);
            DioData[j/8] |= Reading << (j % 8);
        }
        sleep (3);
        for (int j = 0 ; j < DioBytes; j++)
        {
            std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)DioData[j] << std::endl;
        }
    }
    std::cout << "Reading inputs with DIO_Read8()" << std::endl;
    for (int i = 0 ; i < 3 ; i++)
    {
        for (int j = 0 ; j < DioBytes ; j++)
        {
            AIOUSB::DIO_Read8(Device, j, &(DioData[j]));
        }
        sleep(3);
        for (int j = 0 ; j < DioBytes; j++)
        {
            std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)DioData[j] << std::endl;
        }
    }

    std::cout << "Reading inputs with DIO_ReadAll()" << std::endl;
    for (int i = 0 ; i < 3 ; i++)
    {
        AIOUSB::DIO_ReadAll(Device, DioData);
        sleep(3);
        for (int j = 0 ; j < DioBytes; j++)
        {
            std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)DioData[j] << std::endl;
        }
    }

}
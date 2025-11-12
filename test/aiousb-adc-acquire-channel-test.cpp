#include <stdio.h>
#include <stdlib.h>
#include "aiousb.h"

#define NUM_SAMPLES 1024
#define FREQUENCY 1000.0

#define err_printf(fmt, ...) \
        do { printf ("%s:%d:%s(): " fmt "\n", __FILE_NAME__, \
                                __LINE__, __func__, ##__VA_ARGS__); } while (0)


int main (int arg, char **argv)
{
    int status;
    AIOUSB::aiousb_device_handle device;
    uint8_t Config[32];
    uint32_t ConfigSize = sizeof(Config);
    uint16_t data_buffer[sizeof (uint16_t) * NUM_SAMPLES] = {0};

    AIOUSB::AiousbInit();
    
    status = AIOUSB::DeviceHandleByIndex(diOnly, &device);

    if (status)
    {
        err_printf("Unable to open device\n");
        exit(1);
    }


    status = AIOUSB::ADC_AcquireChannel(device,
                                        2,
                                        1000.0,
                                        NUM_SAMPLES,
                                        data_buffer);
    err_printf("status(ADC_AcquireChannel) = %d", status);
    return 0;
}
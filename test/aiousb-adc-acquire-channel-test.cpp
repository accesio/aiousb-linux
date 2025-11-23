#include <stdio.h>
#include <stdlib.h>
#include "aiousb.h"

#define NUM_SAMPLES 1024
#define FREQUENCY 1000.0
#define CHANNEL 2
#define GAIN_CODE 0x80

#define err_printf(fmt, ...) \
        do { printf ("%s:%d:%s(): " fmt "\n", __FILE_NAME__, \
                                __LINE__, __func__, ##__VA_ARGS__); } while (0)

void dump_counts(uint16_t *data_buffer, int columns)
{
    for (int i = 0; i < NUM_SAMPLES; i++)
    {
        printf("%5u ", data_buffer[i]);
        if ((i + 1) % columns == 0)
        {
            printf("\n");
        }
    }
    printf("\n");
}

void dump_volts(double *data_buffer, int columns)
{
    for (int i = 0; i < NUM_SAMPLES; i++)
    {
        printf("%8.4f ", data_buffer[i]);
        if ((i + 1) % columns == 0)
        {
            printf("\n");
        }
    }
    printf("\n");
}


int main (int arg, char **argv)
{
    int status;
    AIOUSB::aiousb_device_handle device;
    uint8_t Config[32];
    uint32_t ConfigSize = sizeof(Config);
    uint16_t data_buffer[sizeof (uint16_t) * NUM_SAMPLES] = {0};
    double data_buffer_v[sizeof (double) * NUM_SAMPLES] = {0};

    AIOUSB::AiousbInit();
    
    status = AIOUSB::DeviceHandleByIndex(diOnly, &device);

    if (status)
    {
        err_printf("Unable to open device\n");
        exit(1);
    }

    for (int i = 0; i < NUM_SAMPLES; i++)
    {
        data_buffer[i] = i;
    }

    status = AIOUSB::ADC_AcquireChannel(device,
                                        CHANNEL,
                                        GAIN_CODE,
                                        FREQUENCY,
                                        NUM_SAMPLES,
                                        data_buffer);
    err_printf("status(ADC_AcquireChannel) = %d", status);
    dump_counts(data_buffer, 16);

    status = AIOUSB::ADC_AcquireChannelV(device,
                                        CHANNEL,
                                        GAIN_CODE,
                                        FREQUENCY,
                                        NUM_SAMPLES,
                                        data_buffer_v);
    err_printf("status(ADC_AcquireChannelV) = %d", status);
    dump_volts(data_buffer_v, 8);
    return 0;
}
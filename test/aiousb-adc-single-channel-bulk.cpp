#include <stdio.h>>

#include "aiousb.h"
#include "AiousbSamples.inc"

#define err_printf(fmt, ...) \
        do { printf ("%s:%d:%s(): " fmt "\n", __FILE_NAME__, \
                                __LINE__, __func__, ##__VA_ARGS__); } while (0)

void SampleChannel(AIOUSB::aiousb_device_handle dev, int channel, double sampleRate, int numSamples)
{
    int status;

    uint8_t Config[32];
    uint32_t ConfigSize = sizeof(Config);

    CONFIG[0x0] = 0x00;
    CONFIG[0x1] = 0x00;
    CONFIG[0x2] = 0x00;
    CONFIG[0x3] = 0x00;
    CONFIG[0x4] = 0x00;
    CONFIG[0x5] = 0x00;
    CONFIG[0x6] = 0x00;
    CONFIG[0x7] = 0x00;
    CONFIG[0x8] = 0x00;
    CONFIG[0x9] = 0x00;
    CONFIG[0xa] = 0x00;
    CONFIG[0xb] = 0x00;
    CONFIG[0xc] = 0x00;
    CONFIG[0xd] = 0x00;
    CONFIG[0xe] = 0x00;
    CONFIG[0xf] = 0x00;
    CONFIG[0x10] = 0x00;
    CONFIG[0x11] = 0x01;
    CONFIG[0x12] = (channel & 0xf) << 4 | (channel & 0xf);
    CONFIG[0x13] = 0x00;
    CONFIG[0x14] = (channel & 0xf0) | (channel & 0xf0) >> 4;
    status = AIOUSB::ADC_SetConfig(dev, Config, &ConfigSize);
    err_printf("status(ADC_SetConfig) = %d", status);
    status = AIOUSB::CTR_8254StartOutputFreq(dev, 0, &sampleRate);
    err_printf("status(CTR_8254StartOutputFreq) = %d", status);


#include <stdio.h>
#include "timespec-util.h"

#include "aiousb.h"
#include "AiousbSamples.inc"
#include <string.h>


#define err_printf(fmt, ...) \
        do { printf ("%s:%d:%s(): " fmt "\n", __FILE_NAME__, \
                                __LINE__, __func__, ##__VA_ARGS__); } while (0)



AIOUSB::aiousb_device_handle Device;

#define START_CHANNEL 0
#define END_CHANNEL 7
#define OVERSAMPLE 3
#define FREQUENCY 5000.0
#define LOOP_COUNT 10000

void DumpConfig(AIOUSB::aiousb_device_handle Device)
{
  uint8_t config_buff[32];
  uint32_t config_size = sizeof(config_buff);
  AIOUSB::ADC_GetConfig(Device, config_buff, &config_size);
  printf("Config dump:\n");
  for (int i = 0; i < config_size; i++)
    {
      printf("%02x ", config_buff[i]);
      if ((i + 1) % 16 == 0)
        printf("\n");
    }
  printf("\n");
}

int main (int argc, char **argv)
{
  double voltages[64];
  double frequency = FREQUENCY;
  int status;
  timespec begin, end;
  uint8_t Config[32];
  uint32_t ConfigSize = sizeof(Config);

  status = AIOUSB::AiousbInit();

  err_printf("status(AiousbInit) = %d", status);

  status = SampleGetDeviceHandle(argc, argv, &Device);

  err_printf("status(SampleGetDeviceHandle) = %d", status);

  if (status)
    {
      err_printf("Unable to open device");
    }

  status = AIOUSB::ADC_GetConfig(Device, Config, &ConfigSize);
  err_printf("status(ADC_GetConfig) = %d", status);

  if (status)
    {
      return status;
    }

  Config[0x11] = 0x05; // scans started by counter/timer
  status = AIOUSB::ADC_SetConfig(Device, Config, &ConfigSize);
  err_printf("status(ADC_SetConfig) = %d", status);

  DumpConfig(Device);

  status = AIOUSB::ADC_GetScanV(Device, voltages);

  err_printf("status = %d", status);

  for (int i = 0 ; i <= END_CHANNEL - START_CHANNEL ; i++)
  {
    printf("%d: %f\n", i, voltages[i]);
  }

  clock_gettime(CLOCK_MONOTONIC, &begin);

  // for (int i = 0; i < LOOP_COUNT ; i++)
  // {
  //   status = ADC_GetScanV(Device, voltages);
  // }

  clock_gettime(CLOCK_MONOTONIC, &end);

  printf("ADC_GetScanV Loop: Channels=%d, scans = %d,  time=%ldms\n", END_CHANNEL-START_CHANNEL+1, LOOP_COUNT, timespec_sub_to_msec(&end, &begin));

}
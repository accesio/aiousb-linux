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
  int status;
  timespec begin, end;
  uint8_t Config[32];
  uint32_t ConfigSize = sizeof(Config);

  status = AIOUSB::AiousbInit();

  err_printf("status = %d", status);

  status = SampleGetDeviceHandle(argc, argv, &Device);

  err_printf("status = %d", status);

  if (status)
    {
      err_printf("Unable to open device");
    }

  // DumpConfig(Device);
  
  // status = AIOUSB::ADC_SetCal(Device, ":AUTO:");
  // err_printf("status(ADC_SetCal) = %d", status);
  // DumpConfig(Device);

  // status = AIOUSB::ADC_SetScanLimits(Device, START_CHANNEL, END_CHANNEL);
  // err_printf("status(ADC_SetScanLimits) = %d", status);
  // DumpConfig(Device);

  // status = AIOUSB::ADC_SetOversample(Device, 5);
  // err_printf("status(ADC_SetOversample) = %d", status);
  // DumpConfig(Device);

  // status = AIOUSB::DAC_SetBoardRange(Device, 0); // used to unsleep the ADC reference on USB-AO16-16A and relateds to the adc-getscanv and now it works
  // err_printf("status(DAC_SetBoardRange) = %d", status);
  // DumpConfig(Device);

  // uint8_t gain_codes[16];
  // memset(gain_codes, 0x01, sizeof(gain_codes));
  // status = AIOUSB::ADC_RangeAll(Device, gain_codes, true);
  // err_printf("status(ADC_RangeAll) = %d", status);
  // DumpConfig(Device);

  Config[0x00] = 0x09; // range
  Config[0x01] = 0x09; // range
  Config[0x02] = 0x09; // range
  Config[0x03] = 0x09; // range
  Config[0x04] = 0x09; // range
  Config[0x05] = 0x09; // range
  Config[0x06] = 0x09; // range
  Config[0x07] = 0x09; // range
  Config[0x08] = 0x09; // range
  Config[0x09] = 0x09; // range
  Config[0x0A] = 0x09; // range
  Config[0x0B] = 0x09; // range
  Config[0x0C] = 0x09; // range
  Config[0x0D] = 0x09; // range
  Config[0x0E] = 0x09; // range
  Config[0x0F] = 0x09; // range
  Config[0x10] = 0x00; // calibration code
  Config[0x11] = 0x05; // trigger & counter mode
    // Start and End Channel, low nybbles 0bEEEESSSS  (0xF0 = 0b11110000)
  Config[0x12] = (END_CHANNEL & 0xf) << 4 | (START_CHANNEL & 0xf);

  Config[0x13] = 0x00; // Oversamples

  // Start and End Channel, high nybbles 0bEEEESSSS (0x50 = 0b01010000)
  Config[0x14] = (END_CHANNEL & 0xf0) | (START_CHANNEL & 0xf0) >> 4;


  // status = AIOUSB::ADC_SetCal(Device, ":AUTO:");
  // err_printf("status(ADC_SetCal) = %d", status);
  // DumpConfig(Device);

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
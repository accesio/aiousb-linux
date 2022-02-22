#include <stdio.h>
#include "timespec-util.h"

#include "aiousb.h"
#include "AiousbSamples.inc"


#define err_printf(fmt, ...) \
        do { printf ("%s:%d:%s(): " fmt "\n", __FILE__, \
                                __LINE__, __func__, ##__VA_ARGS__); } while (0)



AIOUSB::aiousb_device_handle Device;

#define START_CHANNEL 0
#define END_CHANNEL 15
#define LOOP_COUNT 10000

int main (int argc, char **argv)
{
  double voltages[64];
  int status;
  timespec begin, end;

  status = AIOUSB::AiousbInit();

  err_printf("status = %d", status);

  status = SampleGetDeviceHandle(argc, argv, &Device);

  err_printf("status = %d", status);

  if (status)
    {
      err_printf("Unable to open device");
    }

  // status = AIOUSB::ADC_SetScanLimits(Device, START_CHANNEL, END_CHANNEL);

  // err_printf("status = %d", status);

  // status = AIOUSB::ADC_SetOversample(Device, 5);

  // err_printf("status = %d", status);

  status = AIOUSB::DAC_SetBoardRange(Device, 0); // used to unsleep the ADC reference on USB-AO16-16A and relateds to the adc-getscanv and now it works
  err_printf("status = %d", status);


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
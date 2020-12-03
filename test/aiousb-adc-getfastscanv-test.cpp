#include <stdio.h>
#include "timespec-util.h"

#include "aiousb.h"

#define err_printf(fmt, ...) \
        do { printf ("%s:%d:%s(): " fmt "\n", __FILE__, \
                                __LINE__, __func__, ##__VA_ARGS__); } while (0)



AIOUSB::aiousb_device_handle device;

#define START_CHANNEL 0
#define END_CHANNEL 15
#define LOOP_COUNT 10000

int main (int arg, char **argv)
{
  double voltages[64];
  int status;
  timespec begin, end;

  status = AIOUSB::AiousbInit();

  err_printf("status = %d", status);

  status = AIOUSB::DeviceHandleByPath(argv[1], &device);

  err_printf("status = %d", status);

  if (status)
    {
      err_printf("Unable to open device");
    }

  status = AIOUSB::ADC_SetScanLimits(device, START_CHANNEL, END_CHANNEL);

  err_printf("status = %d", status);

  status = AIOUSB::ADC_SetOversample(device, 5);

  err_printf("status = %d", status);

  status = AIOUSB::ADC_InitFastScanV(device);

  err_printf("status = %d", status);

  status = AIOUSB::ADC_GetFastScanV(device, voltages);

  err_printf("status = %d", status);

  for (int i = 0 ; i <= END_CHANNEL - START_CHANNEL ; i++)
  {
    printf("%d: %f\n", i, voltages[i]);
  }

  clock_gettime(CLOCK_MONOTONIC, &begin);

  for (int i = 0; i < LOOP_COUNT ; i++)
  {
    status = ADC_GetFastScanV(device, voltages);
  }

  clock_gettime(CLOCK_MONOTONIC, &end);

  printf("aiousb_get_fast_scan_v Loop: Channels=%d, scans = %d,  time=%ldms\n", END_CHANNEL-START_CHANNEL+1, LOOP_COUNT, timespec_sub_to_msec(&end, &begin));

  AIOUSB::ADC_ResetFastScanV(device);

}

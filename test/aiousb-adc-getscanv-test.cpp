#include <stdio.h>
#include "timespec-util.h"

#include "aiousb.h"

#define err_printf(fmt, ...) \
        do { printf ("%s:%d:%s(): " fmt "\n", __FILE__, \
                                __LINE__, __func__, ##__VA_ARGS__); } while (0)



aiousb_device_handle device;

#define START_CHANNEL 0
#define END_CHANNEL 15

int main (int arg, char **argv)
{ 
  double voltages[64];
  int status;

  status = aiousb_init();

  err_printf("status = %d", status);

  status = aiousb_device_handle_by_path(argv[1], &device);

  err_printf("status = %d", status);

  if (status)
    {
      err_printf("Unable to open device");
    }

  status = aiousb_set_scan_limits(device, START_CHANNEL, END_CHANNEL);

  err_printf("status = %d", status);


  status = aiousb_get_scan_v(device, voltages);

  err_printf("status = %d", status);

  for (int i = 0 ; i <= END_CHANNEL - START_CHANNEL ; i++)
  {
    printf("%d: %f\n", i, voltages[i]);
  }



}
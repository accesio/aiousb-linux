#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include "aiousb.h"

#define err_printf(fmt, ...) \
        do { printf ("%s:%d:%s(): \n" fmt, __FILE__, \
                                __LINE__, __func__, ##__VA_ARGS__); } while (0)



int main (int arg, char **argv)
{
  int status;
  AIOUSB::aiousb_device_handle device;
  uint8_t *config_buff = NULL;;
  uint32_t config_size = 0;

  AIOUSB::AiousbInit();
  status = AIOUSB::DeviceHandleByPath("/dev/accesio/usb_ai16_64ma_3", &device);

  if (status)
  {
      err_printf("Unable to open device\n");
      exit(1);
  }

  printf("device = %p\n", device);

  status = AIOUSB::ADC_GetConfig(device, NULL, &config_size);

  config_buff = (uint8_t *) malloc(config_size);

  status = AIOUSB::ADC_GetConfig(device, config_buff, &config_size);


  AIOUSB::ADC_Range1(device, 2, 5, 0);

  status = AIOUSB::ADC_GetConfig(device, config_buff, &config_size);


  free(config_buff);
}


#include <stdio.h>
#include "aiousb.h"
#include "AiousbSamples.inc"

#define err_printf(fmt, ...) \
        do { printf ("%s:%d:%s(): " fmt "\n", __FILE__, \
                                __LINE__, __func__, ##__VA_ARGS__); } while (0)

int main (int argc, char **argv)
{
  int status;
  uint32_t device_mask;
  uint8_t read_val;
  uint32_t device;


  status = AIOUSB::AiousbInit();

  err_printf("status = %d", status);

  if (status)
  {
    err_printf("Unable to open device");
  }

  device_mask = AIOUSB::GetDevices();

  if (!device_mask)
  {
    err_printf("No devices detected");
    exit(-1);
  }

  for (int i = 0 ; i < 32 ; i++)
  {
    if (!(device_mask & 1 << i)) continue;

    status = AIOUSB::CustomEEPROMRead(i, 0, 1, &read_val);

    err_printf("Device Index: %x, read_val: 0x%x", i, read_val);

    device = AIOUSB::DeviceIndexByEEPROMByte(read_val);

    err_printf("Device index: %x, errno: %d", device, errno);

    device = AIOUSB::DeviceIndexByEEPROMByte(~read_val);

    err_printf("Device index: %x, errno: %d", device, errno);
  }
}

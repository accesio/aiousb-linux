#include <stdio.h>
#include <unistd.h>

#include "aiousb.h"

#define START_CHANNEL 0
#define END_CHANNEL 15
#define OVERSAMPLE 0
#define FREQUENCY 5000

#define err_printf(fmt, ...) \
        do { printf ("%s:%d:%s(): \n" fmt, __FILE__, \
                                __LINE__, __func__, ##__VA_ARGS__); } while (0)


void callback (uint16_t *buff, uint32_t buff_size,
              uint32_t flags, void *context)
{
  err_printf("buff = %p, buff_size = %u, flags = 0x%x, context = %p\n",
                buff, buff_size, flags, context);
  double volts;

  //Just print the first sample per channel in the callback for testing.
  for (int sample = 0; sample <(END_CHANNEL - START_CHANNEL + 1); sample++)
  {
    //this volts assumes a range of 0 - 10 v and no post_scale.
    //see ADC_GetScanV for a more complete implementation.
    volts = buff[sample] * 1.0/65536.0;
    volts = volts * 10;
    //printf("channel %d: %f\n", sample + 1, volts);
  }
}



int main (int argc, char **argv)
{
  int status;
  AIOUSB::aiousb_device_handle device = NULL;
  double frequency = FREQUENCY;
  uint8_t config_buff[21];
  uint32_t config_size = sizeof(config_buff);

  status = AIOUSB::AiousbInit();

  if (status != 0)
    {
      err_printf("aiousb_init() faild: %d", status);
      return -1;
    }

	if (argc != 2)
	{
		printf("usage: aiousb-continuous-acquisition <filename>\n");
		return -1;
	}

	status = AIOUSB::DeviceHandleByPath(argv[1], &device);

	if (status)
	{
		err_printf("Error opening device: %d\n", status);
		return -1;
	}

  status = AIOUSB::ADC_GetConfig(device, config_buff, &config_size);

if (status)
{
  err_printf("Error getting config");
}

config_buff[0x11] = 0x5; //set scans to be started by counter

status = AIOUSB::ADC_SetConfig(device, config_buff, &config_size);

if (status)
{
  err_printf("Error setting config");
}

status = AIOUSB::ADC_SetScanLimits(device, START_CHANNEL, END_CHANNEL);

if (status)
{
  err_printf("Error setting scan limits");
}

status = AIOUSB::ADC_SetOversample(device, OVERSAMPLE);


  AIOUSB::CTR_8254StartOutputFreq(device, 0, &frequency);

  status = AIOUSB::ADC_BulkContinuousStart(device,
                                512 * (END_CHANNEL - START_CHANNEL + 1),
                                1,
                                0,
                                callback);

  if (status)
  {
    err_printf("status = %d", status);
  }
  else
  {
    sleep(600);
  }

  AIOUSB::ADC_BulkContinuousEnd(device);

  sleep(5);



}
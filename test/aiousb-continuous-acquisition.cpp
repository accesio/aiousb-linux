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
              uint32_t flags, uint32_t context)
{
  err_printf("buff = %p, buff_size = %u, flags = 0x%x, context = %u\n",
                buff, buff_size, flags, context);
  double volts;

  //Just print the first sample per channel in the callback for testing.
  for (int sample = 0; sample <(END_CHANNEL - START_CHANNEL + 1); sample++)
  {
    //this volts assumes a range of 0 - 10 v and no post_scale.
    //see aiousb_get_scan_v for a more complete implementation.
    volts = buff[sample] * 1.0/65536.0;
    volts = volts * 10;
    printf("channel %d: %f\n", sample + 1, volts);
  }
}



int main (int argc, char **argv)
{
  int status;
  aiousb_device_handle device = NULL;
  double frequency = FREQUENCY;
  uint8_t config_buff[21];
  uint32_t config_size = sizeof(config_buff);

  status = aiousb_init();

  if (status != 0)
    {
      err_printf("aiousb_init() faild: %d", status);
      return -1;
    }

	if (argc != 2)
	{
		printf("usage: aiousb-test <filename>\n");
		return -1;
	}

	status = aiousb_device_handle_by_path(argv[1], &device);

	if (status)
	{
		err_printf("Error opening device: %d\n", status);
		return -1;
	}

  status = aiousb_adc_get_config(device, config_buff, &config_size);

if (status)
{
  err_printf("Error getting config");
}

config_buff[0x11] = 0x5; //set scans to be started by counter

status = aiousb_adc_set_config(device, config_buff, &config_size);

if (status)
{
  err_printf("Error setting config");
}

status = aiousb_set_scan_limits(device, START_CHANNEL, END_CHANNEL);

if (status)
{
  err_printf("Error setting scan limits");
}

status = aiousb_adc_set_oversample(device, OVERSAMPLE);


  aiousb_ctr_8254_start_output_frequency(device, 0, &frequency);

  status = aiousb_adc_bulk_continuous_start(device,
                                512 * (END_CHANNEL - START_CHANNEL + 1),
                                3,
                                0,
                                callback);

  if (status)
  {
    err_printf("status = %d", status);
  }
  else
  {
    sleep(6);
  }

  aiousb_adc_bulk_continuous_end(device);

  sleep(5);



}
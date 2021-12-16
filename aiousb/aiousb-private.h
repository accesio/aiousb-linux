#ifndef AIOUSB_PRIVATE_H
#define AIOUSB_PRIVATE_H
#include <stdio.h>
#include "accesio_usb_ids.h"
#include "adc-threads.h"

#define aiousb_library_err_print(fmt, ...) \
  do { printf( "%s:%d:%s(): " fmt "\n", __FILE__, __LINE__, __FUNCTION__, \
      ##__VA_ARGS__); } while (0);

#define aiousb_debug_print(fmt, ...) \
    do { if (AIOUSB_DEBUG) printf ("%s:%d:%s(): " fmt "\n", __FILE__, __LINE__, __func__, \
      ##__VA_ARGS__); } while (0);

#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))

#ifndef NO_HOTPLUG
#define NO_HOTPLUG 0
#endif

namespace AIOUSB {

struct pnp_data
{
    uint8_t pnp_size;
    uint16_t rev;
    uint8_t adc_config_bytes;
    uint8_t dio_control_bytes;
    uint8_t dio_bytes;
    uint8_t counter_blocks;
    uint8_t dac_words;
    uint8_t b_has_deb;
    uint8_t b_has_latch;
    uint8_t pwmt_table_max_len;
    uint8_t map_table_max_len;
    uint8_t map_high_src_bit;
    uint8_t map_high_dst_bit;
    uint8_t b_has_dio_write1;
};
struct aiousb_device
{
  int fd;
  struct acces_usb_device_descriptor descriptor;
  uint8_t *last_dio_data;
  int b_firmware_20;
  struct pnp_data pnp_data;
  //dio streaming stuff
  int b_dio_open, b_dio_read;
  size_t streaming_block_size;
  //thread for getscan functions
  pthread_t adc_worker;
  struct adc_worker_context adc_worker_context;
  //thread for continuous acquisition
  pthread_t adc_cont_buff_worker;
  struct adc_cont_buff_worker_context adc_cont_buff_worker_context;
  //thread for continuous acquisition
  pthread_t adc_cont_acq_worker;
  struct adc_cont_acq_worker_context adc_cont_acq_worker_context;
  char *dev_path;
  ContinuousAdcWorker *ContAdc;
  //for fast scan
  uint8_t *config_buff_bak;
  uint8_t *config_fast;
  unsigned int config_size;
};

} //namespace AIOUSB

#endif

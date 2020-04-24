#ifndef AIOUSB_H
#define AIOUSB_H

#include <stdint.h>
#include "accesio_usb_ioctl.h"

typedef struct aiousb_device* aiousb_device_handle;

#define ADC_CONT_CALLBACK_FLAG_BEGIN_BC 0x1
#define ADC_CONT_CALLBACK_FLAG_END_STREAM 0x2
#define ADC_CONT_CALLBACK_FLAG_INSERTED 0x4

typedef void (*adc_cont_callback) (uint16_t *buff, uint32_t buff_size,
              uint32_t flags, uint32_t context);

int aiousb_device_open (const char *fname, aiousb_device_handle *device);
void aiousb_device_close (aiousb_device_handle device);

int aiousb_generic_vendor_read(aiousb_device_handle device, 
            uint8_t request, uint16_t value, uint16_t index, 
            uint16_t size, void *data);

int aiousb_generic_vendor_write(aiousb_device_handle device,
            uint8_t request, uint16_t value, uint16_t index,
            uint16_t size, void *data);

int aiousb_generic_bulk_in (aiousb_device_handle device, 
          unsigned int pipe_index, void *data, int size,
          int *transferred);

int aiousb_generic_bulk_out (aiousb_device_handle device, unsigned int pipe_index,
           void *data, int size,	int *transferred);

int aiousb_dio_configure (aiousb_device_handle device, uint8_t b_tristate,
          void *out_mask, void *data);

int aiousb_dio_configure_ex (aiousb_device_handle device, void * out_mask,
                void *data, void *tristate_mask);

int aiousb_dio_configure_masked(aiousb_device_handle device, void *outs,
                void *outs_mask, void *data, void *data_mask, void *tristates,
                void *tristates_mask);

int aiousb_dio_write_all(aiousb_device_handle device, void *data);

int aiousb_dio_write_8(aiousb_device_handle device, uint32_t byte_index,
                uint8_t data);

int aiousb_dio_write1(aiousb_device_handle device, uint32_t bit_index,
                uint8_t data);

int aiousb_dio_read_all(aiousb_device_handle device, void *data);

int aiousb_dio_read_8(aiousb_device_handle device, uint32_t byte_index,
                uint8_t *data);

int aiousb_dio_read_1(aiousb_device_handle device, uint32_t bit_index,
                uint8_t *data);

int aiousb_dio_configuration_query(aiousb_device_handle device, void *out_mask,
                void *tristate_mask);

int aiousb_dio_stream_open(aiousb_device_handle device, uint32_t is_read);

int aiousb_dio_stream_close(aiousb_device_handle device);

int aiousb_dio_stream_set_clocks(aiousb_device_handle device, double *read_hz,
                double *write_hz);

int aiousb_dio_stream_frame (aiousb_device_handle device, unsigned long frame_points,
                unsigned short *frame_data, size_t *bytes_transferred);

int aiousb_ctr_8254_mode(aiousb_device_handle device, uint32_t block_index, 
                uint32_t counter_index, uint32_t mode);

int aiousb_ctr_8254_load(aiousb_device_handle device, uint32_t block_index,
                uint32_t counter_index, uint16_t load_value);

int aiousb_ctr_8254_mode_load(aiousb_device_handle device, uint32_t block_index,
                uint32_t counter_index, uint32_t mode, uint16_t load_value);

int aiousb_ctr_8254_start_output_frequency(aiousb_device_handle device, 
                uint32_t block_index, double *frequency);

int aiousb_ctr_8254_read(aiousb_device_handle device, uint32_t block_index,
                uint32_t counter_index, uint16_t *read_value);

int aiousb_ctr_8254_read_all(aiousb_device_handle device, uint16_t *data);

int aiousb_ctr_8254_read_status(aiousb_device_handle device, uint32_t block_index,
                uint32_t counter_index, uint8_t *read_value, uint8_t *status);

int aiousb_ctr_8254_read_mode_load(aiousb_device_handle device, uint32_t block_index,
                uint32_t counter_index, uint32_t mode, uint16_t load_value,
                uint16_t *read_value);

int aiousb_get_scan_v(aiousb_device_handle device, double *data);

int aiousb_get_channel_v (aiousb_device_handle device, uint32_t channel_index,
                double *volts);

int aiousb_get_trig_scan_v (aiousb_device_handle device, double *data,
                uint32_t timeout_ms);

int aiousb_set_scan_limits (aiousb_device_handle device, uint32_t start_channel,
                uint32_t end_channel);

int aiousb_adc_bulk_continuous_start (aiousb_device_handle device,
                uint32_t buff_size, uint32_t base_buff_count, uint32_t context,
                adc_cont_callback callback);

int aiousb_adc_get_config(aiousb_device_handle device, uint8_t *config_buff, 
                uint32_t *config_size);

int aiousb_adc_range_all(aiousb_device_handle device, uint8_t *gain_codes, 
                uint32_t *b_differential);

int aiousb_adc_range1(aiousb_device_handle device, uint32_t adc_channel,
                uint8_t gain_code, uint32_t b_differential);

int aiousb_adc_set_oversample(aiousb_device_handle device, uint8_t oversample);

int aiousb_adc_set_config(aiousb_device_handle device, uint8_t *config_buff,
              uint32_t *config_size);

#endif
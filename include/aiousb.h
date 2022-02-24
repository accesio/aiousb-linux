#ifndef AIOUSB_H
#define AIOUSB_H

#include <stdint.h>
#include <stddef.h>
#include "accesio_usb_ioctl.h"

#ifdef __cplusplus
namespace AIOUSB {
#endif

typedef struct aiousb_device* aiousb_device_handle;

#define ADC_CONT_CALLBACK_FLAG_BEGIN_BC 0x1
#define ADC_CONT_CALLBACK_FLAG_END_STREAM 0x2
#define ADC_CONT_CALLBACK_FLAG_INSERTED 0x4

typedef void (*adc_cont_callback) (uint16_t *buff, uint32_t buff_size,
              uint32_t flags, void *context);

//init type stuff
#ifdef __cplusplus
extern "C"
{
#endif
int AiousbInit();
int DeviceHandleByPath (const char *fname, aiousb_device_handle *device);
int DeviceHandleByIndex(unsigned long device_index, aiousb_device_handle *device);
int DeviceIndexByPath (const char *fname, unsigned long *device_index);
uint32_t GetDevices();
#ifdef __cplusplus
}
#endif

//Special device indexes
#define diNone  0xffffffff
#define diFirst 0xfffffffe
#define diOnly  0xfffffffd


//device handle based functions
#ifdef __cplusplus

int QueryDeviceInfo(aiousb_device_handle device,
                              uint32_t *pid, uint32_t *name_size, char *name,
                              uint32_t *dio_bytes, uint32_t *counters);

int GetDeviceSerialNumber(aiousb_device_handle device, uint64_t *serial_number);

int GenericVendorRead(aiousb_device_handle device,
            uint8_t request, uint16_t value, uint16_t index,
            uint16_t size, void *data);

int GenericVendorWrite(aiousb_device_handle device,
            uint8_t request, uint16_t value, uint16_t index,
            uint16_t size, void *data);

int AWU_GenericBulkIn (aiousb_device_handle device,
          unsigned int pipe_index, void *data, int size,
          int *transferred);

int AWU_GenericBulkOut (aiousb_device_handle device, unsigned int pipe_index,
           void *data, int size,	int *transferred);

int CustomEEPROMWrite(aiousb_device_handle device,
            uint32_t start_address, uint32_t data_size, void *data);

int CustomEEPROMRead(aiousb_device_handle device,
            uint32_t start_address, uint32_t data_size, void *data);

int DIO_Configure (aiousb_device_handle device, uint8_t b_tristate,
          void *out_mask, void *data);

int DIO_ConfigureEx (aiousb_device_handle device, void * out_mask,
                void *data, void *tristate_mask);

int DIO_ConfigureMasked(aiousb_device_handle device, void *outs,
                void *outs_mask, void *data, void *data_mask, void *tristates,
                void *tristates_mask);

int DIO_WriteAll(aiousb_device_handle device, void *data);

int DIO_Write8(aiousb_device_handle device, uint32_t byte_index,
                uint8_t data);

int DIO_Write1(aiousb_device_handle device, uint32_t bit_index,
                uint8_t data);

int DIO_ReadAll(aiousb_device_handle device, void *data);

int DIO_Read8(aiousb_device_handle device, uint32_t byte_index,
                uint8_t *data);

int DIO_Read1(aiousb_device_handle device, uint32_t bit_index,
                uint8_t *data);

int DIO_ConfigurationQuery(aiousb_device_handle device, void *out_mask,
                void *tristate_mask);

int DIO_StreamOpen(aiousb_device_handle device, uint32_t is_read);

int DIO_StreamClose(aiousb_device_handle device);

int DIO_StreamSetClocks(aiousb_device_handle device, double *read_hz,
                double *write_hz);

int DIO_StreamFrame (aiousb_device_handle device, unsigned long frame_points,
                unsigned short *frame_data, size_t *bytes_transferred);

int CTR_8254Mode(aiousb_device_handle device, uint32_t block_index,
                uint32_t counter_index, uint32_t mode);

int CTR_8254Load(aiousb_device_handle device, uint32_t block_index,
                uint32_t counter_index, uint16_t load_value);

int CTR_8254ModeLoad(aiousb_device_handle device, uint32_t block_index,
                uint32_t counter_index, uint32_t mode, uint16_t load_value);

int CTR_8254StartOutputFreq(aiousb_device_handle device,
                uint32_t block_index, double *frequency);

int CTR_8254Read(aiousb_device_handle device, uint32_t block_index,
                uint32_t counter_index, uint16_t *read_value);

int CTR_8254ReadAll(aiousb_device_handle device, uint16_t *data);

int CTR_8254ReadStatus(aiousb_device_handle device, uint32_t block_index,
                uint32_t counter_index, uint8_t *read_value, uint8_t *status);

int CTR_8254ReadModeLoad(aiousb_device_handle device, uint32_t block_index,
                uint32_t counter_index, uint32_t mode, uint16_t load_value,
                uint16_t *read_value);

int ADC_GetScanV(aiousb_device_handle device, double *data);

int ADC_GetChannelV (aiousb_device_handle device, uint32_t channel_index,
                double *volts);

int ADC_GetTrigScanV (aiousb_device_handle device, double *data,
                uint32_t timeout_ms);

int ADC_SetScanLimits (aiousb_device_handle device, uint32_t start_channel,
                uint32_t end_channel);

int ADC_BulkContinuousStart (aiousb_device_handle device,
                uint32_t buff_size, uint32_t base_buff_count, void *context,
                adc_cont_callback callback);

int ADC_BulkContinuousEnd (aiousb_device_handle device);

int ADC_GetConfig(aiousb_device_handle device, uint8_t *config_buff,
                uint32_t *config_size);

int ADC_RangeAll(aiousb_device_handle device, uint8_t *gain_codes,
                uint32_t *b_differential);

int ADC_Range1(aiousb_device_handle device, uint32_t adc_channel,
                uint8_t gain_code, uint32_t b_differential);

int ADC_SetCal(aiousb_device_handle device, const char *CalFileName);

int ADC_SetCalAndSave(aiousb_device_handle device, const char *CalFileName,
                  const char *OutFileName);

int ADC_SetOversample(aiousb_device_handle device, uint8_t oversample);

int ADC_SetConfig(aiousb_device_handle device, uint8_t *config_buff,
              uint32_t *config_size);

int ADC_InitFastScanV(aiousb_device_handle device);

int ADC_GetFastScanV(aiousb_device_handle device, double *data);

int ADC_ResetFastScanV(aiousb_device_handle device);


int DAC_SetBoardRange (aiousb_device_handle device, uint32_t range_code);

int DAC_Direct(aiousb_device_handle device, uint32_t channel, uint16_t counts);

int DAC_MultiDirect(aiousb_device_handle device, void * dac_data, uint32_t data_count);

int AbortPipe(aiousb_device_handle device);

int ResetChip(aiousb_device_handle device);

#endif


///Device index based functions
#ifdef __cplusplus
extern "C"
{
#endif

int QueryDeviceInfo(unsigned long device_index,
                            uint32_t *pid, uint32_t *name_size, char *name,
                            uint32_t *dio_bytes, uint32_t *counters);

int GetDeviceSerialNumber(unsigned long device_index, uint64_t *serial_number);

int GenericVendorRead(unsigned long device_index,
            uint8_t request, uint16_t value, uint16_t index,
            uint16_t size, void *data);

int GenericVendorWrite(unsigned long device_index,
            uint8_t request, uint16_t value, uint16_t index,
            uint16_t size, void *data);

int AWU_GenericBulkIn (unsigned long device_index,
          unsigned int pipe_index, void *data, int size,
          int *transferred);

int AWU_GenericBulkOut (unsigned long device_index, unsigned int pipe_index,
           void *data, int size,	int *transferred);

int CustomEEPROMWrite(unsigned long device_index,
            uint32_t start_address, uint32_t data_size, void *data);

int CustomEEPROMRead(unsigned long device_index,
            uint32_t start_address, uint32_t data_size, void *data);

int DIO_Configure (unsigned long device_index, uint8_t b_tristate,
          void *out_mask, void *data);

int DIO_ConfigureEx (unsigned long device_index, void * out_mask,
                void *data, void *tristate_mask);

int DIO_ConfigureMasked(unsigned long device_index, void *outs,
                void *outs_mask, void *data, void *data_mask, void *tristates,
                void *tristates_mask);

int DIO_WriteAll(unsigned long device_index, void *data);

int DIO_Write8(unsigned long device_index, uint32_t byte_index,
                uint8_t data);

int DIO_Write1(unsigned long device_index, uint32_t bit_index,
                uint8_t data);

int DIO_ReadAll(unsigned long device_index, void *data);

int DIO_Read8(unsigned long device_index, uint32_t byte_index,
                uint8_t *data);

int DIO_Read1(unsigned long device_index, uint32_t bit_index,
                uint8_t *data);

int DIO_ConfigurationQuery(unsigned long device_index, void *out_mask,
                void *tristate_mask);

int DIO_StreamOpen(unsigned long device_index, uint32_t is_read);

int DIO_StreamClose(unsigned long device_index);

int DIO_StreamSetClocks(unsigned long device_index, double *read_hz,
                double *write_hz);

int DIO_StreamFrame (unsigned long device_index, unsigned long frame_points,
                unsigned short *frame_data, size_t *bytes_transferred);

int CTR_8254Mode(unsigned long device_index, uint32_t block_index,
                uint32_t counter_index, uint32_t mode);

int CTR_8254Load(unsigned long device_index, uint32_t block_index,
                uint32_t counter_index, uint16_t load_value);

int CTR_8254ModeLoad(unsigned long device_index, uint32_t block_index,
                uint32_t counter_index, uint32_t mode, uint16_t load_value);

int CTR_8254StartOutputFreq(unsigned long device_index,
                uint32_t block_index, double *frequency);

int CTR_8254Read(unsigned long device_index, uint32_t block_index,
                uint32_t counter_index, uint16_t *read_value);

int CTR_8254ReadAll(unsigned long device_index, uint16_t *data);

int CTR_8254ReadStatus(unsigned long device_index, uint32_t block_index,
                uint32_t counter_index, uint8_t *read_value, uint8_t *status);

int CTR_8254ReadModeLoad(unsigned long device_index, uint32_t block_index,
                uint32_t counter_index, uint32_t mode, uint16_t load_value,
                uint16_t *read_value);

int ADC_GetScanV(unsigned long device_index, double *data);

int ADC_GetChannelV (unsigned long device_index, uint32_t channel_index,
                double *volts);

int ADC_GetTrigScanV (unsigned long device_index, double *data,
                uint32_t timeout_ms);

int ADC_SetScanLimits (unsigned long device_index, uint32_t start_channel,
                uint32_t end_channel);

int ADC_BulkContinuousStart (unsigned long device_index,
                uint32_t buff_size, uint32_t base_buff_count, void *context,
                adc_cont_callback callback);

int ADC_BulkContinuousEnd (unsigned long device_index);

int ADC_GetConfig(unsigned long device_index, uint8_t *config_buff,
                uint32_t *config_size);

int ADC_RangeAll(unsigned long device_index, uint8_t *gain_codes,
                uint32_t *b_differential);

int ADC_Range1(unsigned long device_index, uint32_t adc_channel,
                uint8_t gain_code, uint32_t b_differential);

int ADC_SetCal(unsigned int device_index, const char *CalFileName);

int ADC_SetCalAndSave(unsigned int device_index, const char *CalFileName,
                  const char *OutFileName);

int ADC_SetOversample(unsigned long device_index, uint8_t oversample);

int ADC_SetConfig(unsigned long device_index, uint8_t *config_buff,
              uint32_t *config_size);

int ADC_InitFastScanV(unsigned long device_index);

int ADC_GetFastScanV(unsigned long device_index, double *data);

int ADC_ResetFastScanV(unsigned long device_index);


int DAC_SetBoardRange (unsigned long device_index, uint32_t range_code);

int DAC_Direct(unsigned long device_index, uint32_t channel, uint16_t counts);

int DAC_MultiDirect(unsigned long device_index, void * dac_data, uint32_t data_count);

int AbortPipe(unsigned long device_index);

int ResetChip(unsigned long device_index);

#ifdef __cplusplus
} //extern
} //namespace AIOUSB
#endif

#endif


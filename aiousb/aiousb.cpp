#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <stddef.h>
#include <math.h>
#include <pthread.h>
#include <time.h>


#include "aiousb.h"
#include "accesio_usb_ioctl.h"
#include "aiousb-private.h"

#include "timespec-util.h"



#define MAX_CONFIG_SIZE 21
#define MAX_CHANNELS_FOR_SCAN 128

#pragma pack(push,1)


struct dio_clock_data
{
  uint8_t disables;
  uint16_t write_oct_dac;
  uint16_t read_oct_dac;
};

struct adc_intermmediate_buff
{
  uint16_t ad0;
  uint16_t ad1;
  uint32_t dits2301;
};

#pragma pack(pop)


struct time_out_context
{
  aiousb_device_handle device;
  uint32_t time_out_ms;
  int b_timeout;
  int b_terminate;
  pthread_mutex_t mutex;
  pthread_cond_t cond;
};



static const uint8_t AUR_DIO_WRITE           = 0x10;
static const uint8_t AUR_DIO_READ            = 0x11;
static const uint8_t AUR_DIO_CONFIG          = 0x12;
static const uint8_t AUR_DIO_CONFIG_QUERY    = 0x13;
static const uint8_t AUR_DIO_LATCH_READ      = 0x14;
static const uint8_t AUR_DIO_DEB_READ        = 0x15;
static const uint8_t AUR_DIO_ADVANCED        = 0x18;
static const uint8_t AUR_DIO_CONFIG_MASKED   = 0x1F;
static const uint8_t AUR_CTR_READ            = 0x20;
static const uint8_t AUR_CTR_MODE            = 0x21;
static const uint8_t AUR_CTR_LOAD            = 0x22;
static const uint8_t AUR_CTR_MODELOAD        = 0x23;
static const uint8_t AUR_CTR_SELGATE         = 0x24;
static const uint8_t AUR_CTR_READALL         = 0x25;
static const uint8_t AUR_CTR_READLATCHED     = 0x26;
static const uint8_t AUR_CTR_COS_BULK_GATE2  = 0x27;
static const uint8_t AUR_CTR_COS_BULK_ABORT  = 0x29;
//TODO: Figure out if I need these or get rid of them
// {
// AUR_CTR_PUR_FIRST       = $28; //Not used with device, for index offsetting
// AUR_CTR_PUR_OFRQ        = $28; //Set up to output frequency
// AUR_CTR_PUR_MFRQ        = $2C; //Set up to measure frequency
// AUR_CTR_PUR_EVCT        = $2D; //Set up to count events
// AUR_CTR_PUR_MPUL        = $2E; //Set up to measure pulse width
// }
static const uint8_t AUR_WDG_STATUS          = 0x2E;
static const uint8_t AUR_DIO_WDG16_DEPREC    = 0x2F;
static const uint8_t AUR_GEN_CLEAR_FIFO_NEXT = 0x34;
static const uint8_t AUR_GEN_CLEAR_FIFO      = 0x35;
static const uint8_t AUR_GEN_CLEAR_FIFO_WAIT = 0x36;
static const uint8_t AUR_GEN_ABORT_AND_CLEAR = 0x38;
static const uint8_t AUR_SET_GPIF_MODE       = 0x39;
static const uint8_t AUR_GEN_REGISTER        = 0x3C;
static const uint8_t AUR_PCIE_WDG            = 0x40;
static const uint8_t AUR_WDG                 = 0x44;
static const uint8_t AUR_OFFLINE_READWRITE   = 0x50;
static const uint8_t AUR_DACDIO_WRITEALL     = 0x8C;
static const uint8_t AUR_SELF_TEST_1         = 0x91;
static const uint8_t AUR_DAC_CONTROL         = 0xB0;
static const uint8_t AUR_DAC_DATAPTR         = 0xB1;
static const uint8_t AUR_DAC_DIVISOR         = 0xB2;
static const uint8_t AUR_DAC_IMMEDIATE       = 0xB3;
static const uint8_t AUR_GEN_STREAM_STATUS   = 0xB4;
static const uint8_t AUR_FLASH_READWRITE     = 0xB5;
static const uint8_t AUR_DAC_RANGE           = 0xB7;
static const uint8_t AUR_PROBE_CALFEATURE    = 0xBA;
static const uint8_t AUR_DIO_SETCLOCKS       = 0xBD;
static const uint8_t AUR_ADC_SET_CONFIG      = 0xBE;
static const uint8_t AUR_ADC_IMMEDIATE       = 0xBF;
static const uint8_t AUR_DIO_SPI_WRITE       = 0xC0;
static const uint8_t AUR_DIO_SPI_READ        = 0xC1;
static const uint8_t AUR_SET_CUSTOM_CLOCKS   = 0xC4;
static const uint8_t AUR_ADC_GET_CONFIG      = 0xD2;
static const uint8_t AUR_DEBUG_DEBUG         = 0xD4;
static const uint8_t AUR_DEBUG_FLASH_1TO1    = 0xF0;
static const uint8_t AUR_DEBUG_FLASH_ERASE   = 0xFC;

//Cypress FX2 vendor requests.
static const uint8_t CUR_RAM_READ            = 0xA3;


int aiousb_device_open (const char *fname, aiousb_device_handle *device)
{
  int i;
  int status;
  struct accesio_usb_aiousb_info accesio_usb_aiousb_info;
  struct aiousb_device *ptr;
  ptr = (struct aiousb_device *)malloc(sizeof(struct aiousb_device));
  memset(ptr, 0, sizeof(struct aiousb_device));
  uint8_t mem_flags[3];

  ptr->fd = open (fname , O_RDWR );
  if (ptr->fd < 0)
    {
      aiousb_library_err_print("Error opening device file");
      return -1;
    }

  status = ioctl(ptr->fd, ACCESIO_USB_AIOUSB_INFO, &accesio_usb_aiousb_info);

  if (status)
    {
      aiousb_library_err_print("Error getting device info");
      return -1;
    }

  for ( i = 0 ; i < NUM_ACCES_USB_DEVICES ; i++)
    {
      if (accesio_usb_aiousb_info.pid == acces_usb_device_table[i].pid_loaded)
        {
          ptr->descriptor = acces_usb_device_table[i];
          if (ptr->descriptor.dio_config_bits == 0)
            {
              ptr->descriptor.dio_config_bits = ptr->descriptor.dio_bytes;
            }
          if (ptr->descriptor.dio_bytes > 0)
            {
              ptr->last_dio_data = (uint8_t *) malloc(ptr->descriptor.dio_bytes);
            }
          break;
        }
    }

  status = aiousb_generic_vendor_read(ptr,
                                CUR_RAM_READ,
                                0x8000,
                                1,
                                sizeof(mem_flags),
                                mem_flags);

  if (status != sizeof(mem_flags))
    {
      aiousb_library_err_print("Attempt to check Firmware rev failed");
      mem_flags[0] = 0xff;
    }

  if (mem_flags[0] != 0xff)
    {
      if (mem_flags[0] >= 3)
        {
          if (!(mem_flags[2] &2))
            {
              ptr->b_firmware_20 = 1;
            }
        }
    }

  status = aiousb_generic_vendor_read(ptr,
                                0x3f,
                                0,
                                0,
                                sizeof(struct pnp_data),
                                &ptr->pnp_data);

  if (status != sizeof(struct  pnp_data))
    {
      aiousb_library_err_print("Attempt to read pnp data failed");
      ptr->pnp_data.pnp_size = 0;
    };

  if (ptr->pnp_data.pnp_size == 0xff)
    {
      ptr->pnp_data.pnp_size = 0;
    }

  ptr->streaming_block_size = 31 * 1024;

  

  aiousb_debug_print("opened %s", fname);
  *device = ptr;

  return 0;
}

void aiousb_device_close(aiousb_device_handle device)
{
  if (device->last_dio_data != NULL)
    {
      free(device->last_dio_data);
      device->last_dio_data = NULL;
    }
  close(device->fd);
  free (device);
}

int aiousb_generic_vendor_read(aiousb_device_handle device, 
            uint8_t request, uint16_t value, uint16_t index, 
            uint16_t size, void *data)
{
  struct  accesio_usb_control_transfer context = {0};
  int status;

  context.request = request;
  context.value = value;
  context.index = index;
  context.size = size;
  context.data = data;
  context.read = 1;

  status = ioctl (device->fd, ACCESIO_USB_CONTROL_XFER, &context);

  if (status)
    {
      printf("ioctl returned %d\n", status);
    }
  return status;
}


int aiousb_generic_vendor_write(aiousb_device_handle device,
            uint8_t request, uint16_t value, uint16_t index,
            uint16_t size, void *data)
{
  struct accesio_usb_control_transfer context = {0};
  int status;

  context.request = request;
  context.value = value;
  context.index = index;
  context.size = size;
  context.data = data;
  context.read = 0;

  status = ioctl (device->fd, ACCESIO_USB_CONTROL_XFER, &context);

  if (status)
    {
      printf("ioctl returned %d\n", status);
    }

  return status;
}

int aiousb_generic_bulk_in (aiousb_device_handle device, 
          unsigned int pipe_index, void *data, int size,
          int *transferred)
{
  struct accesio_usb_bulk_transfer context = {0};
  int status;

  context.pipe_index = pipe_index;
  context.data = data;
  context.size = size;
  context.transferred = transferred;
  context.read = 1;

  status = ioctl(device->fd, ACCESIO_USB_BULK_XFER, &context);

  printf("%s: status = %d\n", __FUNCTION__, status);

  return status;
}

int aiousb_generic_bulk_out (aiousb_device_handle device, unsigned int pipe_index,
           void *data, int size,	int *transferred)
{
  struct accesio_usb_bulk_transfer context = {0};
  int status;

  printf("WARNING: bulk_out hasn't been proven to work yet\n");

  context.pipe_index = pipe_index;
  context.data = data;
  context.size = size;
  context.transferred = transferred;
  context.read = 0;

  status = ioctl(device->fd, ACCESIO_USB_BULK_XFER, &context);

  printf("%s: status = %d\n", __FUNCTION__, status);

  return status;
}

int aiousb_dio_configure (aiousb_device_handle device, uint8_t b_tristate,
          void *out_mask, void *data)
{
  uint8_t *payload;
  uint16_t out_mask_size;
  uint16_t payload_size;
  int status;

  if (device->descriptor.dio_bytes == 0)
    {
      return -EBADRQC;
    }

  if ((out_mask == NULL) || ( data == NULL))
    {
      return -EINVAL;
    }

  out_mask_size = (device->descriptor.dio_config_bits + 7) / 8;
  payload_size = device->descriptor.dio_bytes + 2 * out_mask_size;

  payload = (uint8_t *) malloc(payload_size);

  memcpy(device->last_dio_data, data, device->descriptor.dio_bytes);

  memcpy(payload, data, device->descriptor.dio_bytes);
  memcpy(&(payload[device->descriptor.dio_bytes]), out_mask, out_mask_size);
  memset(&(payload[device->descriptor.dio_bytes + out_mask_size]),
          0,
          out_mask_size);

  status = aiousb_generic_vendor_write (device, 
            AUR_DIO_CONFIG, b_tristate ? 1 : 0,
            0,
            payload_size,
            payload);

  if (status != payload_size)
  {
    aiousb_library_err_print("aiousb_generic_vendor_returned: %d payload_size: %d",
                status,
                payload_size);
  }

  free(payload);

  return 0;

}

int aiousb_dio_configure_ex (aiousb_device_handle device, void * out_mask,
                void *data, void *tristate_mask)
{
  uint8_t *payload;
  uint16_t out_mask_size;
  uint16_t tristate_size;
  uint16_t payload_size;
  int status;

  if (device->descriptor.dio_bytes == 0)
  {
    return -EBADRQC;
  }

  if ((out_mask == NULL) || (data == NULL) || (tristate_mask == NULL))
  {
    return -EINVAL;
  }

  out_mask_size = (device->descriptor.dio_config_bits + 7) / 8;
  tristate_size = (device->descriptor.tristate + 7) / 8;
  payload_size = device->descriptor.dio_bytes + out_mask_size + tristate_size;

  payload = (uint8_t *)malloc(payload_size);

  memcpy(device->last_dio_data, data, device->descriptor.dio_bytes);
  memcpy(&(payload[device->descriptor.dio_bytes]), out_mask, out_mask_size);
  memcpy(&(payload[device->descriptor.dio_bytes + out_mask_size]),
          tristate_mask,
          tristate_size);

  status = aiousb_generic_vendor_write(device,
            AUR_DIO_CONFIG,
            0,
            device->descriptor.dio_bytes,
            payload_size,
            payload);

  if (status != payload_size)
    {
      aiousb_library_err_print("aiousb_generic_vendor_returned: %d payload_size: %d", status, payload_size);
    }

  free(payload);

  return 0;

}

int aiousb_dio_configure_masked(aiousb_device_handle device, void *outs,
                void *outs_mask, void *data, void *data_mask, void *tristates,
                void *tristates_mask)
{
  uint8_t *payload, *current;
  uint16_t payload_size;
  int status;

  uint16_t outs_length, tristate_length;

  if ((device->descriptor.dio_bytes == 0) || device->descriptor.tristate == 0)
    {
      return -EBADRQC;
    }

  outs_length = (device->descriptor.dio_config_bits + 7) / 8;
  tristate_length = (device->descriptor.tristate + 7) / 8;
  payload_size = device->descriptor.dio_bytes * 2 +
                  outs_length * 2 +
                  tristate_length * 2;

  payload =(uint8_t *) malloc(payload_size);
  current = payload;

  if (outs_mask == NULL)
    {
      memset(current, 0, outs_length);
    }
  else
    {
      memcpy(current, outs_mask, outs_length);
    }
  current += outs_length;

  if (outs == NULL)
    {
      memset(current, 0, outs_length);
    }
  else
    {
      memcpy(current, outs, outs_length);
    }
  current += outs_length;

  if (data_mask == NULL)
    {
      memset(current, 0xff, device->descriptor.dio_bytes);
    }
  else
    {
      memcpy(current, data_mask, device->descriptor.dio_bytes);
    }
  current += device->descriptor.dio_bytes;

  if (data == NULL)
    {
      memset(current, 0, device->descriptor.dio_bytes);
    }
  else
    {
      memcpy(current, data, device->descriptor.dio_bytes);
    }
  current += device->descriptor.dio_bytes;

  if (tristates_mask == NULL)
    {
      memset(current, 0, tristate_length);
    }
  else
    {
      memcpy(current, tristates_mask, tristate_length);
    }
  current += tristate_length;

  if (tristates == NULL)
    {
      memset(current, 0xff, tristate_length);
    }
  else
    {
      memcpy(current, tristates, tristate_length);
    }

  status = aiousb_generic_vendor_write(device,
                              AUR_DIO_CONFIG_MASKED,
                              0,
                              0,
                              payload_size,
                              payload);

  if (status != payload_size)
    {
      aiousb_library_err_print("aiousb_generic_vendor_returned: %d payload_size: %d",
                  status,
                  payload_size);
      
    }
  else
    {
      status = 0;
    }

  return status;
}


int aiousb_dio_write_all(aiousb_device_handle device, void *data)
{
  int status;

  if (device->descriptor.dio_bytes == 0)
    {
      return -EBADRQC;
    }

  memcpy(device->last_dio_data, data, device->descriptor.dio_bytes);

  status = aiousb_generic_vendor_write(device, 
                                  AUR_DIO_WRITE,
                                  0,
                                  0,
                                  device->descriptor.dio_bytes,
                                  data);

  if (status != device->descriptor.dio_bytes)
    {
      aiousb_library_err_print("aiousb_generic_vendor_write returned: %d dio_bytes: %d",
                              status,
                              device->descriptor.dio_bytes);
    }
  else
    {
      status = 0;
    }

  return status;
} 

int aiousb_dio_write_8(aiousb_device_handle device, uint32_t byte_index,
                uint8_t data)
{
  int status;

  if (device->descriptor.dio_bytes == 0)
    {
      return -EBADRQC;
    }

  if (byte_index >= device->descriptor.dio_bytes)
    {
      return -EINVAL;
    }

  device->last_dio_data[byte_index] = data;

  status = aiousb_generic_vendor_write(device, 
                                      AUR_DIO_WRITE,
                                      0,
                                      0,
                                      device->descriptor.dio_bytes,
                                      device->last_dio_data);

  if (status != device->descriptor.dio_bytes)
    {
      aiousb_library_err_print("aiousb_generic_vendor_write return %d", status);
    }
  else
    {
      status = 0;
    }
  return status;
}
    

int aiousb_dio_write1(aiousb_device_handle device, uint32_t bit_index,
                uint8_t data)
{
  uint32_t byte_index;
  int status;

  if (device->descriptor.dio_bytes == 0)
    {
      return -EBADRQC;
    }

  byte_index = bit_index / 8;

  if (device->descriptor.dio_bytes < byte_index)
    {
      return -EINVAL;
    }

  if (data)
    {
      device->last_dio_data[byte_index] |= 1 << (bit_index & 0x7);
    }
  else
    {
      device->last_dio_data[byte_index] &= ~(1 << (bit_index & 0x7));
    }

  if ((device->b_firmware_20) &&
      (device->pnp_data.pnp_size >= offsetof(struct pnp_data, b_has_dio_write1)) &&
      (device->pnp_data.b_has_dio_write1))
      {
        status = aiousb_generic_vendor_write(device, 
                                              AUR_DIO_WRITE,
                                              (uint16_t)data,
                                              bit_index,
                                              0,
                                              NULL);

        if (status != 0)
          {
            aiousb_library_err_print("status = %d", status);
          }
        else
          {
            status = 0;
          }
      }
  else
    {
      status = aiousb_generic_vendor_write(device,
                                            AUR_DIO_WRITE,
                                            0,
                                            0,
                                            device->descriptor.dio_bytes,
                                            device->last_dio_data);
      if (status != device->descriptor.dio_bytes)
        {
          aiousb_library_err_print("status = %d", status);
        }
      else
        {
          status = 0;
        }
    }


  return status; 
}

int aiousb_dio_read_all(aiousb_device_handle device, void *data)
{
  int status;

  if (device->descriptor.dio_bytes == 0)
    {
      return -EBADRQC;
    }

  status = aiousb_generic_vendor_read(device,
                                      AUR_DIO_READ,
                                      0,
                                      0,
                                      device->descriptor.dio_bytes,
                                      data);

  if (status != device->descriptor.dio_bytes)
    {
      aiousb_library_err_print("aiousb_generic_vendor_read returned %d", status);
    }
  else
    {
      status = 0;
    }

  return status;
}

int aiousb_dio_read_8(aiousb_device_handle device, uint32_t byte_index,
                uint8_t *data)
{
  int status;
  uint8_t *all_bytes;

  if (device->descriptor.dio_bytes == 0)
    {
      return -EBADRQC;
    }

  if (byte_index >= device->descriptor.dio_bytes)
    {
      return -EINVAL;
    }

  all_bytes = (uint8_t *) malloc(device->descriptor.dio_bytes);

  status = aiousb_generic_vendor_read(device,
                                      AUR_DIO_READ,
                                      0,
                                      0,
                                      device->descriptor.dio_bytes,
                                      all_bytes);

  if (status != device->descriptor.dio_bytes)
    {
      aiousb_library_err_print("aiousb_gneric_vendor_read returned %d", status);
    }
  else
    {
      status = 0;
      *data = all_bytes[byte_index];
    }

  free(all_bytes);

  return status;
}

int aiousb_dio_read_1(aiousb_device_handle device, uint32_t bit_index,
                uint8_t *data)
{
  int status;
  uint8_t *all_bytes;

  if (device->descriptor.dio_bytes == 0)
    {
      return -EBADRQC;
    }

  if (bit_index >= device->descriptor.dio_bytes * 8)
    {
      return -EINVAL;
    }

  all_bytes = (uint8_t *) malloc(device->descriptor.dio_bytes);

  status = aiousb_generic_vendor_read(device,
                                      AUR_DIO_READ,
                                      0,
                                      0,
                                      device->descriptor.dio_bytes,
                                      all_bytes);

  if (status != device->descriptor.dio_bytes)
    {
      aiousb_library_err_print("aiousb_generic_vendor_read returned %d", status);
    }
  else
    {
      status = 0;
      if (all_bytes[bit_index / 8] & 1 << bit_index % 8)
        {
          *data = 1;
        }
      else
        {
          *data = 0;
        }
    }

    free(all_bytes);

    return status;
}

//TODO: Untested
int aiousb_dio_configuration_query(aiousb_device_handle device, void *out_mask,
                void *tristate_mask)
{
  int status;
  uint8_t *data;
  uint16_t out_length, tristate_length, length;

  if (device->descriptor.tristate == 0)
    {
      return -EBADRQC;
    }

  if ((out_mask == NULL) || (tristate_mask == NULL))
    {
      return -EINVAL;
    }

  out_length = (device->descriptor.dio_config_bits + 7) / 8;
  tristate_length = (device->descriptor.tristate + 7) / 8;
  length = out_length + tristate_length;
           

  data =(uint8_t *) malloc (length);

  status = aiousb_generic_vendor_read(device,
                                      AUR_DIO_CONFIG_QUERY,
                                      0,
                                      device->descriptor.dio_bytes,
                                      length,
                                      data);

  if (status != length)
   {
     aiousb_library_err_print("aiousb_generic_vendor_read returned %d", status);
   }
  else
    {
      memcpy(out_mask, data, out_length);
      memcpy(tristate_mask, data + out_length, tristate_length);
      status = 0;
    }

  free(data);

  return status;
}

//TODO: Untested
int aiousb_dio_stream_open(aiousb_device_handle device, uint32_t is_read)
{
  int status;
  if (!device->descriptor.b_dio_stream)
    {
      return -EBADRQC;
    }

  if (device->b_dio_open)
    {
      return -EMFILE; 
    }

  if (is_read)
    {
      status = aiousb_generic_vendor_write(device, 0xbc, 0, 0, 0, NULL);
    }
  else
    {
      status = aiousb_generic_vendor_write(device, 0xbb, 0, 0, 0, NULL);
    }

  if (!status)
    {
      device->b_dio_open = 1;
    }
  else
    {
     aiousb_library_err_print("aiousb_generic_vendor_read returned %d", status);
    }

  return status;
}

//TODO: Untested
int aiousb_dio_stream_close(aiousb_device_handle device)
{
  if (!device->descriptor.b_dio_stream)
    {
      return -EBADRQC;
    }

  if (!device->b_dio_open)
    {
      return -ENOENT; 
    }

  device->b_dio_open = 0;
  return 0;
}

uint16_t motorala_word(uint16_t w)
{
  return w >> 8 | w << 8;
}

//TODO: Untested
uint16_t oct_dac_from_freq(double *freq)
{
  int octave, offset;
  uint16_t retval;

  if (*freq == 0)
    {
      return 0;
    }

  if (*freq > 40000000)
    {
      *freq = 40000000;
    }

  octave = floor(3.322 * log10(*freq / 1039.0));

  if (octave < 0)
    {
      octave = 0;
      offset = 0;
      retval = 0;
    }
  else
    {
      offset = round(2048 - (ldexp(2078, 10 + octave) / *freq));
      retval = (octave << 12) | (offset << 2);
      retval = motorala_word(retval);
    }
    *freq = (double)(2078 << octave) / (2.0 - (double)offset / 1024.0);
    return retval;
}

//TODO: Untested
int aiousb_dio_stream_set_clocks(aiousb_device_handle device, double *read_hz,
                double *write_hz)
{
  int status;
  struct dio_clock_data clock_data = {0};

  if (!device->descriptor.b_dio_stream)
    {
      return -EBADRQC;
    }

  clock_data.disables = 0;
  clock_data.disables |= 0x1;
  clock_data.disables |= 0x2;
  clock_data.write_oct_dac = oct_dac_from_freq(write_hz);
  clock_data.read_oct_dac = oct_dac_from_freq(read_hz);

  status = aiousb_generic_vendor_read(device,
                                  AUR_DIO_SETCLOCKS,
                                  0,
                                  0,
                                  sizeof(struct dio_clock_data),
                                  &clock_data);

  if (status != sizeof(struct dio_clock_data))
    {
     aiousb_library_err_print("aiousb_generic_vendor_read returned %d", status);
    }
  return status;

}

//TODO: Untested. This needs to be tested before exposing to customers.
//TODO: figure out the right data type for frame_data
int aiousb_dio_stream_frame (aiousb_device_handle device, unsigned long frame_points,
                unsigned short *frame_data, size_t *bytes_transferred)
{
  int (*fptr)(aiousb_device_handle, unsigned int, void *, int, int*);
  unsigned int pipe_index;
  int status;
  int this_transfer;
  

  if (device->descriptor.b_dio_stream == 0)
    {
      return -EBADRQC;
    }

  if (device->b_dio_open == 0)
    {
      return -ENOENT;
    }

  if (frame_points == 0)
    {
      return -EINVAL;
    }

  if (device->b_dio_read == 1)
    {
      pipe_index = 0x86;
      fptr = &aiousb_generic_bulk_in;
    }
  else
    {
      pipe_index = 0x02;
      fptr = &aiousb_generic_bulk_out;
    }

  *bytes_transferred = 0;

  do
    {
      status = (*fptr)(device, 
              pipe_index, 
              frame_data + *bytes_transferred,
              frame_points > device->streaming_block_size ? device->streaming_block_size : frame_points,
              &this_transfer);

      if ((status < 0) || ( 0 == this_transfer))
        {
          aiousb_library_err_print("Error calling bulk transfer");
          break;
        }

      frame_points = frame_points - this_transfer;
      bytes_transferred += this_transfer;
    }while (frame_points > 0);

    return status;

}

//TODO: Untested. This needs to be tested before exposing to customers.
int map_counter_block(aiousb_device_handle device, uint32_t *block_index, 
                uint32_t *counter_index)
{
  if (!(*block_index))
    {
      *block_index = *counter_index / 3;
      *counter_index = *counter_index % 3;
    }

  if ((device->descriptor.counters < *block_index) || (*counter_index > 3))
    {
      return -EINVAL;
    }

    return 0;
}

//TODO: Untested. This needs to be tested before exposing to customers.
int aiousb_ctr_8254_mode(aiousb_device_handle device, uint32_t block_index, 
                uint32_t counter_index, uint32_t mode)
{
  int status;

  if (device->descriptor.counters == 0)
    {
      return -EBADRQC;
    }

  if (mode >= 6)
    {
      return -EINVAL;
    }
  
  mode = (counter_index << 6) | (mode << 1) | 0x30;

  status = map_counter_block(device, &block_index, &counter_index);

  if (status)
    {
      aiousb_library_err_print("map_counter_block returned %d", status);
      return status;
    }

  status = aiousb_generic_vendor_write(device,
                          AUR_CTR_MODE,
                          block_index | (mode << 8),
                          0,
                          0,
                          NULL);

  return status;
}

//TODO: Untested. This needs to be tested before exposing to customers.
int aiousb_ctr_8254_load(aiousb_device_handle device, uint32_t block_index,
                uint32_t counter_index, uint16_t load_value)
{
  int status;
  uint8_t mode;

  if (device->descriptor.counters == 0)
    {
      return -EBADRQC;
    }

  status = map_counter_block (device, &block_index, &counter_index);

  if (status)
    {
      aiousb_library_err_print("map_counter_block returned %d", status);
      return status;
    }

  mode = counter_index << 6;

  status = aiousb_generic_vendor_write(device, 
                                    AUR_CTR_LOAD,
                                    block_index | (mode << 8),
                                    load_value,
                                    0,
                                    NULL);

  return status;
}

//TODO: Untested. This needs to be tested before exposing to customers.
int aiousb_ctr_8254_mode_load(aiousb_device_handle device, uint32_t block_index,
                uint32_t counter_index, uint32_t mode, uint16_t load_value)
{
  int status;

  if (device->descriptor.counters == 0)
    {
      return -EBADRQC;
    }

  status = map_counter_block (device, &block_index, &counter_index);

  if (status)
    {
      aiousb_library_err_print("map_counter_block returned %d", status);
      return status;
    }

  mode = (counter_index << 6) | (mode << 1) | 0x30;

  status = aiousb_generic_vendor_write(device,
                                      AUR_CTR_MODELOAD,
                                      block_index | (mode << 8),
                                      load_value,
                                      0,
                                      NULL);

  return status;
}  

//TODO: Untested. This needs to be tested before exposing to customers.
int aiousb_ctr_8254_start_output_frequency(aiousb_device_handle device, 
                uint32_t block_index, double *frequency)
{
  int status;
  uint32_t temp, divisor_a, divisor_b;
  double err, min_err, hz;
  long double divisor_ab;

  if (device->descriptor.counters == 0)
    {
      return -EBADRQC;
    }

  if ((device->descriptor.counters <= block_index) || (frequency == NULL))
    {
      return -EINVAL;
    }

  if (*frequency <= 0)
    {
      aiousb_ctr_8254_mode(device, block_index, 1, 2);
      status = aiousb_ctr_8254_mode(device, block_index, 2, 3);
    }
  else
    {
      if (*frequency * 4 > device->descriptor.root_clock)
        {
          divisor_a = divisor_b = 2;
        }
      else
        {
          divisor_ab = device->descriptor.root_clock / *frequency;
          temp = round(sqrt(divisor_ab));
          divisor_a = round(divisor_ab / temp);
          divisor_b = temp;
          min_err = abs(*frequency - (device->descriptor.root_clock / (divisor_a * *frequency)));

          for ( ; temp > 2 ; temp--)
            {
              divisor_a = round(divisor_ab / *frequency);
              if (divisor_a > 0xffff) //16 bit limit
              {
                break;
              }
              err = abs(*frequency - (device->descriptor.root_clock / (divisor_a * *frequency)));
              if (0 == err)
                {
                  divisor_b = temp;
                  break;
                }
              if (err < min_err)
                {
                  divisor_b = temp;
                  min_err = err;
                }
            }
            divisor_a = round(divisor_ab / divisor_b);
        }
        *frequency = device->descriptor.root_clock / (divisor_a * divisor_b);

        aiousb_ctr_8254_mode_load(device, block_index, 1, 2, divisor_a);
        status = aiousb_ctr_8254_mode_load(device, block_index, 2, 3, divisor_b);
    }
    return status;
}

//TODO: Untested. This needs to be tested before exposing to customers.
int aiousb_ctr_8254_read(aiousb_device_handle device, uint32_t block_index,
                uint32_t counter_index, uint16_t *read_value)
{
  int status;

  if (device->descriptor.counters == 0)
    {
      return -EBADRQC;
    }

  status = map_counter_block(device, &block_index, &counter_index);

  if (status)
    {
      aiousb_library_err_print("map_counter_block returned %d", status);
      return status;
    }

  status = aiousb_generic_vendor_read(device,
                                    AUR_CTR_READ,
                                    block_index | (counter_index << 8),
                                    0,
                                    sizeof(read_value),
                                    read_value);

  if (status != sizeof(read_value))
    {
      aiousb_library_err_print("aiousb_generic_vendor_read returned %d", status);
    }
  else
    {
      status = 0;
    }
  
  return status;
}

//TODO: Untested. This needs to be tested before exposing to customers.
int aiousb_ctr_8254_read_all(aiousb_device_handle device, uint16_t *data)
{
  int status;
  uint16_t length;

  if (device->descriptor.counters == 0)
    {
      return -EBADRQC;
    }

  length = device->descriptor.counters * 2 * 3;

  status = aiousb_generic_vendor_read(device,
                                    AUR_CTR_READALL,
                                    0,
                                    0,
                                    length,
                                    data);

  if (status != length)
    {
      aiousb_library_err_print("aiousb_generic_vendor_read returned %d", status);
    }
  else
    {
      status = 0;
    }
  
  return status;
}

int aiousb_ctr_8254_read_status(aiousb_device_handle device, uint32_t block_index,
                uint32_t counter_index, uint8_t *read_value, uint8_t *status)
{
  int aiousb_status;
  uint8_t payload[3] = {0};
  

  if (device->descriptor.counters == 0)
    {
      return -EBADRQC;
    }

  aiousb_status = map_counter_block(device, &block_index, &counter_index);

  if (aiousb_status)
    {
      aiousb_library_err_print("map_counter_block returned %d", aiousb_status);
      return aiousb_status;
    }

  aiousb_status = aiousb_generic_vendor_read(device,
                                    AUR_CTR_READ,
                                    block_index | (counter_index << 8),
                                    0,
                                    sizeof(payload),
                                    &payload);

  if (aiousb_status != sizeof(payload))
    {
      aiousb_library_err_print("aiousb_generic_vendor_read returned %d", aiousb_status);
    }
  else
    {
      aiousb_status = 0;
      *read_value = payload[0];
      *status = payload[2];
    }

  return aiousb_status;
}

int aiousb_ctr_8254_read_mode_load(aiousb_device_handle device, uint32_t block_index,
                uint32_t counter_index, uint32_t mode, uint16_t load_value,
                uint16_t *read_value)
{
  int status;

  if (device->descriptor.counters == 0)
    {
      return -EBADRQC;
    }

  if (mode >= 6)
    {
      return -EINVAL;
    }

  status = map_counter_block(device, &block_index, &counter_index);

  if (status)
    {
      aiousb_library_err_print("map_counter_block returned %d", status);
      return status;
    }

  mode = (counter_index << 6) | (mode << 1) | 0x30;

  status = aiousb_generic_vendor_read(device,
                                AUR_CTR_MODELOAD,
                                block_index | (counter_index << 8),
                                load_value,
                                sizeof(*read_value),
                                read_value);

  if (status != sizeof(*read_value))
    {
      aiousb_library_err_print("aiousb_generic_vendor_read returned %d", status);
    }
  else
    {
      status = 0;
    }

  return 0;
}

//Not in header file because it is deprecated, but needed by
//adc_get_scan_inner_adc_bulk
int aiousb_adc_get_immediate(aiousb_device_handle device, uint32_t channel, 
                        uint16_t *buffer)
{
  if (device->descriptor.b_adc_bulk)
    {
      return -EBADRQC;
    }

    //TODO: Verify that worker threads are not active? Not sure if it is
    //required since this function should only be called from within the 
    //library

    if (buffer == NULL)
      {
        return -EINVAL;
      }

    return aiousb_generic_vendor_write(device,
                            AUR_ADC_IMMEDIATE,
                            0,
                            channel,
                            0,
                            NULL);
}



void *timeout_worker (void *context)
{
  struct time_out_context *time_out_context = (struct time_out_context *)context;
  struct timespec now, end_time;
  int status;

  status = clock_gettime(CLOCK_MONOTONIC, &now);

  if (status)
  {
    aiousb_library_err_print("clock_gettime returned %d", status);
    return NULL;
  }

  timespec_add_msec(&end_time, &now, time_out_context->time_out_ms);

  pthread_mutex_lock(&time_out_context->mutex);
  while ((status != -ETIMEDOUT) && (!time_out_context->b_terminate))
    {
      pthread_cond_timedwait(&time_out_context->cond,
                          &time_out_context->mutex,
                          &end_time);
    };
  pthread_mutex_unlock(&time_out_context->mutex);

  if (status == -ETIMEDOUT)
    {
      aiousb_debug_print("Timeout occurred");
      time_out_context->b_timeout = 1;
      aiousb_generic_vendor_write(time_out_context->device,
                              AUR_ADC_IMMEDIATE,
                              0,
                              0,
                              0,
                              NULL);
    }
}

//TODO: Needs to be tested
int aiousb_get_scan_inner_adc_bulk(aiousb_device_handle device, uint8_t *config_buff, 
                  uint32_t *config_size, uint16_t **ad_buff,
                  uint32_t *ad_buff_length, uint8_t start_channel,
                  uint8_t end_channel, uint32_t time_out_ms)
{
  int status;
  pthread_t timeout_thread = 0;
  struct time_out_context time_out_context = {0};
  uint8_t channel_count;
  int bytes_remaining;
  uint32_t bc_data;
  int transferred = 0;
  uint8_t *byte_ptr;


  status = aiousb_adc_get_config(device, config_buff, config_size);

  if (status)
    {
      return status;
    }

  if (time_out_ms)
    {
      config_buff[0x11] |= 0x4;
    }
  else
    {
      config_buff[0x11] = 0x4 | config_buff[0x11] & ~0x3;
    }
  
  start_channel = config_buff[0x12] & 0xf;
  end_channel = config_buff[0x12] >> 4;

  if (*config_size >= 21)
    {
      start_channel |= (config_buff[20] & 0xf) << 4;
      end_channel |= config_buff[20] & 0xf0;
    }

  channel_count = end_channel - start_channel + 1;
  config_buff[0x13] = 1 > config_buff[0x13] ? 1 : config_buff[0x13];

  if (channel_count * (1 + config_buff[0x13]) > 1024)
    {
      config_buff[0x13] = 1024 / channel_count - 1;
    }
  *ad_buff_length = channel_count * (1 + config_buff[0x13]);
  *ad_buff = (uint16_t *) malloc(*ad_buff_length * sizeof(uint16_t));

  if (*ad_buff == NULL)
    {
      aiousb_library_err_print("Couldn't allocate buffer");
      return -ENOMEM;
    }
  bytes_remaining = *ad_buff_length * sizeof(uint16_t);
  byte_ptr = (uint8_t *)(*ad_buff);

  status = aiousb_adc_set_config(device, config_buff, config_size);

  bc_data = 0x5;

  status = aiousb_generic_vendor_write(device,
                                0xbc,
                                0,
                                bytes_remaining,
                                sizeof(bc_data),
                                &bc_data);

  if (status)
  {
    goto ERR_OUT;
  }

  if (0 != time_out_ms)
      {
        time_out_context.device = device;
        time_out_context.time_out_ms = time_out_ms;
        pthread_cond_init(&time_out_context.cond, NULL);
        pthread_mutex_init(&time_out_context.mutex, NULL);
        pthread_create(&timeout_thread, NULL, timeout_worker, &time_out_context);
      }
  else
    {
      aiousb_adc_get_immediate(device, 0, *ad_buff);
    }

  do
    {
        status = aiousb_generic_bulk_in(device,
                            0,
                            byte_ptr,
                            bytes_remaining,
                            &transferred);
        if (status)
          {
            break;
          }
        bytes_remaining -= transferred;
        byte_ptr += transferred;
        transferred = 0;
    }while (bytes_remaining > 0);

    if (time_out_ms)
    {
      pthread_mutex_lock(&time_out_context.mutex);
      if (time_out_context.b_timeout)
        {
          aiousb_library_err_print("Thread timed out");
          status = -ETIMEDOUT;
        }
      else
        {
          time_out_context.b_terminate = 1;
          pthread_cond_signal(&time_out_context.cond);
        }
      pthread_mutex_unlock(&time_out_context.mutex);
    }

ERR_OUT:
  if (status)
    {
      if (*ad_buff != NULL)
        {
          free(ad_buff);
        }
    }


}

//Not in header file because it is deprecated, but needed by
//adc_get_scan_inner_adc_dio_stream
int aiousb_adc_bulk_acquire(aiousb_device_handle device, uint32_t *buff_size,
                      uint16_t *buff)
{
  if (!(device->descriptor.b_adc_bulk || device->descriptor.b_adc_dio_stream))
    {
      return -EBADRQC;
    }

  //TODO: Verify that worker threads are in the state we want? Not sure if it is
  //required since this function should only be called from within the library

  memset(&device->adc_worker_context, 0, sizeof(struct adc_worker_context));

  device->adc_worker_context.device = device;
  device->adc_worker_context.pipe_index = 0;
  device->adc_worker_context.bytes_left = *buff_size;
  device->adc_worker_context.tar = buff;
  device->adc_worker_context.block_size = device->streaming_block_size;
  if (device->descriptor.b_adc_dio_stream)
    {
      device->adc_worker_context.bcs_style = bcs_dio;
      if (device->adc_worker_context.block_size & 0x100)
        {
          //For historical reasons, DIO streaming block size is in words, ADC
          //streaming block size is in bytes. The user sees StreamingBlockSize
          //ADC-style(insofar as they see it at all), but 
          //AIOUSB_SetStreamingBlockSize effectively left it in integral
          //half-packets. So, if there's a stray half-packet, then give it 
          //another half-packet to make integral packets.
          device->adc_worker_context.block_size += 0x100;
        }
    }
  else
    {
      device->adc_worker_context.bcs_style = bcs_adc;
    }

  pthread_mutex_init(&device->adc_worker_context.mutex, NULL);
  pthread_mutex_init(&device->adc_worker_context.cond_mutex, NULL);
  pthread_cond_init(&device->adc_worker_context.cond, NULL);
  pthread_create(&device->adc_worker,
              NULL,
              adc_worker_execute,
              &device->adc_worker_context);

  


}

static const int ADC_DIO_OVERSAMPLE = 0x40;

int aiousb_get_scan_inner_adc_dio_stream(aiousb_device_handle device, uint8_t *config_buff, 
                  uint32_t *config_size, uint16_t **ad_buff,
                  uint32_t *ad_buff_length, uint8_t start_channel,
                  uint8_t end_channel, uint32_t time_out_ms)
{
  struct adc_intermmediate_buff *inter_buff = NULL;
  uint32_t inter_buff_length = ADC_DIO_OVERSAMPLE;
  int status = 0;
  struct timespec now, end_time;
  int i;

  if (time_out_ms == 0)
    {
      time_out_ms = 1000;
    }

  inter_buff_length = ADC_DIO_OVERSAMPLE;
  inter_buff = (adc_intermmediate_buff *) malloc(inter_buff_length);

  if (inter_buff == NULL)
    {
      return -ENOMEM;
    }

  status = aiousb_adc_bulk_acquire(device,
                          &inter_buff_length,
                          (uint16_t *)inter_buff);
  if (status != 0)
  {
    free(ad_buff);
    return status;
  }

  status = clock_gettime(CLOCK_MONOTONIC, &now);

  if (status)
  {
    aiousb_library_err_print("clock_gettime returned %d", status)
    pthread_mutex_lock(&device->adc_worker_context.mutex);
    device->adc_worker_context.b_abort = 1;
    pthread_mutex_unlock(&device->adc_worker_context.mutex);
    //TODO: Figure out who frees the memory?
  }

  timespec_add_msec(&end_time, &now, time_out_ms);

  pthread_mutex_lock(&device->adc_worker_context.cond_mutex);
  while ((status != -ETIMEDOUT) && (device->adc_worker_context.bytes_left != 0))
  {
    pthread_cond_timedwait(&device->adc_worker_context.cond,
                        &device->adc_worker_context.cond_mutex,
                        &end_time);
  };
  pthread_mutex_unlock(&device->adc_worker_context.cond_mutex);

  if (status == -ETIMEDOUT)
    {
      //TODO:Figure out who frees the memory?
      return status;
    }

  //The reference code calls a function adc_bulk_poll that looks like it just
  //repeats the wait done above with a timout of zero
  start_channel = 0;
  end_channel = 1;
  memset(config_buff, 0, *config_size);

  aiousb_adc_get_config(device, config_buff, config_size);
  config_buff[0x13] = ADC_DIO_OVERSAMPLE - 1;

  *ad_buff = (uint16_t *)malloc(2*ADC_DIO_OVERSAMPLE);

  for (i = 0 ; i < ADC_DIO_OVERSAMPLE ; i++)
    {
      *ad_buff[i] = inter_buff[i].ad0;
      *ad_buff[ADC_DIO_OVERSAMPLE + i] = inter_buff[i].ad1;
    }
    config_buff[0x13] = 0;
}

int aiousb_get_scan_inner_imm_adcs (aiousb_device_handle device, 
                  uint8_t *config_buff, uint32_t *config_size, 
                  uint16_t **ad_buff, uint32_t *ad_buff_length,
                  uint8_t start_channel, uint8_t end_channel, 
                  uint32_t time_out_ms)
{
  int status;
  *ad_buff =(uint16_t *) malloc(device->descriptor.imm_dacs);
  int i;

  status = aiousb_generic_vendor_read(device,
                                    AUR_ADC_IMMEDIATE,
                                    0,
                                    0,
                                    device->descriptor.imm_adcs * 2,
                                    *ad_buff);

  start_channel = 0;
  end_channel = device->descriptor.imm_adcs - 1;

  memset(config_buff, 0x02, 16);
  config_buff[0x13] = 0;

}

//TODO: Make start_channel and end_channel pointers
//TODO: Figure out where to join threads in the functions called here
int aiousb_get_scan_inner(aiousb_device_handle device, uint8_t *config_buff, 
                  uint32_t *config_size, uint16_t **ad_buff,
                  uint32_t *ad_buff_length, uint8_t start_channel,
                  uint8_t end_channel, uint32_t time_out_ms)
{
  int status;

  if (device->descriptor.b_adc_bulk)
    {
      return aiousb_get_scan_inner_adc_bulk(device,
                            config_buff,
                            config_size,
                            ad_buff,
                            ad_buff_length,
                            start_channel,
                            end_channel,
                            time_out_ms);
    }
  else if (device->descriptor.b_adc_dio_stream)
    {
      return aiousb_get_scan_inner_adc_dio_stream(device,
                            config_buff,
                            config_size,
                            ad_buff,
                            ad_buff_length,
                            start_channel,
                            end_channel,
                            time_out_ms);
    }
  else if (device->descriptor.imm_adcs != 0)
    {
      return aiousb_get_scan_inner_imm_adcs(device,
                            config_buff,
                            config_size,
                            ad_buff,
                            ad_buff_length,
                            start_channel,
                            end_channel,
                            time_out_ms);
    }
  else
    {
      return -EBADRQC;
    }
}


int aiousb_get_scan_v(aiousb_device_handle device, double *data)
{
  int status = 0;
  uint8_t config_buff[MAX_CONFIG_SIZE];
  uint32_t config_size;
  uint8_t start_channel, end_channel;
  uint16_t *ad_buff = NULL;
  uint32_t ad_buff_length = 0;
  int i, j, channel, total;

  config_size = device->descriptor.config_bytes;

  status = aiousb_get_scan_inner(device,
                                config_buff,
                                &config_size,
                                &ad_buff,
                                &ad_buff_length,
                                start_channel,
                                end_channel,
                                0);

  if (status)
    {
      aiousb_library_err_print("aiousb_get_scan_inner returned %d", status);
      goto ERR_OUT;
    }

  if (config_buff[0x13] != 0)
    {
      i = 0;
      for (channel = start_channel ; channel <= end_channel ; channel++)
        {
          total = 0;
          for (j = 1 ; j <= config_buff[0x13] ; j++)
            {
              total += ad_buff[i * ( 1 + config_buff[0x13] ) + j];
            }
          data[channel] = round(total/config_buff[0x13]);
          i++;
        }
    }
  else
    {
      for (channel = start_channel ; channel <= end_channel ; channel++)
        {
          data[channel] = ad_buff[channel - start_channel];
        }
    }

ERR_OUT:

  if (ad_buff)
    {
      free(ad_buff);
    }
  return status;

}

int aiousb_get_channel_v (aiousb_device_handle device, uint32_t channel_index,
                double *volts)
{
  double data[MAX_CHANNELS_FOR_SCAN];

  aiousb_get_scan_v(device, data);
  *volts = data[channel_index];
}

int aiousb_get_trig_scan_v (aiousb_device_handle device, double *data,
                uint32_t timeout_ms)
{
  uint8_t config_buff[MAX_CONFIG_SIZE];
  uint32_t config_buff_size;
  uint16_t *ad_buff = NULL;
  uint32_t ad_buff_length;
  int status = 0;
  uint8_t range_shift, range_code;
  uint8_t start_channel, end_channel;
  int channel, total;
  int i,j;
  double volts;

  if (timeout_ms <= 0)
    {
      timeout_ms = 1;
    }
  
  config_buff_size = device->descriptor.config_bytes;
  status = aiousb_get_scan_inner(device,
                          config_buff,
                          &config_buff_size,
                          &ad_buff,
                          &ad_buff_length,
                          start_channel,
                          end_channel,
                          timeout_ms);

  if (status)
    {
      aiousb_library_err_print("aiousb_get_scan_inner returned %d", status);
      goto ERR_OUT;
    }

  range_shift = device->descriptor.range_shift;

  if (config_buff[0x13] != 0)
    {
      i = 0;
      for (channel = start_channel ; start_channel <= end_channel ; channel++)
        {
          total = 0;
          range_code = config_buff[channel >> range_shift];

          for (j = 1 ; j <= config_buff[0x13] ; j++)
            {
              total += ad_buff[i * ( 1 + config_buff[0x13] ) + j];
            }
          volts = total / config_buff[0x13] * ( 1 /65536.0);
          if (range_code & 1) volts = volts * 2 - 1;
          if (!(range_code & 2)) volts = volts * 2;
          if (!(range_code & 4)) volts = volts * 5;

          data[channel] = volts;
          i++;
        }
    }
    else
      {
        i = 0;
        for (channel = start_channel ; start_channel <= end_channel ; channel++)
          {
            volts = ad_buff[i] * (5/65536.0);
            data[channel] = volts;
          }
      }


ERR_OUT:
  if (ad_buff != NULL)
    {
      free(ad_buff);
    }
    return status;
}

int aiousb_adc_bulk_continuous_start_inner (aiousb_device_handle device,
                    uint32_t buff_size, uint32_t base_buff_count,
                    uint32_t context, adc_cont_callback callback, double *hertz)
{

}

int aiousb_adc_bulk_continuous_start (aiousb_device_handle device,
                uint32_t buff_size, uint32_t base_buff_count, uint32_t context,
                adc_cont_callback callback)
{

}

int aiousb_set_scan_limits (aiousb_device_handle device, uint32_t start_channel,
                uint32_t end_channel)
{
  uint8_t config_buff[MAX_CONFIG_SIZE] = {0};
  int status;

  if (!(device->descriptor.b_adc_bulk))
    {
      return -EBADRQC;
    }

  if ((start_channel > end_channel) || 
      (end_channel > device->descriptor.adc_mux_channels))
    {
      return -EINVAL;
    }

  status = aiousb_generic_vendor_read(device,
                                  AUR_ADC_GET_CONFIG,
                                  0,
                                  0,
                                  device->descriptor.config_bytes,
                                  config_buff);


  if (status != device->descriptor.config_bytes)
    {
      aiousb_library_err_print("aiousb_generic_vendor_read returned %d", status);
    }
  else
    {
      status = 0;
    }

  if (device->descriptor.config_bytes < 0x15)
    {
      config_buff[0x12] = start_channel | (end_channel << 4);
    }
  else
    {
      config_buff[0x12] = (start_channel & 0xf) | (end_channel << 4);
      config_buff[0x14] = (start_channel >> 4) | (end_channel & 0xf0);
    }
    
  status = aiousb_generic_vendor_write(device,
                                    AUR_ADC_SET_CONFIG,
                                    0,
                                    0,
                                    device->descriptor.config_bytes,
                                    config_buff);

  if (status != device->descriptor.config_bytes)
    {
      aiousb_library_err_print("aiousb_generic_vendor_write returned %d", status);
    }
  else
    {
      status = 0;
    }

  return status;
}

int aiousb_adc_get_config(aiousb_device_handle device, uint8_t *config_buff, 
                uint32_t *config_size)
{
  int status;

  if (device->descriptor.config_bytes == 0)
    {
      return -EBADRQC;
    }

  if (*config_size <  device->descriptor.config_bytes)
    {
      *config_size = device->descriptor.config_bytes;
      return -ENOMEM;
    }

  if (config_buff == NULL)
    {
      return -EINVAL;
    }

  status = aiousb_generic_vendor_read(device,
                                    AUR_ADC_GET_CONFIG,
                                    0,
                                    0,
                                    device->descriptor.config_bytes,
                                    config_buff);

  if (status != device->descriptor.config_bytes)
    {
      aiousb_library_err_print("aiousb_generic_vendor_read returned %d", status);
    }
  else
    {
      status = 0;
    }

  return status;
}

int aiousb_adc_set_config(aiousb_device_handle device, uint8_t *config_buff,
              uint32_t *config_size)
{
  int status;
  if (device->descriptor.config_bytes == 0)
    {
      return -EBADRQC;
    }

  if (*config_size < device->descriptor.config_bytes)
    {
      *config_size = device->descriptor.config_bytes;
      return -ENOMEM;
    }

  if (config_buff == NULL)
    {
      return -EINVAL;
    }

  status = aiousb_generic_vendor_write(device,
                                  AUR_ADC_SET_CONFIG,
                                  0,
                                  0,
                                  device->descriptor.config_bytes,
                                  config_buff);

  if (status != device->descriptor.config_bytes)
    {
      aiousb_library_err_print("aiousb_generic_vendor_read returned %d", status);
    }
  else
    {
      status = 0;
    }

  return status;
}

int aiousb_adc_range_all(aiousb_device_handle device, uint8_t *gain_codes, 
                uint32_t *b_differential)
{
  
  int status;
  int i;

  if ((device->descriptor.adc_channels == 0)  ||
      (!(device->descriptor.b_adc_bulk || device->descriptor.b_adc_dio_stream)))
    {
      return -EBADRQC;
    }

  if (gain_codes == NULL)
    {
      return -EINVAL;
    }

  for (i = 0 ; i < device->descriptor.adc_channels ; i++)
    {
      if (gain_codes[i] & 0xf8)
        {
          return -EINVAL;
        }
    }

  if (device->descriptor.b_adc_bulk)
    {
      uint8_t *config_buff = NULL;
      uint32_t config_size = 0;

      config_size = device->descriptor.config_bytes;
      config_buff = (uint8_t *)malloc(config_size);

      status = aiousb_adc_get_config(device, config_buff, &config_size);

      if (status)
      {
        aiousb_library_err_print("aiousb_adc_get_config returned %d", status);
      }
      
      for (i = 0 ; i < device->descriptor.adc_channels ; i++)
        {
          config_buff[i] = gain_codes[i] | (b_differential ? 1 : 0) << 3;
        }

      status = aiousb_adc_set_config(device, config_buff, &config_size);
      if (status)
        {
          aiousb_library_err_print("aiousb_adc_set_config returned %d", status);
        }

      free(config_buff);
    }
  else
    {
      status = aiousb_generic_vendor_write(device,
                                        AUR_ADC_SET_CONFIG,
                                        0,
                                        0,
                                        device->descriptor.config_bytes,
                                        gain_codes);

      if (status != device->descriptor.config_bytes)
        {
          aiousb_library_err_print("aiousb_generic_vendor_write returned %d", status);
        }
      else
        {
          status = 0;
        }
    }

    return status;
}


int aiousb_adc_set_oversample(aiousb_device_handle device, uint8_t oversample)
{
  uint8_t *config_buff = NULL;
  uint32_t config_size = 0;
  int status;

  if (!(device->descriptor.b_adc_bulk))
    {
      return -EBADRQC;
    }

  config_size = device->descriptor.config_bytes;
  config_buff =(uint8_t *) malloc(config_size);

  status = aiousb_adc_get_config(device, config_buff, &config_size);

  if (status)
    {
      aiousb_library_err_print("aiousb_adc_get_config returned %d", status);
    }  


  config_buff[0x13] = oversample;

  status = aiousb_adc_set_config(device, config_buff, &config_size);

  if (status)
    {
      aiousb_library_err_print("aiousb_adc_set_config returned %d", status);
    }

  free(config_buff);
}




int aiousb_adc_range1(aiousb_device_handle device, uint32_t adc_channel,
                uint8_t gain_code, uint32_t b_differential)
{
  uint8_t *config_buff = NULL;
  uint32_t config_size = 0;
  int status;
  
  if (!(device->descriptor.b_adc_bulk || device->descriptor.b_adc_dio_stream))
    {
      return -EBADRQC;
    }

  if (adc_channel > device->descriptor.adc_channels)
    {
      return -EINVAL;
    }

  if (gain_code & 0xf8)
    {
      return -EINVAL;
    }
  
  config_size = device->descriptor.config_bytes;
  config_buff = (uint8_t *) malloc(config_size);

  status = aiousb_adc_get_config(device, config_buff, &config_size);

  if (status)
    {
      aiousb_library_err_print("aiousb_adc_get_config returned %d", status);
    }

  config_buff[adc_channel % device->descriptor.adc_channels] = 
                      gain_code | (b_differential ? 1 : 0) << 3;

  status = aiousb_adc_set_config(device, config_buff, &config_size);

  if (status)
    {
      aiousb_library_err_print("aiousb_adc_set_config returned %d", status);
    }

  free(config_buff);

  return status;
}
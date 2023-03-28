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
#include <dirent.h>
#if NO_HOTPLUG != 1
#include <libudev.h>
#endif

#include <array>
#include <fstream>

#include "aiousb.h"
#include "accesio_usb_ioctl.h"
#include "aiousb-private.h"

#include "timespec-util.h"

namespace AIOUSB {

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

static const double dac_dio_stream_imm_hz = 1024*1024;

#define AIOUSB_MAX_PATH 269  /*It's usually less than 30. compiler complains if less than 269*/
#define AIOUSB_MAX_DEVICES 32   /*Maybe make this dynamic someday */


static aiousb_device_handle aiousb_devices[AIOUSB_MAX_DEVICES];
static pthread_mutex_t aiousb_devices_lock;
static int aiousb_init_complete;
std::thread hotplug_thread;
std::atomic<bool> exiting;
int pipe_fds[2];

#define ACCES_USB_DEV_DIR "/dev/accesio/"

int aiousb_device_open (const char *fname, aiousb_device_handle *device);

void lib_exit( void )
{
  exiting = true;
  write(pipe_fds[1], "exit", strlen("exit"));
#if NO_HOTPLUG != 1
  hotplug_thread.join();
#endif
}

int aiousb_device_present (int device_index)
{
  if (aiousb_devices[device_index] == NULL) return 0;
  return (!(aiousb_devices[device_index]->fd == -1));
}

int aiousb_get_available_device_index()
{
  int device_index = 0;
  while (device_index < AIOUSB_MAX_DEVICES && aiousb_device_present(device_index) ) device_index++;
  if (AIOUSB_MAX_DEVICES == device_index)
  {
    aiousb_library_err_print("No device indexes available\n");
  }
  return device_index;
}

void scan_devices()
{
  //scan the directory for names
  DIR *dir = opendir(ACCES_USB_DEV_DIR);
  char fname[AIOUSB_MAX_PATH];
  struct dirent *entry;

  if (dir == nullptr) return;

   pthread_mutex_lock(&aiousb_devices_lock);
   while ((entry = readdir(dir)))
    {
      if (strstr(entry->d_name, "usb"))
        {
          bool match = false;
          sprintf(fname, ACCES_USB_DEV_DIR"%s", entry->d_name);
          //for each name check if there's already an entry with
          //that name and a valid file descriptor
          for (int i = 0; i < AIOUSB_MAX_DEVICES; i++)
          {
            if ((aiousb_device_present(i))  && (!strcmp(fname, aiousb_devices[i]->dev_path)))
              {
                match = true;
              }
          }
          if (!match)
          {
            //if no entry exist open device
            int device_index = aiousb_get_available_device_index();
            if (device_index == AIOUSB_MAX_DEVICES)
            {
              aiousb_library_err_print("No available index. Ignoring device");
              continue; //Could break here, but if this ever happens I think we
                        //want to know how many devices it thinks are on the system
            }
            if (aiousb_device_open(fname, &aiousb_devices[device_index]))
              {
                continue;
              }
            aiousb_devices[device_index]->dev_path = (char *)malloc(strlen(fname) + 1);
            strcpy(aiousb_devices[device_index]->dev_path, fname);
          }
        }
    };
    pthread_mutex_unlock(&aiousb_devices_lock);
}

void check_removed ()
{
  aiousb_debug_print(">>>");
  pthread_mutex_lock(&aiousb_devices_lock);
  for (int i = 0 ; i < AIOUSB_MAX_DEVICES ; i++)
  {
    if (!aiousb_device_present(i)) continue;

    struct stat fstat;
    int fstat_status;
    fstat_status = ::fstat(aiousb_devices[i]->fd, &fstat);
    if (fstat_status)
      {
        aiousb_devices[i]->fd = -1;
        continue;
      }
    if (!fstat.st_nlink)
      {
        aiousb_devices[i]->fd = -1;
      }
  }
  pthread_mutex_unlock(&aiousb_devices_lock);
}
#if NO_HOTPLUG != 1
void hotplug_monitor (int n)
{
  struct udev* udev = udev_new();
  struct udev_monitor* mon = udev_monitor_new_from_netlink(udev, "udev");
  int fd;

  udev_monitor_filter_add_match_subsystem_devtype(mon, "usb", NULL);
  udev_monitor_enable_receiving(mon);

  fd = udev_monitor_get_fd(mon);

  fd_set fds;
  int status;

  FD_ZERO(&fds);
  FD_SET(fd, &fds);
  FD_SET(pipe_fds[0], &fds);

  while (!exiting)
    {

      status = select(FD_SETSIZE, &fds, NULL, NULL, NULL);
      if (status < 0)
        {
          aiousb_library_err_print("select returned %d", status);
          break;
        }
      if (FD_ISSET(fd, &fds))
        {
        struct udev_device *dev = udev_monitor_receive_device(mon);
        if (udev == nullptr) continue;

        const char* IdVendor = udev_device_get_sysattr_value(dev, "idVendor");
        const char* idProduct = udev_device_get_sysattr_value(dev, "idProduct");
        const char* Action = udev_device_get_action(dev);

        aiousb_debug_print("Action = %s\n", Action);

        if (!strcmp(Action, "remove"))
        {
          check_removed();
          continue;
        }

        if ((!IdVendor)  || (strcmp(IdVendor, "1605")))
          {
            continue;
          }

        uint16_t pid = strtol(idProduct, NULL, 16);

        for (unsigned int i = 0; i < NUM_ACCES_USB_DEVICES;i++)
          {
            if (pid == acces_usb_device_table[i].pid_loaded) scan_devices();
          }
        }
    }
}
#endif

int aiousb_device_open (const char *fname, aiousb_device_handle *device)
{
  int status;
  struct accesio_usb_aiousb_info accesio_usb_aiousb_info;
  struct aiousb_device *ptr;
  ptr = (struct aiousb_device *)malloc(sizeof(struct aiousb_device));
  memset(ptr, 0, sizeof(struct aiousb_device));
  uint8_t mem_flags[3];

  aiousb_debug_print("Enter");

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

  for (unsigned int i = 0 ; i < NUM_ACCES_USB_DEVICES ; i++)
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

  status = GenericVendorRead(ptr,
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

  status = GenericVendorRead(ptr,
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

  if ( 0 == ptr->descriptor.imm_adc_post_scale)
      ptr->descriptor.imm_adc_post_scale = 1.0;


  aiousb_debug_print("opened %s", fname);
  *device = ptr;

  return 0;
}

int AiousbInit()
{
  DIR *dir;
  struct dirent *entry;
  char fname[AIOUSB_MAX_PATH];

  aiousb_debug_print("Enter");

  if (aiousb_init_complete)
    {
#if NO_HOTPLUG != 1
      return -EALREADY;
#else
      check_removed();
      scan_devices();
      return 0;
#endif
    }

  pthread_mutex_init(&aiousb_devices_lock, NULL);

  dir = opendir(ACCES_USB_DEV_DIR);


  if (dir != nullptr)
  {
    pthread_mutex_lock(&aiousb_devices_lock);
    while ((entry = readdir(dir)))
      {
        if (strstr(entry->d_name, "usb"))
          {
            int device_index;
            sprintf(fname, ACCES_USB_DEV_DIR"%s", entry->d_name);
            device_index = aiousb_get_available_device_index();
            if (device_index == AIOUSB_MAX_DEVICES)
            {
              aiousb_library_err_print("No available index. Ignoring device");
              continue; //Could break here, but if this ever happens I think we
                        //want to know how many devices it thinks are on the system
            }
            if (aiousb_device_open(fname, &aiousb_devices[device_index]))
              {
                continue;
              }
            aiousb_devices[device_index]->dev_path = (char *)malloc(strlen(fname) + 1);
            strcpy(aiousb_devices[device_index]->dev_path, fname);
          }
      }
      pthread_mutex_unlock(&aiousb_devices_lock);
  }
    if(pipe(pipe_fds))
    {
      aiousb_library_err_print("Error creating pipe. Not starting hotplug monitor");
    }
    else
    {
      exiting = false;
      atexit(&lib_exit);
#if NO_HOTPLUG != 1
      hotplug_thread = std::thread(&hotplug_monitor, NULL);
#endif
    }
    aiousb_init_complete = true;
    return 0;
}

void aiousb_device_close(aiousb_device_handle device)
{
  aiousb_debug_print("Enter");
  if (device->last_dio_data != NULL)
    {
      free(device->last_dio_data);
      device->last_dio_data = NULL;
    }
  close(device->fd);
  free (device);
}

int DeviceHandleByPath (const char *fname, aiousb_device_handle *device)
{
  int i;
  int ret_val = -1;

  aiousb_debug_print("Enter");

  if (!aiousb_init_complete) return -ENAVAIL;

  pthread_mutex_lock(&aiousb_devices_lock);
  for ( i = 0 ; i < AIOUSB_MAX_DEVICES ; i++)
  {
    if (aiousb_device_present(i) && !strcmp(fname, aiousb_devices[i]->dev_path))
    {
      *device = aiousb_devices[i];
      ret_val = 0;
      break;
    }
  }
  pthread_mutex_unlock(&aiousb_devices_lock);
  return ret_val;
}

aiousb_device_handle aiousb_handle_by_index_private(unsigned long device_index)
{
  aiousb_debug_print("Enter");


  if (device_index == diOnly)
  {
    int device_count = 0;
    int device_index = 0;
    pthread_mutex_lock(&aiousb_devices_lock);
    for (int i = 0 ; i < AIOUSB_MAX_DEVICES ; i++)
    {
      if (aiousb_device_present(i))
      {
        device_count++;
        device_index = i;
      }
      if (device_count > 1) break;
    }
    pthread_mutex_unlock(&aiousb_devices_lock);
    return (1 == device_count) ? aiousb_devices[device_index] : NULL;
  }
  if (device_index == diFirst)
  {
    int device_index = 0;
    do
    {
      if (aiousb_device_present(device_index)) return aiousb_devices[device_index];
      device_index++;
    } while (device_index < AIOUSB_MAX_DEVICES);
    return NULL;
  }
  return aiousb_devices[device_index];
}

int DeviceHandleByIndex(unsigned long device_index, aiousb_device_handle *device)
{
  aiousb_debug_print("Enter");

  if (!aiousb_init_complete) return -ENAVAIL;


  *device = aiousb_handle_by_index_private(device_index);
  if (device == NULL)
    {
      return -EINVAL;
    }
  else
    {
      return 0;
    }
}

int DeviceIndexByPath (const char *fname, unsigned long *device_index)
{
  int i;
  int ret_val = -1;

  aiousb_debug_print("Enter");

  if (!aiousb_init_complete) return -ENAVAIL;

  pthread_mutex_lock(&aiousb_devices_lock);
  for ( i = 0 ; i <AIOUSB_MAX_DEVICES ; i++)
  {
    if (!strcmp(fname, aiousb_devices[i]->dev_path) && aiousb_devices[i]->fd != -1)
    {
      *device_index = i;
      ret_val = 0;
      break;
    }
  }
  pthread_mutex_unlock(&aiousb_devices_lock);
  return ret_val;
}

uint32_t GetDevices()
{
  uint32_t retval = 0;

  if (!aiousb_init_complete) return -ENAVAIL;
  pthread_mutex_lock(&aiousb_devices_lock);
  for (int i = 0 ; i < AIOUSB_MAX_DEVICES ; i++)
  {
    if (aiousb_device_present(i)) retval |= 1 << i;
  }
  pthread_mutex_unlock(&aiousb_devices_lock);

  return retval;
}

int QueryDeviceInfo(aiousb_device_handle device,
                              uint32_t *pid, uint32_t *name_size, char *name,
                              uint32_t *dio_bytes, uint32_t *counters)
{
  unsigned long device_index = 0;

  if(!(DeviceIndexByPath(device->dev_path, &device_index)) &&
    (aiousb_device_present(device_index)))
  {

      if (pid != nullptr) *pid = device->descriptor.pid_loaded;
      //TODO: Make the name copy match description in manual. ie set name_size to actual length
      if ((name_size != nullptr) && (name != nullptr)) strncpy(name, device->descriptor.name, *name_size);
      if (dio_bytes != nullptr) *dio_bytes = device->descriptor.dio_bytes;
      if (counters != nullptr) *counters = device->descriptor.counters;
      return 0;
  }
  else
  {
    return -ENODEV;
  }
}

int GetDeviceSerialNumber(aiousb_device_handle device, uint64_t *serial_number)
{
  return GenericVendorRead(device, 0xa2, 0x1df8, 0, sizeof(uint64_t), serial_number);
}


int GenericVendorRead(aiousb_device_handle device,
            uint8_t request, uint16_t value, uint16_t index,
            uint16_t size, void *data)
{
  struct  accesio_usb_control_transfer context = {0};
  int status;
  aiousb_debug_print("Enter");

  context.request = request;
  context.value = value;
  context.index = index;
  context.size = size;
  context.data = data;
  context.read = 1;

  status = ioctl (device->fd, ACCESIO_USB_CONTROL_XFER, &context);

  if (status)
    {
      aiousb_debug_print("ioctl returned %d", status);
    }
  return status;
}



int GenericVendorWrite(aiousb_device_handle device,
            uint8_t request, uint16_t value, uint16_t index,
            uint16_t size, void *data)
{
  struct accesio_usb_control_transfer context = {0};
  int status;

    aiousb_debug_print("Enter");

  context.request = request;
  context.value = value;
  context.index = index;
  context.size = size;
  context.data = data;
  context.read = 0;

  status = ioctl (device->fd, ACCESIO_USB_CONTROL_XFER, &context);

  if (status)
    {
      aiousb_debug_print("ioctl returned %d", status);
    }

  return status;
}

int AWU_GenericBulkIn (aiousb_device_handle device,
          unsigned int pipe_index, void *data, int size,
          int *transferred)
{
  struct accesio_usb_bulk_transfer context = {0};
  int status;
  aiousb_debug_print("Enter");


  context.pipe_index = pipe_index;
  context.data = data;
  context.size = size;
  context.transferred = transferred;
  context.read = 1;

  status = ioctl(device->fd, ACCESIO_USB_BULK_XFER, &context);

  aiousb_debug_print("status = %d", status);

  return status;
}

int AWU_GenericBulkOut (aiousb_device_handle device, unsigned int pipe_index,
           void *data, int size,	int *transferred)
{
  struct accesio_usb_bulk_transfer context = {0};
  int status;

  aiousb_debug_print("Enter");

  context.pipe_index = pipe_index;
  context.data = data;
  context.size = size;
  context.transferred = transferred;
  context.read = 0;

  status = ioctl(device->fd, ACCESIO_USB_BULK_XFER, &context);

  return status;
}

int CustomEEPROMWrite(aiousb_device_handle device,
            uint32_t start_address, uint32_t data_size, void *data)
{
  unsigned int block_size = 32;
  int status;
  uint8_t *write_ptr = (uint8_t *)data;
  if (start_address > 0xff)
    {
      return -EINVAL;
    }
  if (data_size > (0x100 - start_address))
    {
      return -EINVAL;
    }

  do
  {
    status = GenericVendorWrite(device,
                                      0xa2,
                                      0x1e00 + start_address,
                                      0,
                                      data_size > block_size ? block_size : data_size,
                                      write_ptr);
    if (status >= 0)
    {
      write_ptr += status;
      start_address += status;
      data_size -= status;
    }
    else
    {
      return status;
    }
  }while (data_size);

  return 0;
}

int CustomEEPROMRead(aiousb_device_handle device,
            uint32_t start_address, uint32_t data_size, void *data)
{
  if (start_address > 0xff)
    {
      return -EINVAL;
    }
  if (data_size > (0x100 - start_address))
    {
      return -EINVAL;
    }

  return GenericVendorRead(device, 0xa2, 0x1e00 + start_address, 0, data_size, data);

}

int DIO_Configure (aiousb_device_handle device, uint8_t b_tristate,
          void *out_mask, void *data)
{
  uint8_t *payload;
  uint16_t out_mask_size;
  uint16_t payload_size;
  int status;

  aiousb_debug_print("Enter");


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

  status = GenericVendorWrite (device,
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

int DIO_ConfigureEx (aiousb_device_handle device, void * out_mask,
                void *data, void *tristate_mask)
{
  uint8_t *payload;
  uint16_t out_mask_size;
  uint16_t tristate_size;
  uint16_t payload_size;
  int status;

  aiousb_debug_print("Enter");

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

  status = GenericVendorWrite(device,
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

int DIO_ConfigureMasked(aiousb_device_handle device, void *outs,
                void *outs_mask, void *data, void *data_mask, void *tristates,
                void *tristates_mask)
{
  uint8_t *payload, *current;
  uint16_t payload_size;
  int status;

  aiousb_debug_print("Enter");

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

  status = GenericVendorWrite(device,
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


int DIO_WriteAll(aiousb_device_handle device, void *data)
{
  int status;

  aiousb_debug_print("Enter");

  if (device->descriptor.dio_bytes == 0)
    {
      return -EBADRQC;
    }

  memcpy(device->last_dio_data, data, device->descriptor.dio_bytes);

  status = GenericVendorWrite(device,
                                  AUR_DIO_WRITE,
                                  0,
                                  0,
                                  device->descriptor.dio_bytes,
                                  data);

  if (status != (int)device->descriptor.dio_bytes)
    {
      aiousb_library_err_print("GenericVendorWrite returned: %d dio_bytes: %d",
                              status,
                              device->descriptor.dio_bytes);
    }
  else
    {
      status = 0;
    }

  return status;
}

int DIO_Write8(aiousb_device_handle device, uint32_t byte_index,
                uint8_t data)
{
  int status;

  aiousb_debug_print("Enter");

  if (device->descriptor.dio_bytes == 0)
    {
      return -EBADRQC;
    }

  if (byte_index >= device->descriptor.dio_bytes)
    {
      return -EINVAL;
    }

  device->last_dio_data[byte_index] = data;

  status = GenericVendorWrite(device,
                                      AUR_DIO_WRITE,
                                      0,
                                      0,
                                      device->descriptor.dio_bytes,
                                      device->last_dio_data);

  if (status != (int)device->descriptor.dio_bytes)
    {
      aiousb_library_err_print("GenericVendorWrite return %d", status);
    }
  else
    {
      status = 0;
    }
  return status;
}


int DIO_Write1(aiousb_device_handle device, uint32_t bit_index,
                uint8_t data)
{
  uint32_t byte_index;
  int status;

  aiousb_debug_print("Enter");

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
        status = GenericVendorWrite(device,
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
      status = GenericVendorWrite(device,
                                            AUR_DIO_WRITE,
                                            0,
                                            0,
                                            device->descriptor.dio_bytes,
                                            device->last_dio_data);
      if (status != (int)device->descriptor.dio_bytes)
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

int DIO_ReadAll(aiousb_device_handle device, void *data)
{
  int status;

  aiousb_debug_print("Enter");

  if (device->descriptor.dio_bytes == 0)
    {
      return -EBADRQC;
    }

  status = GenericVendorRead(device,
                                      AUR_DIO_READ,
                                      0,
                                      0,
                                      device->descriptor.dio_bytes,
                                      data);

  if (status != (int)device->descriptor.dio_bytes)
    {
      aiousb_library_err_print("GenericVendorRead returned %d", status);
    }
  else
    {
      status = 0;
    }

  return status;
}

int DIO_Read8(aiousb_device_handle device, uint32_t byte_index,
                uint8_t *data)
{
  int status;
  uint8_t *all_bytes;

  aiousb_debug_print("Enter");

  if (device->descriptor.dio_bytes == 0)
    {
      return -EBADRQC;
    }

  if (byte_index >= device->descriptor.dio_bytes)
    {
      return -EINVAL;
    }

  all_bytes = (uint8_t *) malloc(device->descriptor.dio_bytes);

  status = GenericVendorRead(device,
                                      AUR_DIO_READ,
                                      0,
                                      0,
                                      device->descriptor.dio_bytes,
                                      all_bytes);

  if (status != (int)device->descriptor.dio_bytes)
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

int DIO_Read1(aiousb_device_handle device, uint32_t bit_index,
                uint8_t *data)
{
  int status;
  uint8_t *all_bytes;

  aiousb_debug_print("Enter");

  if (device->descriptor.dio_bytes == 0)
    {
      return -EBADRQC;
    }

  if (bit_index >= device->descriptor.dio_bytes * 8)
    {
      return -EINVAL;
    }

  all_bytes = (uint8_t *) malloc(device->descriptor.dio_bytes);

  status = GenericVendorRead(device,
                                      AUR_DIO_READ,
                                      0,
                                      0,
                                      device->descriptor.dio_bytes,
                                      all_bytes);

  if (status != (int)device->descriptor.dio_bytes)
    {
      aiousb_library_err_print("GenericVendorRead returned %d", status);
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
int DIO_ConfigurationQuery(aiousb_device_handle device, void *out_mask,
                void *tristate_mask)
{
  int status;
  uint8_t *data;
  uint16_t out_length, tristate_length, length;

  aiousb_debug_print("Enter");

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

  status = GenericVendorRead(device,
                                      AUR_DIO_CONFIG_QUERY,
                                      0,
                                      device->descriptor.dio_bytes,
                                      length,
                                      data);

  if (status != length)
   {
     aiousb_library_err_print("GenericVendorRead returned %d", status);
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
int DIO_StreamOpen(aiousb_device_handle device, uint32_t is_read)
{
  int status;

  aiousb_debug_print("Enter");

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
      status = GenericVendorWrite(device, 0xbc, 0, 0, 0, NULL);
    }
  else
    {
      status = GenericVendorWrite(device, 0xbb, 0, 0, 0, NULL);
    }

  if (!status)
    {
      device->b_dio_open = 1;
    }
  else
    {
     aiousb_library_err_print("GenericVendorRead returned %d", status);
    }

  return status;
}

//TODO: Untested
int DIO_StreamClose(aiousb_device_handle device)
{
  aiousb_debug_print("Enter");

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
int DIO_StreamSetClocks(aiousb_device_handle device, double *read_hz,
                double *write_hz)
{
  int status;
  struct dio_clock_data clock_data = {0};

  aiousb_debug_print("Enter");

  if (!device->descriptor.b_dio_stream)
    {
      return -EBADRQC;
    }

  clock_data.disables = 0;
  clock_data.disables |= 0x1;
  clock_data.disables |= 0x2;
  clock_data.write_oct_dac = oct_dac_from_freq(write_hz);
  clock_data.read_oct_dac = oct_dac_from_freq(read_hz);

  status = GenericVendorRead(device,
                                  AUR_DIO_SETCLOCKS,
                                  0,
                                  0,
                                  sizeof(struct dio_clock_data),
                                  &clock_data);

  if (status != sizeof(struct dio_clock_data))
    {
     aiousb_library_err_print("GenericVendorRead returned %d", status);
    }
  return status;

}

//TODO: Untested. This needs to be tested before exposing to customers.
//TODO: figure out the right data type for frame_data
int DIO_StreamFrame (aiousb_device_handle device, unsigned long frame_points,
                unsigned short *frame_data, size_t *bytes_transferred)
{
  int (*fptr)(aiousb_device_handle, unsigned int, void *, int, int*);
  unsigned int pipe_index;
  int status;
  int this_transfer;

  aiousb_debug_print("Enter");

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
      fptr = &AWU_GenericBulkIn;
    }
  else
    {
      pipe_index = 0x02;
      fptr = &AWU_GenericBulkOut;
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
  aiousb_debug_print("Enter");

  if (!(*block_index))
    {
      *block_index = *counter_index / 3;
      *counter_index = *counter_index % 3;
    }

  if ((device->descriptor.counters < (int)*block_index) || (*counter_index > 3))
    {
      return -EINVAL;
    }

    return 0;
}

//TODO: Untested. This needs to be tested before exposing to customers.
int CTR_8254Mode(aiousb_device_handle device, uint32_t block_index,
                uint32_t counter_index, uint32_t mode)
{
  int status;
  aiousb_debug_print("Enter");

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

  status = GenericVendorWrite(device,
                          AUR_CTR_MODE,
                          block_index | (mode << 8),
                          0,
                          0,
                          NULL);

  return status;
}

//TODO: Untested. This needs to be tested before exposing to customers.
int CTR_8254Load(aiousb_device_handle device, uint32_t block_index,
                uint32_t counter_index, uint16_t load_value)
{
  int status;
  uint8_t mode;

  aiousb_debug_print("Enter");

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

  status = GenericVendorWrite(device,
                                    AUR_CTR_LOAD,
                                    block_index | (mode << 8),
                                    load_value,
                                    0,
                                    NULL);

  return status;
}

//TODO: Untested. This needs to be tested before exposing to customers.
int CTR_8254ModeLoad(aiousb_device_handle device, uint32_t block_index,
                uint32_t counter_index, uint32_t mode, uint16_t load_value)
{
  int status;

  aiousb_debug_print("Enter");

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

  status = GenericVendorWrite(device,
                                      AUR_CTR_MODELOAD,
                                      block_index | (mode << 8),
                                      load_value,
                                      0,
                                      NULL);

  return status;
}

//TODO: Untested. This needs to be tested before exposing to customers.
int CTR_8254StartOutputFreq(aiousb_device_handle device,
                uint32_t block_index, double *frequency)
{
  int status;
  uint32_t temp, divisor_a, divisor_b;
  double err, min_err;
  long double divisor_ab;

  aiousb_debug_print("Enter");

  if (device->descriptor.counters == 0)
    {
      return -EBADRQC;
    }

  if ((device->descriptor.counters <= (int)block_index) || (frequency == NULL))
    {
      return -EINVAL;
    }

  if (*frequency <= 0)
    {
      CTR_8254Mode(device, block_index, 1, 2);
      status = CTR_8254Mode(device, block_index, 2, 3);
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

        CTR_8254ModeLoad(device, block_index, 1, 2, divisor_a);
        status = CTR_8254ModeLoad(device, block_index, 2, 3, divisor_b);
    }
    return status;
}

//TODO: Untested. This needs to be tested before exposing to customers.
int CTR_8254Read(aiousb_device_handle device, uint32_t block_index,
                uint32_t counter_index, uint16_t *read_value)
{
  int status;

  aiousb_debug_print("Enter");

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

  status = GenericVendorRead(device,
                                    AUR_CTR_READ,
                                    block_index | (counter_index << 8),
                                    0,
                                    sizeof(read_value),
                                    read_value);

  if (status != sizeof(read_value))
    {
      aiousb_library_err_print("GenericVendorRead returned %d", status);
    }
  else
    {
      status = 0;
    }

  return status;
}

//TODO: Untested. This needs to be tested before exposing to customers.
int CTR_8254ReadAll(aiousb_device_handle device, uint16_t *data)
{
  int status;
  uint16_t length;

  aiousb_debug_print("Enter");

  if (device->descriptor.counters == 0)
    {
      return -EBADRQC;
    }

  length = device->descriptor.counters * 2 * 3;

  status = GenericVendorRead(device,
                                    AUR_CTR_READALL,
                                    0,
                                    0,
                                    length,
                                    data);

  if (status != length)
    {
      aiousb_library_err_print("GenericVendorRead returned %d", status);
    }
  else
    {
      status = 0;
    }

  return status;
}

int CTR_8254ReadStatus(aiousb_device_handle device, uint32_t block_index,
                uint32_t counter_index, uint8_t *read_value, uint8_t *status)
{
  int aiousb_status;
  uint8_t payload[3] = {0};

  aiousb_debug_print("Enter");

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

  aiousb_status = GenericVendorRead(device,
                                    AUR_CTR_READ,
                                    block_index | (counter_index << 8),
                                    0,
                                    sizeof(payload),
                                    &payload);

  if (aiousb_status != sizeof(payload))
    {
      aiousb_library_err_print("GenericVendorRead returned %d", aiousb_status);
    }
  else
    {
      aiousb_status = 0;
      *read_value = payload[0];
      *status = payload[2];
    }

  return aiousb_status;
}

int CTR_8254ReadModeLoad(aiousb_device_handle device, uint32_t block_index,
                uint32_t counter_index, uint32_t mode, uint16_t load_value,
                uint16_t *read_value)
{
  int status;

  aiousb_debug_print("Enter");

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

  status = GenericVendorRead(device,
                                AUR_CTR_MODELOAD,
                                block_index | (counter_index << 8),
                                load_value,
                                sizeof(*read_value),
                                read_value);

  if (status != sizeof(*read_value))
    {
      aiousb_library_err_print("GenericVendorRead returned %d", status);
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

  aiousb_debug_print("Enter, device->descriptor.b_adc_bulk = %d", device->descriptor.b_adc_bulk);

  if (!device->descriptor.b_adc_bulk)
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

    return GenericVendorWrite(device,
                            AUR_ADC_IMMEDIATE,
                            0,
                            channel,
                            0,
                            NULL);
}



static void *timeout_worker (void *context)
{
  struct time_out_context *time_out_context = (struct time_out_context *)context;
  struct timespec now, end_time;
  int status;

  aiousb_debug_print("Enter");

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
      GenericVendorWrite(time_out_context->device,
                              AUR_ADC_IMMEDIATE,
                              0,
                              0,
                              0,
                              NULL);
    }
    return NULL;
}

//TODO: Needs to be tested
int aiousb_get_scan_inner_adc_bulk(aiousb_device_handle device, uint8_t *config_buff,
                  uint32_t *config_size, uint16_t **ad_buff,
                  uint32_t *ad_buff_length, uint8_t *start_channel,
                  uint8_t *end_channel, uint32_t time_out_ms)
{
  int status;
  pthread_t timeout_thread = 0;
  struct time_out_context time_out_context = {0};
  uint8_t channel_count;
  int bytes_remaining;
  uint32_t bc_data;
  int transferred = 0;
  uint8_t *byte_ptr;

  aiousb_debug_print("Enter");

  {
    //TODO: DEBUG CODE THAT SHOULD BE REMOVED
    char dbg_string[1024] = {0};
    for (unsigned int i = 0; i < *config_size ; i++)
    {
      sprintf(dbg_string + strlen(dbg_string), "0x%x = 0x%x\n", i, config_buff[i]);
    }
    aiousb_debug_print("%s", dbg_string);
  }


  status = ADC_GetConfig(device, config_buff, config_size);

    {
      //TODO: DEBUG CODE THAT SHOULD BE REMOVED
    char dbg_string[1024] = {0};
    for (unsigned int i = 0; i < *config_size ; i++)
    {
      sprintf(dbg_string + strlen(dbg_string), "0x%x = 0x%x\n", i, config_buff[i]);
    }
    aiousb_debug_print("%s", dbg_string);
  }

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
      config_buff[0x11] = 0x4 | (config_buff[0x11] & ~0x3);
    }

  *start_channel = config_buff[0x12] & 0xf;
  *end_channel = config_buff[0x12] >> 4;

  if (*config_size >= 21)
    {
      *start_channel |= (config_buff[20] & 0xf) << 4;
      *end_channel |= config_buff[20] & 0xf0;
    }

  channel_count = *end_channel - *start_channel + 1;
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

  status = ADC_SetConfig(device, config_buff, config_size);

  bc_data = 0x5;

  aiousb_debug_print("Sending bytes_remaining = %d", bytes_remaining);

  status = GenericVendorWrite(device,
                                0xbc,
                                0,
                                *ad_buff_length,
                                sizeof(bc_data),
                                &bc_data);

  if (status != sizeof(bc_data))
  {
    goto ERR_OUT;
  }

  aiousb_debug_print("time_out_ms = %d", time_out_ms);

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
        status = AWU_GenericBulkIn(device,
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

    return status;
}

//Not in header file because it is deprecated, but needed by
//adc_get_scan_inner_adc_dio_stream
int aiousb_adc_bulk_acquire(aiousb_device_handle device, uint32_t *buff_size,
                      uint16_t *buff)
{

  aiousb_debug_print("Enter");

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


  return 0;
}

static const int ADC_DIO_OVERSAMPLE = 0x40;

int aiousb_get_scan_inner_adc_dio_stream(aiousb_device_handle device, uint8_t *config_buff,
                  uint32_t *config_size, uint16_t **ad_buff,
                  uint32_t *ad_buff_length, uint8_t *start_channel,
                  uint8_t *end_channel, uint32_t time_out_ms)
{
  struct adc_intermmediate_buff *inter_buff = NULL;
  uint32_t inter_buff_length = ADC_DIO_OVERSAMPLE;
  int status = 0;
  struct timespec now, end_time;
  int i;

  aiousb_debug_print("Enter");

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
  *start_channel = 0;
  *end_channel = 1;
  memset(config_buff, 0, *config_size);

  ADC_GetConfig(device, config_buff, config_size);
  config_buff[0x13] = ADC_DIO_OVERSAMPLE - 1;

  *ad_buff = (uint16_t *)malloc(2*ADC_DIO_OVERSAMPLE);

  for (i = 0 ; i < ADC_DIO_OVERSAMPLE ; i++)
    {
      *ad_buff[i] = inter_buff[i].ad0;
      *ad_buff[ADC_DIO_OVERSAMPLE + i] = inter_buff[i].ad1;
    }
    config_buff[0x13] = 0;
  return 0;
}

int aiousb_get_scan_inner_imm_adcs (aiousb_device_handle device,
                  uint8_t *config_buff, uint32_t *config_size,
                  uint16_t **ad_buff, uint32_t *ad_buff_length,
                  uint8_t *start_channel, uint8_t *end_channel,
                  uint32_t time_out_ms)
{
  int status;
  *ad_buff =(uint16_t *) malloc(device->descriptor.imm_dacs);

  aiousb_debug_print("Enter");

  status = GenericVendorRead(device,
                                    AUR_ADC_IMMEDIATE,
                                    0,
                                    0,
                                    device->descriptor.imm_adcs * 2,
                                    *ad_buff);

  if (status < 0)
  {
    aiousb_library_err_print("Error reading from device");
    return status;
  }

  status = 0;

  *start_channel = 0;
  *end_channel = device->descriptor.imm_adcs - 1;

  memset(config_buff, 0x02, 16);
  config_buff[0x13] = 0;

  return status;

}

//TODO: Make start_channel and end_channel pointers
//TODO: Figure out where to join threads in the functions called here
int aiousb_get_scan_inner(aiousb_device_handle device, uint8_t *config_buff,
                  uint32_t *config_size, uint16_t **ad_buff,
                  uint32_t *ad_buff_length, uint8_t *start_channel,
                  uint8_t *end_channel, uint32_t time_out_ms)
{

  aiousb_debug_print("Enter");

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


double volts_from_counts (aiousb_device_handle device, int counts, int range_code)
{
  double v = counts * (1.0 /65536.0);

  if (range_code & 0x1)
    {
      v = v * 2 - 1;
    }
  if ((range_code & 0x2) == 0)
    {
      v = v * 2;
    }
  if ((range_code & 0x4) == 0)
    {
      v = v * 5;
    }
  if (device->descriptor.imm_adc_post_scale != 1)
    {
      v = v * (double)device->descriptor.imm_adc_post_scale;
    }
  return v;
}

int ADC_GetScanV(aiousb_device_handle device, double *data)
{
  int status = 0;
  uint8_t config_buff[MAX_CONFIG_SIZE] = {0};
  uint32_t config_size;
  uint8_t start_channel, end_channel;
  uint16_t *ad_buff = NULL;
  uint32_t ad_buff_length = 0;
  int i, j, channel, total, data_index, range_code;

  aiousb_debug_print("Enter");

  config_size = device->descriptor.config_bytes;

  status = aiousb_get_scan_inner(device,
                                config_buff,
                                &config_size,
                                &ad_buff,
                                &ad_buff_length,
                                &start_channel,
                                &end_channel,
                                0);

  if (status)
    {
      aiousb_library_err_print("aiousb_get_scan_inner returned %d", status);
      goto ERR_OUT;
    }

  data_index = 0;
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
          range_code = config_buff[channel >> device->descriptor.range_shift];
          data[data_index] = volts_from_counts(device,
                                    total / config_buff[0x13],
                                    range_code);
          data_index++;
          i++;
        }
    }
  else
    {
      for (channel = start_channel ; channel <= end_channel ; channel++)
        {
          range_code = config_buff[channel]  >> device->descriptor.range_shift;
          data[channel] = volts_from_counts(device,
                                      ad_buff[channel - start_channel],
                                      range_code);
        }
    }

ERR_OUT:

  if (ad_buff)
    {
      free(ad_buff);
    }
  return status;

}

int ADC_GetChannelV (aiousb_device_handle device, uint32_t channel_index,
                double *volts)
{
  int status;

  double data[MAX_CHANNELS_FOR_SCAN];

  aiousb_debug_print("Enter");

  status = ADC_GetScanV(device, data);

  if (status)
  {
    aiousb_library_err_print("Error getting scan");
    return status;
  }

  *volts = data[channel_index];

  return status;

}

int ADC_GetTrigScanV (aiousb_device_handle device, double *data,
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

  aiousb_debug_print("Enter");

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
                          &start_channel,
                          &end_channel,
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
                    void *context, adc_cont_callback callback, double *hertz)
{

  aiousb_debug_print("Enter");


  //TODO: Verify board capabilities and buffer size
  //TODO: Make sure threads are in an acceptable state

  device->ContAdc = new ContinuousAdcWorker(device,
                        buff_size,
                        base_buff_count,
                        context,callback);

  device->ContAdc->Execute();
  return 0;


//   //initialize buf worker
//   device->adc_cont_buff_worker_context.adc_cont_acq_worker_context = &device->adc_cont_acq_worker_context;
//   device->adc_cont_buff_worker_context.bytes_per_buff = buff_size;
//   device->adc_cont_buff_worker_context.callback = callback;
//   device->adc_cont_buff_worker_context.callback_context = context;
//   pthread_mutex_init(&device->adc_cont_buff_worker_context.buff_mutex, NULL);
//   sem_init(&device->adc_cont_buff_worker_context.kill_sem, 0, 0);
//   sem_init(&device->adc_cont_buff_worker_context.blank_buf_sem, 0, base_buff_count);
//   sem_init(&device->adc_cont_buff_worker_context.data_buf_sem, 0, 0);
//   device->adc_cont_buff_worker_context.buf_buf =
//     (adc_continuous_buffer_handle *) malloc(sizeof(adc_continuous_buffer) * base_buff_count);



//   //initialize cont_acq_worker
//   device->adc_cont_acq_worker_context.adc_cont_buff_worker_context = &device->adc_cont_buff_worker_context;
//   pthread_cond_init(&device->adc_cont_acq_worker_context.start_cond, NULL);
//   pthread_mutex_init(&device->adc_cont_acq_worker_context.start_cond_mutex, NULL);
//   device->adc_cont_acq_worker_context.device = device;
//   device->adc_cont_acq_worker_context.pipe_index = 0;
//   if (device->descriptor.b_adc_dio_stream)
//     {
//       device->adc_cont_acq_worker_context.bcs_style = bcs_dio;
//       device->adc_cont_acq_worker_context.b_counter_control = 0;
//       if (hertz == NULL)
//         {
//           hertz = &device->descriptor.root_clock;
//         }
//     }
//   else
//     {
//       device->adc_cont_acq_worker_context.bcs_style = bcs_adc;
//       if (hertz == NULL)
//         {
//           device->adc_cont_acq_worker_context.b_counter_control = 0;
//         }
//       else
//         {
//           device->adc_cont_acq_worker_context.b_counter_control = 1;
// //TODO: CalcHzDivisors
//         }
//     }

//     pthread_create(&device->adc_cont_acq_worker, NULL, adc_cont_buff_worker_execute, &device->adc_cont_acq_worker_context);
//     pthread_create(&device->adc_cont_buff_worker, NULL, adc_cont_buff_worker_execute, &device->adc_cont_buff_worker_context);

//     pthread_join(device->adc_cont_acq_worker, NULL);
//     pthread_join(device->adc_cont_buff_worker, NULL);

}

int ADC_BulkContinuousStart (aiousb_device_handle device,
                uint32_t buff_size, uint32_t base_buff_count, void *context,
                adc_cont_callback callback)
{
  aiousb_debug_print("Enter");

  return aiousb_adc_bulk_continuous_start_inner(device,
                                            buff_size,
                                            base_buff_count,
                                            context,
                                            callback,
                                            NULL);
}

int ADC_BulkContinuousEnd (aiousb_device_handle device)
{
  device->ContAdc->Terminate();
  delete device->ContAdc;
  device->ContAdc = NULL;
  return 0;
}

int ADC_SetScanLimits (aiousb_device_handle device, uint32_t start_channel,
                uint32_t end_channel)
{
  uint8_t config_buff[MAX_CONFIG_SIZE] = {0};
  int status;

  aiousb_debug_print("Enter");

  if (!(device->descriptor.b_adc_bulk))
    {
      return -EBADRQC;
    }

  if ((start_channel > end_channel) ||
      (end_channel > device->descriptor.adc_mux_channels))
    {
      return -EINVAL;
    }

  status = GenericVendorRead(device,
                                  AUR_ADC_GET_CONFIG,
                                  0,
                                  0,
                                  device->descriptor.config_bytes,
                                  config_buff);


  if (status != (int)device->descriptor.config_bytes)
    {
      aiousb_library_err_print("GenericVendorRead returned %d", status);
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

  status = GenericVendorWrite(device,
                                    AUR_ADC_SET_CONFIG,
                                    0,
                                    0,
                                    device->descriptor.config_bytes,
                                    config_buff);

  if (status != (int)device->descriptor.config_bytes)
    {
      aiousb_library_err_print("GenericVendorWrite returned %d", status);
    }
  else
    {
      status = 0;
    }

  return status;
}

int ADC_GetConfig(aiousb_device_handle device, uint8_t *config_buff,
                uint32_t *config_size)
{
  int status;

  aiousb_debug_print("Enter");

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

  status = GenericVendorRead(device,
                                    AUR_ADC_GET_CONFIG,
                                    0,
                                    0,
                                    device->descriptor.config_bytes,
                                    config_buff);

  if (status != (int)device->descriptor.config_bytes)
    {
      aiousb_library_err_print("GenericVendorRead returned %d", status);
    }
  else
    {
      status = 0;
    }

  return status;
}

int ADC_SetConfig(aiousb_device_handle device, uint8_t *config_buff,
              uint32_t *config_size)
{
  int status;

  aiousb_debug_print("Enter");

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

  status = GenericVendorWrite(device,
                                  AUR_ADC_SET_CONFIG,
                                  0,
                                  0,
                                  device->descriptor.config_bytes,
                                  config_buff);

  if (status != (int)device->descriptor.config_bytes)
    {
      aiousb_library_err_print("GenericVendorRead returned %d", status);
    }
  else
    {
      status = 0;
    }

  return status;
}

int ADC_InitFastScanV(aiousb_device_handle device)
{
  int status;

  if (!device->descriptor.b_adc_bulk)
  {
    return -EBADRQC;
  }

  if (device->descriptor.config_bytes < 20)
  {
    return -EBADRQC;
  }

  if (device->config_buff_bak == nullptr)
  {
    device->config_buff_bak = new uint8_t[device->descriptor.config_bytes];
  }

  if (device->config_fast == nullptr)
  {
    device->config_fast = new uint8_t[device->descriptor.config_bytes];
  }

  ADC_GetConfig(device,
                    device->config_buff_bak,
                    &device->config_size);


  memcpy(device->config_fast, device->config_buff_bak, 0x10);
  device->config_fast[0x11] = 0x4;
  device->config_fast[0x13] = device->config_buff_bak[0x13] ? device->config_buff_bak[0x13] : 1;

  device->config_fast[0x12] = device->config_buff_bak[0x12];
  if (device->descriptor.config_bytes > 0x14)
    {
      device->config_fast[0x14] = device->config_buff_bak[0x14];
    }

  status = ADC_SetConfig(device,
                          device->config_fast,
                          &device->config_size);

  if (status)
    {
      aiousb_library_err_print("Error setting config");
      //does it make sense to try and restore?
      ADC_SetConfig(device,
                    device->config_buff_bak,
                    &device->config_size);
      delete[] device->config_buff_bak;
      delete[] device->config_fast;
    }

  return status;

}

int ADC_GetFastScanV(aiousb_device_handle device, double *data)
{
  int start_channel, end_channel, num_channels;
  uint16_t *ad_buff = nullptr;
  uint32_t ad_buff_length = 0;
  int status;
  uint32_t bc_data;

  if (!device->descriptor.b_adc_bulk)
    {
      return -EBADRQC;
    }

  if (device->descriptor.config_bytes < 20)
    {
      return -EBADRQC;
    }

  start_channel = device->config_fast[0x12] & 0xf;
  end_channel = device->config_fast[0x12] >> 4;
  if (device->descriptor.config_bytes >= 21)
    {
      start_channel |= (device->config_fast[20] & 0xf) << 4;
      end_channel |= device->config_fast[20] & 0xf0;
    }
  num_channels = end_channel - start_channel + 1;
  ad_buff_length = num_channels * (1 + device->config_fast[0x13]);

  ad_buff = new uint16_t[ad_buff_length];
  bc_data = 0x5;

  //TODO: Think I need a switch statement for adc_bulk, dio_stream, imm_adcs here
  //adc_bulk
  {
    status = GenericVendorWrite(device,
                                    0xbc,
                                    0,
                                    ad_buff_length,
                                    sizeof(bc_data),
                                    &bc_data);

    if (status != sizeof(bc_data))
      {
        aiousb_library_err_print("Error setting bc_data");
        goto err_out;
      }

    status = GenericVendorWrite(device, AUR_ADC_IMMEDIATE, 0, 0, 0, NULL);

    if (status)
      {
        aiousb_library_err_print("Error sending AUR_ADC_IMMEDIATE");
        goto err_out;
      }

    {
    uint8_t *byte_ptr = (uint8_t *)ad_buff;
    int transferred = 0;
    int bytes_remaining = ad_buff_length * sizeof(uint16_t);
    do
      {
        status = AWU_GenericBulkIn(device, 0, byte_ptr, bytes_remaining, &transferred);

        if (status)
          {
            aiousb_library_err_print("error during bulk in");
            goto err_out;
          }

        bytes_remaining -= transferred;
        byte_ptr += transferred;
        transferred = 0;
      }while (bytes_remaining > 0);
    }
  } //end adc_bulk


  {
    int total, range_code;
    int i = 0;
    int j;
    int channel;
    int data_index = 0;

    for (channel = start_channel ; channel <= end_channel ; channel++)
      {
        total = 0;
        for (j = 1 ; j <= device->config_fast[0x13] ; j++)
          {
            total += ad_buff[i * (1 + device->config_fast[0x13]) + j];
          }
          range_code = device->config_fast[channel >> device->descriptor.range_shift];
          data[data_index] = volts_from_counts(device,
                            total / device->config_fast[0x13],
                            range_code);
          i++;
          data_index++;
      }
  }

err_out:
  if (ad_buff != nullptr)
    {
      delete[] ad_buff;
    }
  return status;

}

int ADC_ResetFastScanV(aiousb_device_handle device)
{
  int status;

  if (!device->descriptor.b_adc_bulk)
    {
      return -EBADRQC;
    }

  if (device->descriptor.config_bytes < 20)
    {
      return -EBADRQC;
    }

  if (device->config_fast == nullptr)
    {
      return -EINVAL;
    }

  status = ADC_SetConfig(device, device->config_buff_bak, &device->config_size);

  if (status)
    {
      aiousb_library_err_print("Error setting config status");
    }

  delete[] device->config_buff_bak;
  delete[] device->config_fast;
  device->config_size = 0;
  return status;
}

int ADC_RangeAll(aiousb_device_handle device, uint8_t *gain_codes,
                uint32_t *b_differential)
{
  int status;

  aiousb_debug_print("Enter");

  if ((device->descriptor.adc_channels == 0)  ||
      (!(device->descriptor.b_adc_bulk || device->descriptor.b_adc_dio_stream)))
    {
      return -EBADRQC;
    }

  if (gain_codes == NULL)
    {
      return -EINVAL;
    }

  for (unsigned int i = 0 ; i < device->descriptor.adc_channels ; i++)
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

      status = ADC_GetConfig(device, config_buff, &config_size);

      if (status)
      {
        aiousb_library_err_print("ADC_GetConfig returned %d", status);
      }

      for (unsigned int i = 0 ; i < device->descriptor.adc_channels ; i++)
        {
          config_buff[i] = gain_codes[i] | (b_differential ? 1 : 0) << 3;
        }

      status = ADC_SetConfig(device, config_buff, &config_size);
      if (status)
        {
          aiousb_library_err_print("ADC_SetConfig returned %d", status);
        }

      free(config_buff);
    }
  else
    {
      status = GenericVendorWrite(device,
                                        AUR_ADC_SET_CONFIG,
                                        0,
                                        0,
                                        device->descriptor.config_bytes,
                                        gain_codes);

      if (status != (int)device->descriptor.config_bytes)
        {
          aiousb_library_err_print("GenericVendorWrite returned %d", status);
        }
      else
        {
          status = 0;
        }
    }

    return status;
}


int ADC_SetOversample(aiousb_device_handle device, uint8_t oversample)
{
  uint8_t *config_buff = NULL;
  uint32_t config_size = 0;
  int status;

  aiousb_debug_print("Enter");


  if (!(device->descriptor.b_adc_bulk))
    {
      return -EBADRQC;
    }

  config_size = device->descriptor.config_bytes;
  config_buff =(uint8_t *) malloc(config_size);

  status = ADC_GetConfig(device, config_buff, &config_size);

  if (status)
    {
      aiousb_library_err_print("ADC_GetConfig returned %d", status);
    }


  config_buff[0x13] = oversample;

  status = ADC_SetConfig(device, config_buff, &config_size);

  if (status)
    {
      aiousb_library_err_print("ADC_SetConfig returned %d", status);
    }

  free(config_buff);

  return status;
}




int ADC_Range1(aiousb_device_handle device, uint32_t adc_channel,
                uint8_t gain_code, uint32_t b_differential)
{
  uint8_t *config_buff = NULL;
  uint32_t config_size = 0;
  int status;

  aiousb_debug_print("Enter");

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

  status = ADC_GetConfig(device, config_buff, &config_size);

  if (status)
    {
      aiousb_library_err_print("ADC_GetConfig returned %d", status);
    }

  config_buff[adc_channel % device->descriptor.adc_channels] =
                      gain_code | (b_differential ? 1 : 0) << 3;

  status = ADC_SetConfig(device, config_buff, &config_size);

  if (status)
    {
      aiousb_library_err_print("ADC_SetConfig returned %d", status);
    }

  free(config_buff);

  return status;
}

//If doing anything with code for SetCal consult CoreExports.pas
//as this is based off of ADC_SetCalAndSave()
class SetCalWorker
{
  public:
    //CalFileName and OutFileName need to be valid for the life of the object.
    SetCalWorker(aiousb_device_handle Device, const char *CalFileName, const char *OutFileName)
    {
      mDevice = Device;
      mCalFileName = CalFileName;
      mShouldAppend = false;
      mOutFileName = OutFileName;

      uint32_t PacketWords;
      if (ioctl(mDevice->fd, ACCESIO_USB_GET_PORT_SPEED) <= 2) //USB_SPEED_FULL
      {
        PacketWords = 0x40 / 2;
      }
      else
      {
        PacketWords = 0x200 / 2;
      }
      mPacketsMask = 0xFFFFFFFF * PacketWords;
      mChunkSize = PacketWords * 4;

    }

    int SetCal();


  private:
    aiousb_device_handle mDevice;
    const char *mCalFileName;
    const char *mOutFileName;
    std::array<uint16_t, 0x10000> mCalTable;
    bool mShouldAppend;
    int mChunkSize;
    uint32_t mPacketsMask;

    double  LoRefRef = 0 * 6553.6;
    double  HiRefRef = 9.9339 * 6553.6;

    int LoadCalTable();
    int WriteCalToFile();
    uint16_t GetHiRef();
    double ConfigureAndBulkAcquire(std::vector<uint8_t> &Config);


};
// Delphi code for determining ChunkSize.
// For now we'll just assume the lower speed until I figure out how to get the
// host controller speed in Linux
//       //Determine packetization and buffering for DoLoadCalTable.
//       if CheckUSBSpeed(DeviceIndex) = usHigh then
//         PacketWords := $200 div 2
//       else
//         PacketWords := $40 div 2
//       ;
//       PacketsMask := $FFFFFFFF * PacketWords;
//       ChunkSize := PacketWords * 4;
int SetCalWorker::LoadCalTable()
{

  bool FirstChunkDouble = true;
  int L, L2;
  int SRAMIndex = 0;
  int Status;

  aiousb_debug_print("Enter");

  do
  {
    L2 = mCalTable.size() - SRAMIndex;

    if (L2 > mChunkSize) L2 = mChunkSize;
    if (L2 < mChunkSize)
    {
      L2 = mChunkSize;
      SRAMIndex = 0x10000 - mChunkSize/2;
    }
    L = L2;

    Status = AWU_GenericBulkOut(mDevice,
                                0,
                                &(mCalTable.data()[SRAMIndex]),
                                sizeof(mCalTable[0]) * L2, &L);

    if (Status != 0)
    {
      aiousb_library_err_print("GenericBulkOut returned %d", Status);
      return -EIO;
    }

    Status = GenericVendorWrite(mDevice, 0xbb, SRAMIndex, L2, 0, nullptr);

    if (Status != 0)
    {
      aiousb_library_err_print("GenericVendorWrite returned %d", Status);
      return -EIO;
    }

    L = L / 2;
    L = L & mPacketsMask;
    SRAMIndex += L;
    if (FirstChunkDouble)
    {
      FirstChunkDouble = false;
      SRAMIndex = 0;
    }
  }while (SRAMIndex < 0x10000);

  return 0;
}

int SetCalWorker::WriteCalToFile() //might get called twice
{
  if (mOutFileName == nullptr) return 0;
  std::ofstream *OutStream;

  if (mShouldAppend)
  {
    OutStream = new std::ofstream(mOutFileName, std::ios::binary | std::ios::app);
  }
  else
  {
    OutStream = new std::ofstream(mOutFileName, std::ios::binary);
  }
  mShouldAppend = true;

  OutStream->write((const char *)mCalTable.data(), mCalTable.size() * sizeof(uint16_t));
  OutStream->close();
  delete OutStream;
  return 0;

}

uint16_t SetCalWorker::GetHiRef()
{
  uint32_t DataSize, RetVal;
  int status;

  DataSize = sizeof(uint32_t);
  status = GenericVendorRead(mDevice, 0xa2, 0x1df2, 0, DataSize, &RetVal);

  if ((status != sizeof(RetVal)) ||
      (RetVal == 0xffff))
  {
    return 0.0;
  }
  else
  {
    return RetVal;
  }
}

double SetCalWorker::ConfigureAndBulkAcquire(std::vector<uint8_t> &Config)
{
  uint32_t L = mDevice->descriptor.config_bytes;
  uint16_t *AdBuff;
  uint32_t AdBuffLength;
  int Status;
  uint8_t StartChannel, EndChannel;
  uint32_t AdTot;

  Status = ADC_SetConfig(mDevice, Config.data(), &L);

  if (Status != 0)
  {
    aiousb_library_err_print("ADC_Setconfig returned %d", Status);
    return 0.0;
  }

  Status = aiousb_get_scan_inner(mDevice, Config.data(), &L, &AdBuff, &AdBuffLength, &StartChannel, &EndChannel, 0);

  if (Status != 0)
  {
    aiousb_library_err_print("adc_get_scan_inner returned %d", Status);
    return 0.0;
  }

  AdTot = 0;

  for (int i = 0 ; i < Config[0x13] ; i++) AdTot += AdBuff[i];
  return (double)AdTot / (double)(Config[0x13]); //TODO: Figure out why we are
                      //dividing by one less than we should to get right answer



}

int SetCalWorker::SetCal()
{
  uint32_t ConfigSize = mDevice->descriptor.config_bytes;
  std::vector<uint8_t> OldConfig(ConfigSize);
  std::vector<uint8_t> NewConfig(ConfigSize);

  double LoRef, HiRef, dRef, ThisRef, HiRead, LoRead, dRead;
  uint8_t ProbeData;
  int Status;

  ADC_GetConfig(mDevice, OldConfig.data(), &ConfigSize);
  std::fill(NewConfig.begin(), NewConfig.end(), 0);


  Status = GenericVendorRead(mDevice,
                              AUR_PROBE_CALFEATURE,
                              0,
                              0,
                              sizeof(ProbeData),
                              &ProbeData);

  if (Status != sizeof(ProbeData)) return -EIO;

  if (ProbeData != 0xbb) return -ENOTSUP;

  if (!strcmp(":AUTO:", mCalFileName))
  {
    LoRef = LoRefRef;
    HiRef = GetHiRef();
    if ( 0 == HiRef) HiRef = HiRefRef;
    dRef = HiRef - LoRef;

    NewConfig[0x00] = 0x01;
    NewConfig[0x10] = 0x05;

    for (int i ; i < 2 ; i++)
    {
      ADC_SetConfig(mDevice, NewConfig.data(), &ConfigSize);

      for (size_t i = 0 ; i < mCalTable.size() ; i++)
      {
        mCalTable[i] = i;
      }
      LoadCalTable();

      NewConfig[0x11] = 0x04;
      NewConfig[0x12] = 0x00;
      NewConfig[0x13] = std::min(0xff, mChunkSize - 1);
      ThisRef = HiRef;

      if ( 0 == i ) ThisRef = 0.5 * (ThisRef + 0x10000);

      NewConfig[0x10] |= 0x02; //cal low ref

      HiRead = ConfigureAndBulkAcquire(NewConfig);

      if (abs(HiRead - ThisRef) > 0x10000) return -EINVAL;

      usleep(10000);

      ThisRef = LoRef;

      if (0 == i ) ThisRef = 0.5 * (ThisRef + 0x10000);

      NewConfig[0x10] &=~0x02;
      LoRead = ConfigureAndBulkAcquire(NewConfig);
      if (abs(LoRead - ThisRef) > 0x100) return -EINVAL;

      usleep(10000);

      dRead = HiRead - LoRead;

      for (int j = 0 ; j < 0x10000 ; j++)
      {
        double F, J;
        F = (j - LoRead) / dRead;
        F = LoRef + F * dRef;
        if (0 == i) F = 0.5 * (F + 0x10000);
        J = round(F);
        if ( J <= 0)
        {
          J = 0;
        }
        else if (J >= 0xFFFF)
        {
          J = 0xFFFF;
        }
        mCalTable[j] = J;
      }
      LoadCalTable();

      NewConfig[0x10] = 1;
      NewConfig[0] = 1;

      WriteCalToFile();

    }
  }
  else if (!strcmp(":NORM:", mCalFileName))
  {
    LoRef = LoRefRef;
    HiRef = GetHiRef();

    if (0 == HiRef ) HiRef = HiRefRef;

    dRef = HiRef - LoRef;
    LoRead = 0x0042;
    HiRead = 0xfe3f;
    dRead = HiRead - LoRead;

    for (int i = 0 ; i < 0x10000 ; i++)
    {
      double F, J;
      F = (i - LoRead) / dRead;
      F = LoRef +  F * dRef;
      J = round(F);
      if (J <= 0 ) J = 0;
      else if (J >= 0xffff) J = 0xffff;
      mCalTable[i] = J;
    }

    NewConfig[0x10] = 0x05;

    for (int i = 0 ; i < 2 ; i++)
    {
      ADC_SetConfig(mDevice, NewConfig.data(), &ConfigSize);
      LoadCalTable();
      WriteCalToFile();
      NewConfig[0x10] = 0x01;
    }
  }
  else if (!(strcmp(":1TO1:", mCalFileName)) ||
            !(strcmp(":NONE:", mCalFileName)))
  {
    for (size_t i = 0 ; i < mCalTable.size() ; i++)
    {
      mCalTable[i] = i;
    }

        NewConfig[0x10] = 0x05;

    for (int i = 0 ; i < 2 ; i++)
    {
      ADC_SetConfig(mDevice, NewConfig.data(), &ConfigSize);
      LoadCalTable();
      WriteCalToFile();
      NewConfig[0x10] = 0x01;
    }
  }
  else if (mCalFileName[0] == ':') return -EINVAL;
  else
  {
   std::ifstream  CalStream(mCalFileName, std::ios::binary | std::ios::in);
   CalStream.seekg(std::ios::end);
   if (CalStream.tellg() >= 0x20000)
   {
     CalStream.seekg(0);
     NewConfig[0x10] = 0x05;
     NewConfig[0x00] = 0x01;

     for (int i = 0 ; i < 2 ; i++)
     {
       ADC_SetConfig(mDevice, NewConfig.data(), &ConfigSize);
       CalStream.read((char *)mCalTable.data(), mCalTable.size() * sizeof(uint16_t));
       LoadCalTable();
       WriteCalToFile();
       NewConfig[0x10] = 0x01;
       NewConfig[0x00] = 0x00;
     }
   }
   else if (CalStream.tellg() > 0x10000)
   {
     CalStream.seekg(0);
     CalStream.read((char *)mCalTable.data(), mCalTable.size() * sizeof(uint16_t));
     LoadCalTable();
     WriteCalToFile();
   }
   CalStream.close();

  }
  ADC_SetConfig(mDevice, OldConfig.data(), &ConfigSize);
  return 0;
}

int ADC_SetCal(aiousb_device_handle device, const char *CalFileName)
{
  return ADC_SetCalAndSave(device, CalFileName, nullptr);
}

int ADC_SetCalAndSave(aiousb_device_handle device, const char *CalFileName,
                  const char *OutFileName)
{
  SetCalWorker Worker(device, CalFileName, OutFileName);
  return Worker.SetCal();
}

int DAC_SetBoardRange (aiousb_device_handle device, uint32_t range_code)
{
  uint8_t config_data[2] = {0};
  uint32_t data_size = 0;
  uint16_t dac_data;
  int status = 0;

  aiousb_debug_print("Enter");

  if (device->descriptor.b_dac_dio_stream)
    {
      data_size = 2;
      status = GenericVendorRead(device, 0xb7, 0, 0, data_size, config_data);

      if (status)
        {
          aiousb_library_err_print("status = %d", status);
          return status;
        }

      if (config_data[0] >= 2)
        {
          if (config_data[1] != 0)
            {
              dac_data = 0x8000; //Bipolar counts for zero volts
            }
          else
            {
              dac_data = 0; //Unipolor counts for zero volts
            }
            status = DAC_Direct(device, 0, dac_data);
            if (status)
              {
                aiousb_library_err_print("status = %d", status);
                return status;
              }
        }
      status = GenericVendorWrite(device, 0xb7, 0, 0, 0, nullptr);
      if (status)
        {
          aiousb_library_err_print("status = %d", status);
        }
      return status;
    }

    if (!device->descriptor.b_dac_board_range)
    {
      return -EBADRQC;
    }
    status = GenericVendorWrite(device, AUR_DAC_RANGE,
                                          range_code,
                                          0,
                                          0,
                                          nullptr);
    if (status)
      {
        aiousb_library_err_print("status = %d", status);
      }
    return status;
}

int DAC_Direct(aiousb_device_handle device, uint32_t channel, uint16_t counts)
{
  //uint32_t dac_stream_data = 0;
  //double clock_hz = 0;

  if ( 0 == device->descriptor.imm_dacs)
    {
      return -EBADRQC;
    }

  //TODO: reference code checks a couple of state variables within the device
  //structure here. bDACOpen and bDACClosing

  if (channel >= device->descriptor.imm_dacs)
    {
      return -EINVAL;
    }

  if (device->descriptor.b_dac_dio_stream)
    {
      //clock_hz = dac_dio_stream_imm_hz;
      //TODO: Implement aiousb_dac_output_process
      //return aiousb_dac_output_process(device, clock_hz, 2, &counts);
      return -ENOSYS;
    }
  else
    {
      return GenericVendorWrite(device,
                                        AUR_DAC_IMMEDIATE,
                                        counts,
                                        channel,
                                        0,
                                        nullptr);
    }

}

int DAC_MultiDirect(aiousb_device_handle device,
                    void * dac_data, uint32_t data_count)
{
  uint32_t max_channel = 0;
  uint16_t *dac_data_ptr = (uint16_t *)dac_data;

  if ( 0 == device->descriptor.imm_dacs)
    {
      return -EBADRQC;
    }

  //TODO: reference code checks a couple of state variables within the device
  //structure here. bDACOpen and bDACClosing

  if (0 == data_count) return 0;

  for (unsigned int i = 0 ; i < data_count ; i++)
    {
      if (dac_data_ptr[i * 2] > max_channel)
        {
          max_channel = dac_data_ptr[i * 2];
        }
    }

  if (max_channel > device->descriptor.imm_dacs) return -EINVAL;

  if (device->descriptor.b_dac_dio_stream)
    {
      return -ENOSYS;
    }
  else
    {
      int DacBlocks = max_channel / 8 + 1;
      size_t ContentSize = DacBlocks * 17;
      uint8_t  *Content = new uint8_t[ContentSize];
      uint16_t *ptr = (uint16_t *)(dac_data);
      int status;


      memset(Content, 0, ContentSize);

      for (unsigned int i = 0 ; i < data_count; i++)
      {
        int Channel = ptr[i*2];
        memcpy(&(Content[Channel * 2 + (Channel / 8) + 1]),
                &(ptr[i * 2 + 1]),
                sizeof(uint16_t));
        Content[(Channel / 8) * 17] |= 1 << (Channel & 7);
      }

      status = GenericVendorWrite(device, AUR_DAC_IMMEDIATE, 0, 0, ContentSize, Content);

      delete[] Content;

      if (status == (int)ContentSize) status = 0;

      return status;

    }


  return -EBADRQC;

}


int AbortPipe(aiousb_device_handle device)
{
  return ioctl(device->fd, ACCESIO_USB_ABORT_PIPE);
}

int ResetChip(aiousb_device_handle device)
{
  int status = 0;
  uint8_t CPUCSByte;

  CPUCSByte = 0x01;

  status = GenericVendorWrite(device, 0xa0, 0xe600, 0, 1, &CPUCSByte);

  if (status)
  {
    aiousb_library_err_print("Error writing to chip. status = %d", status);
    return status;
  }

  CPUCSByte = 0x00;
  status = GenericVendorWrite(device, 0xa0, 0xe600, 0, 1, &CPUCSByte);

  if (status)
  {
    aiousb_library_err_print("Error writing to chip. status = %d", status);
  }

  return status;
}


int QueryDeviceInfo(unsigned long device_index,
                            uint32_t *pid, uint32_t *name_size, char *name,
                            uint32_t *dio_bytes, uint32_t *counters)
{
  return QueryDeviceInfo(aiousb_handle_by_index_private(device_index),
                            pid,
                            name_size,
                            name,
                            dio_bytes,
                            counters);
}

int GetDeviceSerialNumber(unsigned long device_index, uint64_t *serial_number)
{
  return GetDeviceSerialNumber(aiousb_handle_by_index_private(device_index),
                                  serial_number);
}

int GenericVendorRead(unsigned long device_index,
            uint8_t request, uint16_t value, uint16_t index,
            uint16_t size, void *data)
{
  return GenericVendorRead(aiousb_handle_by_index_private(device_index),
                                      request,
                                      value,
                                      index,
                                      size,
                                      data);
}

int GenericVendorWrite(unsigned long device_index,
            uint8_t request, uint16_t value, uint16_t index,
            uint16_t size, void *data)
{
  return GenericVendorWrite(aiousb_handle_by_index_private(device_index),
                              request,
                              value,
                              index,
                              size,
                              data);
}

int AWU_GenericBulkIn (unsigned long device_index,
          unsigned int pipe_index, void *data, int size,
          int *transferred)
{
  return AWU_GenericBulkIn (aiousb_handle_by_index_private(device_index),
                  pipe_index,
                  data,
                  size,
                  transferred);
}

int AWU_GenericBulkOut (unsigned long device_index, unsigned int pipe_index,
           void *data, int size,	int *transferred)
{
  return AWU_GenericBulkOut (aiousb_handle_by_index_private(device_index),
                            pipe_index,
                            data,
                            size,
                            transferred);
}

int CustomEEPROMWrite(unsigned long device_index,
            uint32_t start_address, uint32_t data_size, void *data)
{
  return CustomEEPROMWrite(aiousb_handle_by_index_private(device_index),
            start_address,
            data_size,
            data);
}

int CustomEEPROMRead(unsigned long device_index,
            uint32_t start_address, uint32_t data_size, void *data)
{
  return CustomEEPROMRead(aiousb_handle_by_index_private(device_index),
              start_address,
              data_size,
              data);
}

int DIO_Configure (unsigned long device_index, uint8_t b_tristate,
          void *out_mask, void *data)
{
  return DIO_Configure (aiousb_handle_by_index_private(device_index),
              b_tristate,
              out_mask,
              data);
}

int DIO_ConfigureEx (unsigned long device_index, void * out_mask,
                void *data, void *tristate_mask)
{
  return DIO_ConfigureEx (aiousb_handle_by_index_private(device_index),
                              out_mask,
                              data,
                              tristate_mask);
}

int DIO_ConfigureMasked(unsigned long device_index, void *outs,
                void *outs_mask, void *data, void *data_mask, void *tristates,
                void *tristates_mask)
{
  return DIO_ConfigureMasked(aiousb_handle_by_index_private(device_index),
                              outs,
                              outs_mask,
                              data,
                              data_mask,
                              tristates,
                              tristates_mask);
}

int DIO_WriteAll(unsigned long device_index, void *data)
{
  return DIO_WriteAll(aiousb_handle_by_index_private(device_index), data);
}


int DIO_Write8(unsigned long device_index, uint32_t byte_index,
                uint8_t data)
{
  return DIO_Write8(aiousb_handle_by_index_private(device_index),
                byte_index,
                data);
}

int DIO_Write1(unsigned long device_index, uint32_t bit_index,
                uint8_t data)
{
  return DIO_Write1(aiousb_handle_by_index_private(device_index),
                      bit_index,
                      data);
}

int DIO_ReadAll(unsigned long device_index, void *data)
{
  return DIO_ReadAll(aiousb_handle_by_index_private(device_index), data);
}

int DIO_Read8(unsigned long device_index, uint32_t byte_index,
                uint8_t *data)
{
  return DIO_Read8(aiousb_handle_by_index_private(device_index),
                byte_index,
                data);
}

int DIO_Read1(unsigned long device_index, uint32_t bit_index,
                uint8_t *data)
{
  return DIO_Read1(aiousb_handle_by_index_private(device_index),
                        bit_index,
                        data);
}

int DIO_ConfigurationQuery(unsigned long device_index, void *out_mask,
                void *tristate_mask)
{
  return DIO_ConfigurationQuery(aiousb_handle_by_index_private(device_index),
                          out_mask,
                          tristate_mask);
}

int DIO_StreamOpen(unsigned long device_index, uint32_t is_read)
{
  return DIO_StreamOpen(aiousb_handle_by_index_private(device_index), is_read);
}

int DIO_StreamClose(unsigned long device_index)
{
  return DIO_StreamClose(aiousb_handle_by_index_private(device_index));
}

int DIO_StreamSetClocks(unsigned long device_index, double *read_hz,
                double *write_hz)
{
  return DIO_StreamSetClocks(aiousb_handle_by_index_private(device_index),
                      read_hz,
                      write_hz);
}

int DIO_StreamFrame (unsigned long device_index, unsigned long frame_points,
                unsigned short *frame_data, size_t *bytes_transferred)
{
  return DIO_StreamFrame (aiousb_handle_by_index_private(device_index),
                frame_points,
                frame_data,
                bytes_transferred);
}

int CTR_8254Mode(unsigned long device_index, uint32_t block_index,
                uint32_t counter_index, uint32_t mode)
{
  return CTR_8254Mode(aiousb_handle_by_index_private(device_index),
                block_index,
                counter_index,
                mode);
}

int CTR_8254Load(unsigned long device_index, uint32_t block_index,
                uint32_t counter_index, uint16_t load_value)
{
  return CTR_8254Load(aiousb_handle_by_index_private(device_index),
                block_index,
                counter_index,
                load_value);
}

int CTR_8254ModeLoad(unsigned long device_index, uint32_t block_index,
                uint32_t counter_index, uint32_t mode, uint16_t load_value)
{
  return CTR_8254ModeLoad(aiousb_handle_by_index_private(device_index),
                      block_index,
                      counter_index,
                      mode,
                      load_value);
}

int CTR_8254StartOutputFreq(unsigned long device_index,
                uint32_t block_index, double *frequency)
{
  return CTR_8254StartOutputFreq(aiousb_handle_by_index_private(device_index),
                block_index,
                frequency);
}

int CTR_8254Read(unsigned long device_index, uint32_t block_index,
                uint32_t counter_index, uint16_t *read_value)
{
  return CTR_8254Read(aiousb_handle_by_index_private(device_index),
                  block_index,
                  counter_index,
                  read_value);
}

int CTR_8254ReadAll(unsigned long device_index, uint16_t *data)
{
  return CTR_8254ReadAll(aiousb_handle_by_index_private(device_index),
                                data);
}

int CTR_8254ReadStatus(unsigned long device_index, uint32_t block_index,
                uint32_t counter_index, uint8_t *read_value, uint8_t *status)
{
  return CTR_8254ReadStatus(aiousb_handle_by_index_private(device_index),
                            block_index,
                            counter_index,
                            read_value,
                            status);
}

int CTR_8254ReadModeLoad(unsigned long device_index, uint32_t block_index,
                uint32_t counter_index, uint32_t mode, uint16_t load_value,
                uint16_t *read_value)
{
  return CTR_8254ReadModeLoad(aiousb_handle_by_index_private(device_index),
                      block_index,
                      counter_index,
                       mode,
                       load_value,
                       read_value);
}

int ADC_GetScanV(unsigned long device_index, double *data)
{
  return ADC_GetScanV(aiousb_handle_by_index_private(device_index), data);
}

int ADC_GetChannelV (unsigned long device_index, uint32_t channel_index,
                double *volts)
{
  return ADC_GetChannelV (aiousb_handle_by_index_private(device_index),
                        channel_index,
                        volts);
}

int ADC_GetTrigScanV (unsigned long device_index, double *data,
                uint32_t timeout_ms);

int ADC_SetScanLimits (unsigned long device_index, uint32_t start_channel,
                uint32_t end_channel)
{
  return ADC_SetScanLimits (aiousb_handle_by_index_private(device_index),
                start_channel,
                end_channel);
}

int ADC_BulkContinuousStart (unsigned long device_index,
                uint32_t buff_size, uint32_t base_buff_count, void *context,
                adc_cont_callback callback)
{
  return ADC_BulkContinuousStart (aiousb_handle_by_index_private(device_index),
                buff_size,
                base_buff_count,
                context,
                callback);
}

int ADC_BulkContinuousEnd (unsigned long device_index)
{
  return ADC_BulkContinuousEnd(aiousb_handle_by_index_private(device_index));
}


int ADC_GetConfig(unsigned long device_index, uint8_t *config_buff,
                uint32_t *config_size)
{
  return ADC_GetConfig(aiousb_handle_by_index_private(device_index),
                          config_buff,
                          config_size);
}

int ADC_RangeAll(unsigned long device_index, uint8_t *gain_codes,
                uint32_t *b_differential)
{
  return  ADC_RangeAll(aiousb_handle_by_index_private(device_index),
                      gain_codes,
                      b_differential);
}

int ADC_Range1(unsigned long device_index, uint32_t adc_channel,
                uint8_t gain_code, uint32_t b_differential)
{
  return ADC_Range1(aiousb_handle_by_index_private(device_index),
                      adc_channel,
                      gain_code,
                      b_differential);
}

int ADC_SetCal(unsigned int device_index, const char *CalFileName)
{
  return ADC_SetCal(aiousb_handle_by_index_private(device_index),
                      CalFileName);
}

int ADC_SetCalAndSave(unsigned int device_index, const char *CalFileName,
                  const char *OutFileName)
{
  return ADC_SetCalAndSave(aiousb_handle_by_index_private(device_index),
                        CalFileName,
                        OutFileName);
}


int ADC_SetOversample(unsigned long device_index, uint8_t oversample)
{
  return ADC_SetOversample(aiousb_handle_by_index_private(device_index), oversample);
}

int ADC_SetConfig(unsigned long device_index, uint8_t *config_buff,
              uint32_t *config_size)
{
  return ADC_SetConfig(aiousb_handle_by_index_private(device_index),
                          config_buff,
                          config_size);
}


int ADC_InitFastScanV(unsigned long device_index)
{
  return ADC_InitFastScanV(aiousb_handle_by_index_private(device_index));
}

int ADC_GetFastScanV(unsigned long device_index, double *data)
{
  return ADC_GetFastScanV(aiousb_handle_by_index_private(device_index), data);
}

int ADC_ResetFastScanV(unsigned long device_index)
{
  return ADC_ResetFastScanV(aiousb_handle_by_index_private(device_index));
}

int DAC_SetBoardRange (unsigned long device_index, uint32_t range_code)
{
  return DAC_SetBoardRange(aiousb_handle_by_index_private(device_index),
                            range_code);
}

int DAC_Direct(unsigned long device_index, uint32_t channel, uint16_t counts)
{
  return DAC_Direct(aiousb_handle_by_index_private(device_index),
                                                    channel,
                                                    counts);
}

int DAC_MultiDirect(unsigned long device_index, void * dac_data, uint32_t data_count)
{
  return DAC_MultiDirect(aiousb_handle_by_index_private(device_index),
                                                        dac_data,
                                                        data_count);
}



int AbortPipe(unsigned long device_index)
{
  return AbortPipe(aiousb_handle_by_index_private(device_index));
}

int ResetChip (unsigned long device_index)
{
  return ResetChip(aiousb_handle_by_index_private(device_index));
}

} //namespace AIOUSB

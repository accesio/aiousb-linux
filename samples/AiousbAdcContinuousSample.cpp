#include <unistd.h>
#include "aiousb.h"
#include "sample_helpers.inc"

#include "AiousbSamples.inc"

#define CONFIG_SIZE 21
#define START_CHANNEL 0
#define END_CHANNEL 15
#define FREQUENCY 125000.0
#define OVERSAMPLE 10
#define SAMPLE_SIZE 2
#define BUFFER_SIZE  512 * SAMPLE_SIZE * ( OVERSAMPLE + 1) * (END_CHANNEL - START_CHANNEL + 1)
#define DURATION 10


struct CbContext {
  int StartChannel;
  int EndChannel;
  uint32_t ConfigSize;
  uint8_t Config[CONFIG_SIZE];
};

void ADContCallback (uint16_t *buff, uint32_t buff_size,
                    uint32_t flags, void *context);

int main (int argc, char **argv)
{
  int Status;
  AIOUSB::aiousb_device_handle Device;
  uint32_t Pid;
  int ChannelCount;
  uint64_t SerialNum;
  struct CbContext Context = {0};
  uint32_t NameSize = 255;
  char Name[NameSize];
  double Frequency = FREQUENCY;

  std::cout<<"ACCES AIOUSB-Linux ADC sample"<<std::endl;

  AIOUSB::AiousbInit();

  //Try to get a device handle.
  Status = SampleGetDeviceHandle(argc, argv, &Device);

  if ( 0 != Status )
  {
    std::cout << "Unable to get device handle" << std::endl;
    SampleUsage(argc, argv);
    exit (-1);
  }

  AIOUSB::QueryDeviceInfo(Device, &Pid, &NameSize, Name, nullptr, nullptr);
  AIOUSB::GetDeviceSerialNumber(Device, &SerialNum);
  std::cout << Name << " detected [" << std::hex << Pid << std::dec << "]" << std::endl;
  std::cout << "Serial Number: " <<std::hex << SerialNum << std::dec << std::endl;

   Context.ConfigSize = sizeof(Context.Config);
   //Generate a sample config to do a scan of all channels
   Context.Config[0x00] = 0x01; // range
   Context.Config[0x01] = 0x01; // range
   Context.Config[0x02] = 0x01; // range
   Context.Config[0x03] = 0x01; // range
   Context.Config[0x04] = 0x01; // range
   Context.Config[0x05] = 0x01; // range
   Context.Config[0x06] = 0x01; // range
   Context.Config[0x07] = 0x01; // range
   Context.Config[0x08] = 0x01; // range
   Context.Config[0x09] = 0x01; // range
   Context.Config[0x0A] = 0x01; // range
   Context.Config[0x0B] = 0x01; // range
   Context.Config[0x0C] = 0x01; // range
   Context.Config[0x0D] = 0x01; // range
   Context.Config[0x0E] = 0x01; // range
   Context.Config[0x0F] = 0x01; // range
   Context.Config[0x10] = 0x00; // calibration code
   Context.Config[0x11] = 0x05; // trigger & counter mode

   Context.StartChannel = 0;
   Context.EndChannel = 0xf;
   // Start and End Channel, low nybbles 0bEEEESSSS  (0xF0 = 0b11110000)
  Context.Config[0x12] = (Context.EndChannel & 0xf) << 4 | (Context.StartChannel & 0xf);

  Context.Config[0x13] = OVERSAMPLE; // Oversamples

  // Start and End Channel, high nybbles 0bEEEESSSS (0x50 = 0b01010000)
  Context.Config[0x14] = (Context.EndChannel & 0xf0) | (Context.StartChannel & 0xf0) >> 4;

  Status = AIOUSB::ADC_SetConfig(Device, Context.Config, &Context.ConfigSize);

  if (Status)
  {
    printf("Error setting config\n");
    return -1;
  }

  std::cout<< "ADC_SetConfig returned " << Status << std::endl;

  Status = AIOUSB::CTR_8254StartOutputFreq(Device, 0, &Frequency);

  if (Status)
  {
    printf("Error starting countern");
    return -1;
  }

  std::cout<< "Set Frequency: " << FREQUENCY << " Actual: " << Frequency << std::endl;

  return 0;

  Status = AIOUSB::ADC_BulkContinuousStart(Device,
                          BUFFER_SIZE,
                          1,
                          &Context,
                          ADContCallback);

  if (Status)
  {
    printf("Error starting continuous acquisition\n");
    return -1;
  }
  else
  {
    sleep(DURATION);
  }

  AIOUSB::ADC_BulkContinuousEnd(Device);

  sleep(1);
}


// Calculate Volts from a range code and counts. Information about the range codes
// and the rest of the configuration buffer used by the ACCES AD devices can be
// found at 
// https://accesio.com/MANUALS/USB%20Software%20Reference%20Manual.html#ADC_SetConfig()
// Range Code   Range
// 0            0 to 10V
// 1            -10 to 10V
// 2            0 to 5V
// 3            -5 to 5V
// 4            0 to 2V
// 5            -2 to 2V
// 6            0 to 1V
// 7            -1 to 1V
double VoltsFromCounts (int counts, int range_code)
{
  double v = counts * (1.0 /65536.0);

  if (range_code & 0x1)
    {
      v = v * 2 - 1;
    }
  if ((range_code & 0x2) == 0)
    {
      v = v * 5;
    }
  if ((range_code & 0x4) == 0)
    {
      v = v * 2;
    }
  return v;
}

void ADContCallback (uint16_t *buff, uint32_t buff_size,
                    uint32_t flags, void *context)
{
  struct CbContext *Context = (struct CbContext *)context;
  
  for (int i = Context->StartChannel ; i < Context -> EndChannel ; i++)
  {
    double volts; 
    uint16_t counts;
    counts = 0;
    for (int j = 0 ; j < Context->Config[0x13] ; j++)
    {
      int n = j + 1;
      counts = counts * (n - 1) / n + (buff[i * Context->Config[0x13] + j])/n;
    }
    volts = VoltsFromCounts(counts, Context->Config[i]);
    printf("Channel: %d, Volts: %f\n", i, volts);
  }

  // uint8_t *Config = (uint8_t *)context;
  // double volts;

  // for (int sample = 0; sample <(END_CHANNEL - START_CHANNEL + 1); sample++)
  // {
    // //this volts assumes a range of 0 - 10 v and no post_scale.
    // //see ADC_GetScanV for a more complete implementation.
    // volts = buff[sample] * 1.0/65536.0;
    // volts = volts * 10;
    // //printf("channel %d: %f\n", sample + 1, volts);
  // }
}
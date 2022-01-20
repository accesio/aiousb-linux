#include <iomanip>
#include <cstring>
#include <list>
#include <algorithm>
#include <chrono>
#include <thread>

#include "aiousb.h"

static const std::list<uint16_t> PIDS_WITH_PER_BIT = {0x8004};

#include "AiousbSamples.inc"

int main (int argc, char **argv)
{
  int Status;
  AIOUSB::aiousb_device_handle Device;
  uint32_t Pid;

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

  uint32_t nameSize = 255;
  char name[nameSize];

  // DIO bytes is something we can query from the aiousb library so we see how
  // many are available on this device.
  AIOUSB::QueryDeviceInfo(Device, &Pid, &nameSize, name, nullptr, nullptr);

  std::cout << name << " detected [" << std::hex << Pid << std::dec << "]" << std::endl;

  uint32_t ConfigSize = 21;
  uint8_t *Config = new uint8_t[ConfigSize];
  Config[0x00] = 0x01; // range
  Config[0x01] = 0x01; // range
  Config[0x02] = 0x01; // range
  Config[0x03] = 0x01; // range
  Config[0x04] = 0x01; // range
  Config[0x05] = 0x01; // range
  Config[0x06] = 0x01; // range
  Config[0x07] = 0x01; // range
  Config[0x08] = 0x01; // range
  Config[0x09] = 0x01; // range
  Config[0x0A] = 0x01; // range
  Config[0x0B] = 0x01; // range
  Config[0x0C] = 0x01; // range
  Config[0x0D] = 0x01; // range
  Config[0x0E] = 0x01; // range
  Config[0x0F] = 0x01; // range
  Config[0x10] = 0x00; // calibration code
  Config[0x11] = 0x05; // trigger & counter mode
  Config[0x12] = 0xF0; // Start and End Channel, low nybbles 0bEEEESSSS  (0xF0 = 0b11110000)
  Config[0x13] = 0x00; // Oversamples
  Config[0x14] = 0x50; // Start and End Channel, high nybbles 0bEEEESSSS (0x50 = 0b01010000) so start channel is 0, end channel is 0x5F

  Status = AIOUSB::ADC_SetConfig(Device, Config, &ConfigSize);
  std::cout<< "ADC_SetConfig returned " << Status << std::endl;

  double *volts = new double[128];
  Status = AIOUSB::ADC_GetScanV(Device, volts);
  std::cout << "ADC_GetScanV returned " << Status << std::endl;

  for (int ch = 0; ch < 96; ch++) // this code currently assumes a 96-channel device is present...
  {
    std::cout << std::setw(3) << ch << std::setw(3) << ": " << volts[ch] << std::endl;
  }

  delete[] Config;
  delete[] volts;

  return 0;
}

#include <iomanip>
#include <cstring>
#include <list>
#include <algorithm>
#include <chrono>
#include <thread>
#include <getopt.h>

#include "aiousb.h"
#include "sample_helpers.inc"

#define START_CHANNEL 0 /*for instructional purposes building the config*/

#include "AiousbSamples.inc"

struct option long_options[] = {
  {"differential", no_argument, nullptr, 'd'},
  {"help", no_argument, nullptr, 'h'},
  {nullptr, 0, nullptr, 0}
};

void Usage () 
{
  std::cout << "Usage:" << std::endl;
  std::cout << "AiousbAdcSample [--differential] [<device_file>]" << std::endl;
  std::cout << "If no device file is specified and only one AIOUSB device is detected that lone device will be used" << std::endl;
}

int GetNumAdcMuxChan(uint32_t Pid);
int GetNumAdcsImm(uint32_t Pid);

void DoChanScan(AIOUSB::aiousb_device_handle Device, int ChannelCount, uint8_t ChannelByte);
void DoImmScan(AIOUSB::aiousb_device_handle Device, int ChannelCount, uint8_t ChannelByte);

int main (int argc, char **argv)
{
  int Status;
  AIOUSB::aiousb_device_handle Device;
  uint32_t Pid;
  int ChannelCount;
  uint64_t SerialNum;
  uint8_t ChannelByte = 0x01; //default to 10V bipolar range
  int option;

  std::cout<<"ACCES AIOUSB-Linux ADC sample"<<std::endl;

  //handle command line arguments
  do
  {
    option = getopt_long(argc, argv, "", long_options, nullptr);
    switch(option)
    {
      case 'd':
        ChannelByte |= 0x08; //set the 4th bit to enable differential mode
        std::cout << "Differential mode enabled" << std::endl;
        break;
      case 'h':
      case '?':
        Usage();
        exit(0);
        break;
      case -1: //end of options
        break;
    }
  }while (-1 != option);
  optind--;

  AIOUSB::AiousbInit();

  //Try to get a device handle.
  Status = SampleGetDeviceHandle(argc - optind, &argv[optind], &Device);

  if ( 0 != Status )
  {
    std::cout << "Unable to get device handle" << std::endl;
    SampleUsage(argc, argv);
    exit (-1);
  }

  uint32_t NameSize = 255;
  char Name[NameSize];

  AIOUSB::QueryDeviceInfo(Device, &Pid, &NameSize, Name, nullptr, nullptr);
  AIOUSB::GetDeviceSerialNumber(Device, &SerialNum);
  std::cout << Name << " detected [" << std::hex << Pid << std::dec << "]" << std::endl;
  std::cout << "Serial Number: " <<std::hex << SerialNum << std::dec << std::endl;

  ChannelCount = GetNumAdcMuxChan(Pid);
  if (0 != ChannelCount) DoChanScan(Device, ChannelCount, ChannelByte);

  ChannelCount = GetNumAdcsImm(Pid);
  if (0 != ChannelCount) DoImmScan(Device, ChannelCount, ChannelByte);


}

//USB-AI style boards
void DoChanScan(AIOUSB::aiousb_device_handle Device, int ChannelCount, uint8_t ChannelByte)
{
  int Status;
  double Volts[ChannelCount];
  uint32_t ConfigSize = 21;
  uint8_t Config[ConfigSize];
  int EndChannel = ChannelCount - 1;

  //Generate a sample config to do a scan of all channels
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

  // Start and End Channel, low nybbles 0bEEEESSSS  (0xF0 = 0b11110000)
  Config[0x12] = (EndChannel & 0xf) << 4 | (START_CHANNEL & 0xf);

  Config[0x13] = 0x00; // Oversamples

  // Start and End Channel, high nybbles 0bEEEESSSS (0x50 = 0b01010000)
  Config[0x14] = (EndChannel & 0xf0) | (START_CHANNEL & 0xf0) >> 4;

  Status = AIOUSB::ADC_SetConfig(Device, Config, &ConfigSize);

    std::cout<< "ADC_SetConfig returned " << Status << std::endl;

  Status = AIOUSB::ADC_GetScanV(Device, Volts);

  std::cout << "ADC_GetScanV returned " << Status << std::endl;

  for (int ch = 0; ch < ChannelCount; ch++)
  {
    std::cout << std::setw(3) << ch << std::setw(3) << ": " << Volts[ch] << std::endl;
  }

  //ADC_GetScanV 

}

//USB-AO boards with immediete ADCs
void DoImmScan(AIOUSB::aiousb_device_handle Device, int ChannelCount, uint8_t ChannelByte)
{
  int Status;
  double Volts[ChannelCount];

  Status = AIOUSB::DAC_SetBoardRange(Device, 0); // used to unsleep the ADC reference on USB-AO16-16A and relateds to the adc-getscanv and now it works
  std::cout << "DAC_SetBoardRange returned " << Status << std::endl;

  Status = AIOUSB::ADC_GetScanV(Device, Volts);

  std::cout << "ADC_GetScanV returned " << Status << std::endl;

  for (int ch = 0; ch < ChannelCount; ch++)
  {
    std::cout << std::setw(3) << ch << std::setw(3) << ": " << Volts[ch] << std::endl;
  }
}

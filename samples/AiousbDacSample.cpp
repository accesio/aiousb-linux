#include <thread>
#include <stdint.h>

#include "aiousb.h"

#include "AiousbSamples.inc"
#include "sample_helpers.inc"

#define ACCES_DAC_RANGE_UNI 0
#define ACCES_DAC_RANGE_BI 1



int main (int argc, char **argv)
{
  int Status;
  AIOUSB::aiousb_device_handle Device;
  uint32_t Pid;
  uint32_t NameSize = 255;
  char Name[NameSize];
  int NumDacs;

  std::cout << "ACCES AIOUSB-Linux DAC sample" << std::endl;

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
  std::cout << Name << " detected [" << std::hex << Pid << std::dec << "]" << std::endl;

  NumDacs = GetNumDacs(Pid);

  if (0 == NumDacs)
  {
    std::cout << "Sample not for device with no DACs" << std::endl;
    return 0;
  }


  std::cout << "Setting DAC range to unipolar, status: " << AIOUSB::DAC_SetBoardRange(Device, 0) << std::endl;

  for (int i = 0 ; i < NumDacs ; i ++)
  {
    std:: cout << "Setting 0 counts on channel " << i << " for three seconds, status: " << AIOUSB::DAC_Direct(Device, i, 0)<< std::endl;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(3000));

  for (int i = 0 ; i < NumDacs ; i ++)
  {
    std:: cout << "Setting 0x8000 counts on channel " << i << " for three seconds, status: " << AIOUSB::DAC_Direct(Device, i, 0x8000)<< std::endl;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(3000));

  for (int i = 0 ; i < NumDacs ; i ++)
  {
    std:: cout << "Setting 0xf000 counts on channel " << i << " for three seconds, status: " << AIOUSB::DAC_Direct(Device, i, 0xf000)<< std::endl;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(3000));

  if (NumDacs == 1)
  {
    std:: cout << "Setting 0 counts on channel 0, status: " << AIOUSB::DAC_Direct(Device, 0, 0)<< std::endl;
  }
  else
  {
    uint16_t dac_data[NumDacs * 2]; //dac_data is two words per data point
                                //dac_data[0] is the channel for the counts in dac_data[1]

    for (int i = 0 ; i < NumDacs ; i++ )
    {
      dac_data[i * 2] = i; //even offsets are channel
      dac_data[i * 2 + 1] = 0xffff/(i + 1); //odd offsets are counts
    }

    std::cout << "Setting all channels with DacMultiDirect(), status = " << AIOUSB::DAC_MultiDirect(Device, dac_data, NumDacs) << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));

    for (int i = 0 ; i < NumDacs ; i++ )
    {
      dac_data[i * 2 + 1] = 0;
    }

    std::cout << "Setting all channels to 0 counts with  DacMultiDirect(), status = " << AIOUSB::DAC_MultiDirect(Device, dac_data, NumDacs) << std::endl;

  }

  return 0;

}


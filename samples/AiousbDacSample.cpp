#include <thread>

#include "aiousb.h"

#include "AiousbSamples.inc"

#define ACCES_DAC_RANGE_UNI 0
#define ACCES_DAC_RANGE_BI 1


int main (int argc, char **argv)
{
  int Status;
  AIOUSB::aiousb_device_handle Device;

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

  std::cout << "Setting DAC range to unipolar, status: " << AIOUSB::DAC_SetBoardRange(Device, 0) << std::endl;

  std::cout << "Setting 0 counts on channel 0 for 3 seconds, status: " << AIOUSB::DAC_Direct(Device, 0, 0)<< std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds(3000));

  std::cout << "Setting 0x8000 counts on channel 0x8000 for 3 seconds, status: " <<  AIOUSB::DAC_Direct(Device, 0, 0x8000) << std::endl;;
  std::this_thread::sleep_for(std::chrono::milliseconds(3000));

  std::cout << "Setting 0xF000 counts on channel 0xF000 for 3 seconds, status: " <<  AIOUSB::DAC_Direct(Device, 0, 0xF000) << std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds(3000));

  return 0;
}
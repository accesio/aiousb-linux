#include <chrono>
#include <thread>
#include <iomanip>

#include <string.h>

#include "aiousb.h"

#include "AiousbSamples.inc"




int main (int argc, char **argv)
{
  int Status;
  AIOUSB::aiousb_device_handle Device;

  AIOUSB::AiousbInit();

  Status = SampleGetDeviceHandle(argc, argv, &Device);

  if (Status)
  {
    std::cout << "Unable to get device handle";
    exit(-1);
  }

  std::cout << "Bit toggling with Write1()" << std::endl;
  for (int i = 0 ; i < 8 ; i++)
  {
    AIOUSB::DIO_Write1(Device, i, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    AIOUSB::DIO_Write1(Device, i, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

std::cout << "Bit toggling with Write8()" << std::endl;
  for (int i = 0 ; i < 8 ; i++)
  {
    AIOUSB::DIO_Write8(Device, 0, 1 << i);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    AIOUSB::DIO_Write8(Device, 0, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  uint8_t Data[4] = {0};

  std::cout << "Bit toggling with WriteAll()" << std::endl;
  for (int i = 0 ; i < 8 ; i++ )
  {
    memset(Data, i, sizeof(Data));
    AIOUSB::DIO_WriteAll(Device, Data);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    memset(Data, 0, sizeof(Data));
    AIOUSB::DIO_WriteAll(Device, Data);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  std::cout << "Reading by bit" << std:: endl;

  for (int i = 0 ; i < 8 ; i++)
  {
    AIOUSB::DIO_Read1(Device, i, Data);
    std::cout << "bit " << i << ": " << std::setw(2) << std::setfill('0') << Data[0] << std::endl;
  }

  std::cout << "Reading by byte" << std::endl;

  for (int i = 0 ; i < 4 ; i++)
  {
    AIOUSB::DIO_Read8(Device, i, Data);
    std::cout << "byte " << i << ": " << std::setw(2) << std::setfill('0') << std::hex << Data[0] << std::endl;
  }

  std::cout << "Reading all" << std::endl;

  AIOUSB::DIO_ReadAll(Device, Data);
  std::cout << "All: " << "0x" << std::hex << std::setw(8) << std::setfill('0')  << uint32_t(*Data) << std::endl;







}
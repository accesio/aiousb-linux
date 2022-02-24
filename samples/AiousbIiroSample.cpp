


// From the manual:
// The board’s outputs are comprised of sixteen FORM C SPDT electro mechanical relays. These relays are all
// de-energized at power-on. Please note that once your hardware is properly installed in your system
// you will likely want to run one of our sample programs to exercise the I/O. When doing so, as soon
// as the sample program is run it will turn on all the relays and leave them in that state until you turn
// them all off within the sample.

#include <list>
#include <algorithm>
#include <string.h>
#include <chrono>
#include <thread>
#include <iomanip>

#include "aiousb.h"

#include "AiousbSamples.inc"

static const std::list<uint16_t> IIRO_PIDS =
{
  0x8010,
  0x8011,
  0x8012,
  0x8014,
  0x8015,
  0x8016,
  0x8018,
  0x8019,
  0x801A,
  0x801C,
  0x801D,
  0x801E,
  0x801F,
};

int main (int argc, char **argv)
{
  int Status;
  AIOUSB::aiousb_device_handle Device;
  uint64_t SerialNum;

  std::cout << "ACCES AIOUSB Linux IIRO sample"<<std::endl;

  AIOUSB::AiousbInit();

  Status = SampleGetDeviceHandle(argc, argv, &Device);

  if ( 0 != Status )
  {
    std::cout << "Unable to get device handle" << std::endl;
    SampleUsage(argc, argv);
    exit (-1);
  }

  uint32_t NameSize = 255;
  char Name[NameSize];
  uint32_t Pid;

  AIOUSB::QueryDeviceInfo(Device, &Pid, &NameSize, Name, nullptr, nullptr);
  AIOUSB::GetDeviceSerialNumber(Device, &SerialNum);
  std::cout << Name << " detected [" << std::hex << Pid << std::dec << "]" << std::endl;
  std::cout << "Serial Number: " <<std::hex << SerialNum << std::dec << std::endl;

  std::cout << Name << " detected" << std::endl;

  if (std::find(IIRO_PIDS.begin(), IIRO_PIDS.end(), Pid) == IIRO_PIDS.end())
  {
    std::cout << "This sample is not intended for use with " << Name << std::endl;
    exit(-1);
  }

  uint8_t Data[4] = { 0xff, 0xff, 0xff, 0xff };

  std::cout << "Energizing all relays" << std::endl;
  AIOUSB::DIO_WriteAll(Device, Data);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  std::cout << "Performing walking bit on relays" << std::endl;

  for (int i = 0 ; i < 8 ; i++)
  {
    AIOUSB::DIO_Write1(Device, i, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    AIOUSB::DIO_Write1(Device, i, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  std::cout << "Reading input " << std::endl;
  AIOUSB::DIO_ReadAll(Device, Data);

  for (int i = 0 ; i < 4 ; i++)
  {
    std::cout << "bits "<< std::dec << i*8 << "-" << (i+1)*8-1 << ": 0x" << std::hex << std::setw(2) << std::setfill('0') << int(Data[i]) << std::endl;
  }
  std::cout << std::endl;


}
#include "aiousb.h"
#include "AiousbSamples.inc"

#include <array>

#include <unistd.h>

#define START_CHANNEL 0
#define END_CHANNEL 15

AIOUSB::aiousb_device_handle Device;
double Readings[16];

void DacDirectTest()
{
  int status;

  status = AIOUSB::DAC_SetBoardRange(Device, 1);

  printf("DAC_SetBoardRange status = %d\n", status);


  status = AIOUSB::DAC_Direct(Device, 0, 0xffff);

  printf("Set DAC0 high status = %d\n", status);
  sleep(5);
  status = AIOUSB::DAC_Direct(Device, 0, 0);
  printf("Set DAC0 low status = %d\n", status);
  sleep(5);
}

void GetAndPrintScan()
{
  int status;
  std::array<double, END_CHANNEL - START_CHANNEL + 1> Voltages;
  status = AIOUSB::ADC_SetScanLimits(Device, START_CHANNEL, END_CHANNEL);

  status = AIOUSB::ADC_SetOversample(Device, 5);

  status = AIOUSB::ADC_GetScanV(Device, Voltages.data());

  for (int i = 0 ; i <= END_CHANNEL - START_CHANNEL ; i++)
  {
    printf("%d: %f\n", i, Voltages[i]);
  }

}

void DioTest()
{
  int Status;
  uint32_t DioBytes;

  Status = AIOUSB::QueryDeviceInfo(Device, nullptr, nullptr, nullptr, &DioBytes, nullptr);

  printf("%s: Status = %d, DioBytes = %d\n", __FUNCTION__, Status, DioBytes);

  std::array<uint8_t, 8> DioData;
  std::array<uint8_t, 8> OutMask;

  std::fill(OutMask.begin(), OutMask.end(), 0xf);


  AIOUSB::DIO_Configure(Device, 0, OutMask.data(), DioData.data());

  printf("Setting DIO0 high\n");
  Status = AIOUSB::DIO_Write1(Device, 0, 1);
  sleep(3);

  printf("Setting DIO0 low\n");
  Status = AIOUSB::DIO_Write1(Device, 0, 0);
  sleep(3);

  printf("Setting DIO0-7 high\n");
  AIOUSB::DIO_Write8(Device, 0, 0xf);
  sleep(8);

  printf("Setting DIO0-7 low\n");
  AIOUSB::DIO_Write8(Device, 0, 0);
  sleep(8);

  std::fill(DioData.begin(), DioData.end(), 0xff);
  printf("setting DIO0-15 high\n");
  AIOUSB::DIO_WriteAll(Device, DioData.data());
  sleep(8);

  std::fill(DioData.begin(), DioData.end(), 0);
  printf("setting DIO0-15 low\n");
  AIOUSB::DIO_WriteAll(Device, DioData.data());
  sleep(8);

}

int main (int argc, char **argv)
{
  int Status;

  AIOUSB::AiousbInit();

  Status = SampleGetDeviceHandle(argc, argv, &Device);

  if (Status)
  {
    std::cout << "Unable to open device" << std::endl;
    exit(-1);
  }
  AIOUSB::ADC_SetCal(Device, ":NONE:");

  GetAndPrintScan();

    AIOUSB::ADC_SetCal(Device, ":NORM:");

  GetAndPrintScan();

  AIOUSB::ADC_SetCalAndSave(Device, ":AUTO:", "outtest.bin");
  GetAndPrintScan();

  DioTest();
  DacDirectTest();
}

#include <unistd.h>
#include <getopt.h>
#include "aiousb.h"
#include "sample_helpers.inc"

#include "AiousbSamples.inc"
#include <stdlib.h>

#define dbg_print(fmt, ...)                     \
  do                                            \
  {                                             \
    printf("%s:%d:%s(): \n" fmt, __FILE_NAME__, \
           __LINE__, __func__, ##__VA_ARGS__);  \
  } while (0)


#define CONFIG_SIZE 21
#define START_CHANNEL 0
#define END_CHANNEL 15
#define FREQUENCY 10000.0
#define OVERSAMPLE 3
#define SAMPLE_SIZE 2
#define BUFFER_SIZE 512 * SAMPLE_SIZE *(OVERSAMPLE + 1) * (END_CHANNEL - START_CHANNEL + 1)
#define DURATION 600

struct option long_options[] = {
    {"help", no_argument, nullptr, 'h'},
    {"differential", no_argument, nullptr, 'd'},
    {"oversample", required_argument, nullptr, 'o'},
    {"frequency", required_argument, nullptr, 'f'},
    {"duration", required_argument, nullptr, 't'},
    {"range", required_argument, nullptr, 'r'},
    {"start-channel", required_argument, nullptr, 's'},
    {"end-channel", required_argument, nullptr, 'e'},
    {nullptr, 0, nullptr, 0}};

struct options
{
  bool Differential;
  int Oversample;
  double Frequency;
  double FrequencyActual;
  int Duration;
  int Range; // range code
  int StartChannel;
  int EndChannel;
};

struct CbContext
{
  int StartChannel;
  int EndChannel;
  int Oversample;
  uint32_t ConfigSize;
  uint8_t Config[CONFIG_SIZE];
};

void Usage()
{
  std::cout << "Usage:" << std::endl;
  std::cout << "AiousbAdcContinuousSample [--differential] [--oversample <oversample>] [--frequency <frequency>] [--duration <duration>] [--range <range-code>] [<device_file>] ";
  std::cout << "[--start-channel <start-channel>] [--end-channel <end-channel>] " << std::endl;
  std::cout << "AiousbAdcContinuousSample [-d] [-o <oversample>] [-f <frequency>] [-t <duration>] [-r <range-code>] [-s <start-channel>] [-e <end-channel>] [<device_file>]" << std::endl;
  std::cout << "Range codes:" << std::endl;
  std::cout << "\t0: 0 to 10V" << std::endl;
  std::cout << "\t1: -10 to 10V" << std::endl;
  std::cout << "\t2: 0 to 5V" << std::endl;
  std::cout << "\t3: -5 to 5V" << std::endl;
  std::cout << "\t4: 0 to 2V" << std::endl;
  std::cout << "\t5: -2 to 2V" << std::endl;
  std::cout << "\t6: 0 to 1V" << std::endl;
  std::cout << "\t7: -1 to 1V" << std::endl;
  std::cout << "If no device file is specified and only one AIOUSB device is detected that lone device will be used" << std::endl;
}

int parse_options(int argc, char **argv, struct options *Options)
{
  int opt;
  int opt_count = 0;

  Options->Differential = false;
  Options->Oversample = OVERSAMPLE;
  Options->Frequency = FREQUENCY;
  Options->FrequencyActual = FREQUENCY; // Placeholder, actual frequency will be set later
  Options->Duration = DURATION;
  Options->Range = 0;
  Options->StartChannel = START_CHANNEL;
  Options->EndChannel = END_CHANNEL;

  while ((opt = getopt_long(argc, argv, "dho:f:t:", long_options, nullptr)) != -1)
  {
    switch (opt)
    {
    case 'd':
      Options->Differential = true;
      break;
    case 'o':
      Options->Oversample = atoi(optarg);
      opt_count++;
      break;
    case 'f':
      Options->Frequency = atof(optarg);
      opt_count++;
      break;
    case 't':
      Options->Duration = atoi(optarg);
      opt_count++;
      break;
    case 'r':
      Options->Range = atoi(optarg);
      opt_count++;
      break;
    case 's':
      Options->StartChannel = atoi(optarg);
      opt_count++;
      break;
    case 'e':
      Options->EndChannel = atoi(optarg);
      opt_count++;
      break;
    case 'h':
    default:
      Usage();
      exit(0);
    }
    opt_count++;
  }
  return opt_count;
}

void print_options(const struct options *Options)
{
  std::cout << "Options:" << std::endl;
  std::cout << "Differential: " << (Options->Differential ? "true" : "false") << std::endl;
  std::cout << "Oversample: " << Options->Oversample << std::endl;
  std::cout << "Frequency: " << Options->Frequency << std::endl;
  std::cout << "Duration: " << Options->Duration << " seconds" << std::endl;
  std::cout << "Range: " << Options->Range << std::endl;
  std::cout << "Start Channel: " << Options->StartChannel << std::endl;
  std::cout << "End Channel: " << Options->EndChannel << std::endl;
}

void ADContCallback(uint16_t *buff, uint32_t buff_size,
                    uint32_t flags, void *context);

int main(int argc, char **argv)
{
  int Status;
  AIOUSB::aiousb_device_handle Device;
  uint32_t Pid;
  int ChannelCount;
  uint64_t SerialNum;
  struct CbContext Context = {0};
  uint32_t NameSize = 255;
  char Name[NameSize];
  int opt_count = 0;

  struct options Options = {};

  opt_count = parse_options(argc, argv, &Options);
  print_options(&Options);

  std::cout << "ACCES AIOUSB-Linux ADC sample" << std::endl;

  AIOUSB::AiousbInit();

  // Try to get a device handle.
  Status = SampleGetDeviceHandle(argc - opt_count, &argv[opt_count], &Device);

  if (0 != Status)
  {
    std::cout << "Unable to get device handle" << std::endl;
    return -1;
  }

  AIOUSB::QueryDeviceInfo(Device, &Pid, &NameSize, Name, nullptr, nullptr);
  AIOUSB::GetDeviceSerialNumber(Device, &SerialNum);
  std::cout << Name << " detected [" << std::hex << Pid << std::dec << "]" << std::endl;
  std::cout << "Serial Number: " << std::hex << SerialNum << std::dec << std::endl;

  Context.ConfigSize = CONFIG_SIZE;

  Status = AIOUSB::ADC_GetConfig(Device, Context.Config, &Context.ConfigSize);

  if (Status)
  {
    std::cout << "Error getting ADC config: " << Status << std::endl;
    return -1;
  }
  // There isn't a convenience function to set the counter and trigger mode
  // The configuration buffer is documented in the API reference manual
  // https://accesio.com/MANUALS/USB%20Software%20Reference%20Manual.html
  Context.Config[0x11] = 0x05; // trigger & counter mode

  Status = AIOUSB::ADC_SetConfig(Device, Context.Config, &Context.ConfigSize);

  if (Status)
  {
    std::cout << "Error setting ADC config: " << Status << std::endl;
    return -1;
  }

  // The context is passed to the callback

  Context.StartChannel = Options.StartChannel;
  Context.EndChannel = Options.EndChannel;

  Status = AIOUSB::ADC_SetScanLimits(Device, Context.StartChannel, Context.EndChannel);

  if (Status)
  {
    std::cout << "Error setting scan limits: " << Status << std::endl;
    return -1;
  }

  Context.Oversample = Options.Oversample;

  Status = AIOUSB::ADC_SetOversample(Device, OVERSAMPLE);

  if (Status)
  {
    std::cout << "Error setting oversample: " << Status << std::endl;
    return -1;
  }

  Status = AIOUSB::CTR_8254StartOutputFreq(Device, 0, &Options.FrequencyActual);

  if (Status)
  {
    printf("Error starting countern");
    return -1;
  }

  std::cout << "Set Frequency: " << Options.Frequency << " Actual: " << Options.FrequencyActual << std::endl;

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
    sleep(Options.Duration);
  }

  AIOUSB::ADC_BulkContinuousEnd(Device);

  sleep(1);
}

double VoltsFromCounts(int counts, int range_code)
{
  double v = counts * (1.0 / 65536.0);

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

void ADContCallback(uint16_t *buff, uint32_t buff_size,
                    uint32_t flags, void *context)
{
  struct CbContext *Context = (struct CbContext *)context;
  static int call_count = 0;

  std::cout << "ADContCallback called with buff_size: " << buff_size
            << ", flags: " << flags << ", context: " << context << std::endl;
  std::cout << "Call count: " << ++call_count << std::endl;

  if (flags & ADC_CONT_CALLBACK_FLAG_END_STREAM)
  {
    std::cout << "End of stream flag set" << std::endl;
    exit(0);
  }

  for (int i = Context->StartChannel; i < Context->EndChannel; i++)
  {
    double volts;
    uint16_t counts;
    counts = 0;
    for (int j = 0; j < Context->Oversample + 1; j++)
    {
      int n = j + 1;
      counts = counts * (n - 1) / n + (buff[i * Context->Config[0x13] + j]) / n;
    }
    volts = VoltsFromCounts(counts, Context->Config[i]);
    printf("Channel: %d, Volts: %f\n", i, volts);
  }
}

bool SampleGetDeviceHandle (int argc, char **argv, aiousb_device_handle *Device)
{
  if ( 1 == argc )
  {
    return aiousb_device_handle_by_index(diOnly, Device);
  }
  else
  {
    return aiousb_device_handle_by_path(argv[1], Device);
  }

}

void SampleUsage (int argc, char **argv)
{
  std::cout << "Usage:" << std::endl;
  std::cout << argv[0] << " [/dev/accesio/<device_file>]" << std::endl;
  std::cout << "If no device file is specified and only one AIOUSB device is detected that lone device will be used" << std::endl;
}
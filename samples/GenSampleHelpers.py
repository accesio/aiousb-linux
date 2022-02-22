#!/usr/bin/env python3

import sys


import sys
import os

from array import array

def GenHelper (Name, ParamName, List):

  print(str.format("int {0}(uint32_t {1})", Name, ParamName))
  print("{");
  print(str.format("\tswitch({0})", ParamName))
  print("\t{")
  for param, count in List:
    print(str.format("\tcase {0}: return {1};", param, count))
  print("\tdefault: return 0;")
  print("\t};")
  print("}")


if __name__ == "__main__":

#Generates helper functions for the samples

  CurrentPid = 0
  CurrentDacs = 0
  CurrentAdcImm = 0
  CurrentAdcChan = 0

  DacsList = list()
  AdcsImmList = list()
  AdcMuxChanList = list()

  print (sys.argv[1])

  with open(str.format("{0}/../include/accesio_usb_ids.h", sys.argv[1])) as fp:
    line = fp.readline()
    while line.find("acces_usb_device_table") == -1:
      line = fp.readline()
    while line:
      if line.find("pid_loaded") != -1:
        if CurrentAdcImm != 0:
          AdcsImmList.insert(0, (CurrentPid, CurrentAdcImm))
          CurrentAdcImm = 0;
        if CurrentDacs != 0:
          DacsList.insert(0, (CurrentPid, CurrentDacs))
          CurrentDacs = 0
        if CurrentAdcChan != 0:
          AdcMuxChanList.insert(0, (CurrentPid, CurrentAdcChan))
        CurrentPid = line.split()[2].strip(",")
      elif line.find("imm_dacs") != -1:
        CurrentDacs = int(line.split()[2].strip(","))
      elif line.find("imm_adcs") != -1:
        CurrentAdcImm = int(line.split()[2].strip(","))
      elif line.find("adc_mux_channels") != -1:
        CurrentAdcChan = int(line.split()[2].strip(","))

      line = fp.readline()

  stdout_bak = sys.stdout;
  sys.stdout = open(str.format("{0}/sample_helpers.inc", sys.argv[1]), 'w')


  print("\n\n")
  print("//There is no API defined to get the number of DACs or the number of ADCs like");
  print("//there is for the DIO Bytes and Counters. For convenience these helper");
  print("//functions are generated from the header file the driver uses for information.");
  print("//They are used by samples to make them a little more flexible");
  print("//They are not intended for production use, but since they are by PID they will");
  print("//always be backwards compatible.");


  GenHelper("GetNumAdcsImm", "Pid", AdcsImmList)
  GenHelper("GetNumDacs", "Pid", DacsList)
  GenHelper("GetNumAdcMuxChan", "Pid", AdcMuxChanList)

  sys.stdout.close();
  sys.stdout = stdout_bak;





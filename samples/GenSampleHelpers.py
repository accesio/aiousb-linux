#!/usr/bin/env python3

import sys


import sys
import os

from array import array


if __name__ == "__main__":

#Generates helper functions for the samples

  CurrentPid = 0
  CurrentDacs = 0
  CurrentAdcs = 0

  DacsList = list()
  AdcsList = list()

  print (sys.argv[1])

  with open(str.format("{0}/../include/accesio_usb_ids.h", sys.argv[1])) as fp:
    line = fp.readline()
    while line.find("acces_usb_device_table") == -1:
      line = fp.readline()
    while line:
      if line.find("pid_loaded") != -1:
        if CurrentAdcs != 0:
          AdcsList.insert(0, (CurrentPid, CurrentAdcs))
          CurrentAdcs = 0;
        if CurrentDacs != 0:
          DacsList.insert(0, (CurrentPid, CurrentDacs))
          CurrentDacs = 0
        CurrentPid = line.split()[2].strip(",")
      elif line.find("imm_dacs") != -1:
        CurrentDacs = int(line.split()[2].strip(","))
      elif line.find("imm_adcs") != -1:
        CurrentAdcs = int(line.split()[2].strip(","))
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


  print("int GetNumAdcs(uint32_t Pid)")
  print("{");
  print("\tswitch(Pid)")
  print("\t{")
  for pid, count in AdcsList:
    print(str.format("\tcase {0}: return {1};", pid, count))
  print("\tdefault: return 0;")
  print("\t};")
  print("}")

  print ("int GetNumDacs(uint32_t Pid)")
  print ("{");
  print ("\tswitch(Pid)")
  print ("\t{")
  for pid, count in DacsList:
    print (str.format("\tcase {0}: return {1};", pid, count))
  print("\tdefault:return 0;")
  print("\t};")
  print("}")

  sys.stdout.close();
  sys.stdout = stdout_bak;





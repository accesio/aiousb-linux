import ctypes

_handle = ctypes.CDLL('/usr/local/lib/libaiousb.so')

def AiousbInit():
  return _handle.AiousbInit()

def ADC_SetScanLimits(index, start, end):
  return _handle.ADC_SetScanLimits(ctypes.c_ulong(index), ctypes.c_int(start), ctypes.c_int(end))

def ADC_SetOversample(index, oversample):
  return _handle.ADC_SetOversample(ctypes.c_ulong(index), ctypes.c_uint8(oversample))

def ADC_GetScanV(index, channels):
  retval = (ctypes.c_double * channels)()
  status = _handle.ADC_GetScanV(ctypes.c_ulong(index), retval)
  return (status, retval)


if __name__ == "__main__":
  status = AiousbInit()
  print ("AiousbInit returned %d" % status)
  status = ADC_SetScanLimits(0, 0, 15)
  print ("ADC_SetScanLimits return %d" %status)
  status = ADC_SetOversample(0, 5)
  print( "ADC_SetOversample returned %d" %status)
  (status, voltages) = ADC_GetScanV(0, 16)
  print("ADC_GetScanV returned %d" %status)
  for x in voltages:
    print("%f", x)

import ctypes

_handle = ctypes.CDLL('./libaiousb.so')

def aiousb_init():
  return _handle.aiousb_init()

def aiousb_set_scan_limits(index, start, end):
  return _handle.aiousb_set_scan_limits(ctypes.c_ulong(index), ctypes.c_int(start), ctypes.c_int(end))

def aiousb_adc_set_oversample(index, oversample):
  return _handle.aiousb_adc_set_oversample(ctypes.c_ulong(index), ctypes.c_uint8(oversample))

def aiousb_get_scan_v(index, channels):
  retval = (ctypes.c_double * channels)()
  status = _handle.aiousb_get_scan_v(ctypes.c_ulong(index), retval)
  return (status, retval)


if __name__ == "__main__":
  status = aiousb_init()
  print ("init returned %d" % status)
  status = aiousb_set_scan_limits(0, 0, 15)
  print ("aiousb_set_scan_limits return %d" %status)
  status = aiousb_adc_set_oversample(0, 5)
  print( "aiousb_adc_set_oversample returned %d" %status)
  (status, voltages) = aiousb_get_scan_v(0, 16)
  print("aiousb_get_scan_v returned %d")
  for x in voltages:
    print("%f", x)

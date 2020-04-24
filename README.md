To build

```
mkdir build
cd build
cmake ..
make
```

To load driver into kernel

```
sudo insmod accesio_usb.ko
```


Basic DIO test located in test directory aiousb-dio-test.cpp

Replace filename in the call to aiousb_device_open with the one created by your device.

The functions to talk to the device are in aiousb.h, and they mirror the api documented in the USB Software Reference Manual.
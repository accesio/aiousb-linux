# aiousb-linux

aiousb is the Linux implementation of the [ACCES I/O USB API](https://accesio.com/MANUALS/USB%20Software%20Reference%20Manual.html)
as a linux kernel module and library.

## Install needed packages

### Deb type
```bash
apt install git cmake build-essential libudev-dev python3
```

### Red Hat type
```bash
yum install git cmake python3-devel libudev-devel
```
## Obtaining the source

```bash
git clone https://github.com/accesio/aiousb-linux
```

## Build and install

```bash
mkdir build
cd build
cmake ..
make
sudo make install
```


## Setting device permissions on module load
By default the device file requires root permissions to access. To change the default permissions for the device pass `dev_mode` to the module as a parameter. The mode can be passed in octal similar to `chmod` command
* To set using insmod
```sh
insmod accesio_usb.ko dev_mode=0666
```
* To set for boot
  * Create a file /etc/modprobe.d/accesio-usb.conf
  ```
  options accesio_usb dev_mode=0666
  ```


# Using the library
To use the library include aiousb.h in your program. The first call into the library must be `AiousbInit()`. All of the functions are documented in the [ACCES I/O USB API](https://accesio.com/MANUALS/USB%20Software%20Reference%20Manual.html)


The source code in the samples directory is intended as a guide. The source code in the test directory is not intended for general use.

## Using the library with NO_HOTPLUG
It is possible to build the library without hotplug support by defining NO_HOTPLUG in the cmake command. This was done to address a known [Python issue](#hotplug-support-breaks-python)
```bash
cmake -DNO_HOTPLUG=y ..
```
When the library is built without hotplug support the behavior of calls to AiousbInit() after the first one will simulate a hotplug by checking for removed devices and then scanning for new ones. When the library is built with hotplug enabled (the default) then subsequent calls to AiousbInit() will return -EALREADY.


# Known issues
## Hotplug support breaks Python
Python support is limited, and in order to use it the cmake command must include -DNO_HOTPLUG when building the project. For more information on using the library in this mode see [here](#using-the-library-with-no_hotplug).

Attempting to use the library in Python when hotplug support is enabled result in a failure when attempting to load the library. The `ctypes.CDLL('./libaiousb.so')` call will cause the program to crash.

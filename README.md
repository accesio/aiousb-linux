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


# Using the library
To use the library include aiousb.h in your program. The first call into the library must be `AiousbInit()`. All of the functions are documented in the [ACCES I/O USB API](https://accesio.com/MANUALS/USB%20Software%20Reference%20Manual.html)


The source code in the samples directory is intended as a guide. The source code in the test directory is not intended for general use.

# Known issues
## Hotplug support breaks Python
Python support is limited, and in order to use it the cmake command must include -DNO_HOTPLUG when building the project. This means the devices that are detected during AiousbInit() will be what the library always thinks is there.
```bash
cmake -DNO_HOTPLUG=y ..
```

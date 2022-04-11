#This Makefile is expected to be used by cmake so it does not copy all of the
# .hex files to /lib/firmware. To see the complete list of files and where
#they will be placed see the install_manifest.txt that is genearted by cmake.
#To manually install the .hex files copy the .hex files in the ../fw directory
#to /lib/firmware on your system.
obj-m += accesio_usb.o
CC		?= gcc
KDIR 		?= /lib/modules/$(shell uname -r)/build
CFLAGS_accesio_usb.o=$(AIO_CFLAGS)

all:
	$(MAKE) CC=$(CC) -C $(KDIR) M=$(shell pwd) modules

clean:
	$(MAKE) CC=$(CC) -C $(KDIR) M=$(shell pwd) clean

install: all
	$(MAKE) CC=$(CC) -C $(KDIR) M=$(shell pwd) modules_install
	depmod -a

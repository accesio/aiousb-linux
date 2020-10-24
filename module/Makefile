CONFIG_MODULE_SIG = n
obj-m += accesio_usb.o
CC		:= gcc
KVERSION        := $(shell uname -r)
KDMOD		:= /lib/modules/$(KVERSION)
KDIR		:= $(KDMOD)/build
KFWDIR		:= /lib/firmware
CFLAGS_accesio_usb.o=$(AIO_CFLAGS)

all:
	$(MAKE) CC=$(CC) -C $(KDIR) M=$(shell pwd) modules
#	cp fw/*.hex $(KFWDIR)/

clean:
	$(MAKE) CC=$(CC) -C $(KDIR) M=$(shell pwd) clean
#	modprobe -r accesio_usb
#	rm -vf $(KDMOD)/accesio_usb.ko $(KDMOD)/extra/accesio_usb.ko
#	rm -vf $(KFWDIR)/ACCESIO-*.hex

install: all
	$(MAKE) CC=$(CC) -C $(KDIR) M=$(shell pwd) modules_install
#	cp accesio_usb.ko $(KDMOD)
	depmod -a
#	modprobe accesio_usb

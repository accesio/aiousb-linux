CONFIG_MODULE_SIG = n
obj-m += accesio_usb.o
CC		:= gcc
KVERSION        := $(shell uname -r)
KDMOD		:= /lib/modules/$(KVERSION)
KDIR		:= $(KDMOD)/build
KFWDIR		:= /lib/firmware

all:
	$(MAKE) CC=$(CC) -C $(KDIR) M=$(PWD) modules
#	cp fw/*.hex $(KFWDIR)/

clean: 
	$(MAKE) CC=$(CC) -C $(KDIR) M=$(PWD) clean
#	modprobe -r accesio_usb
#	rm -vf $(KDMOD)/accesio_usb.ko $(KDMOD)/extra/accesio_usb.ko
#	rm -vf $(KFWDIR)/ACCESIO-*.hex

install: all
	$(MAKE) CC=$(CC) -C $(KDIR) M=$(PWD) modules_install
	cp accesio_usb.ko $(KDMOD)
	depmod -a
	modprobe accesio_usb
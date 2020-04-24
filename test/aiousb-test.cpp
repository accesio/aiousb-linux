#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include "aiousb.h"

#define err_printf(fmt, ...) \
        do { printf ("%s:%d:%s(): \n" fmt, __FILE__, \
                                __LINE__, __func__, ##__VA_ARGS__); } while (0)


void read_serial_num(aiousb_device_handle device)
{
	long long serial_num = 0;
	unsigned long size = sizeof(serial_num);
	int status;

	status = aiousb_generic_vendor_read(device, 0xa2, 0x1df8, 0, size, &serial_num);

	if (status < 0)
	{
		err_printf("Error doing read: %d", status);
	}
	else
	{
		printf("Read serial num: 0x%llx\n", serial_num);
	}
}

int main(int argc, char **argv)
{
	int fd;
	int status;

	aiousb_device_handle device;
	if (argc != 2)
	{
		printf("usage: aiousb-test <filename>\n");
		return -1;
	}

	status = aiousb_device_open(argv[1], &device);

	if (status)
	{
		printf("Error opening device: %d\n", status);
	}

	read_serial_num(device);

	aiousb_device_close(device);

	return 0;
}
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>

#include "aiousb.h"

#define err_printf(fmt, ...) \
        do { printf ("%s:%d:%s(): \n" fmt, __FILE__, \
                                __LINE__, __func__, ##__VA_ARGS__); } while (0)

static bool terminating;

void read_serial_num(AIOUSB::aiousb_device_handle device)
{
	long long serial_num = 0;
	unsigned long size = sizeof(serial_num);
	int status;

	status = AIOUSB::GenericVendorRead(device, 0xa2, 0x1df8, 0, size, &serial_num);

	if (status < 0)
	{
		err_printf("Error doing read: %d", status);
	}
	else
	{
		printf("Read serial num: 0x%llx\n", serial_num);
	}
}

void read_serial_num(unsigned long device_index)
{
	long long serial_num = 0;
	unsigned long size = sizeof(serial_num);
	int status;

	status = AIOUSB::GenericVendorRead(device_index, 0xa2, 0x1df8, 0, size, &serial_num);

	if (status < 0)
	{
		err_printf("Error doing read: %d", status);
	}
	else
	{
		printf("Read serial num: 0x%llx\n", serial_num);
	}
}

//void sigint_handler(int sig)
//{
//	printf("SIGINT\n");
//	exit(0);
//}

int main(int argc, char **argv)
{
	int status;
	uint32_t device_mask;

	status = AIOUSB::AiousbInit();

	if (status != 0)
	{
		printf("aiousb_init() failed: %d\n", status);
		return -1;
	}

//	signal(SIGINT, sigint_handler);

	device_mask = 0;
	uint32_t pid ;
	uint32_t name_size;
	char name[64];

	uint64_t serial_number;

	while (!terminating)
	{
		device_mask = AIOUSB::GetDevices();
		printf("device_mask = 0x%x\n", device_mask);
		for (int i = 0; i < 32 ; i++)
		{
			if ((device_mask >> i) % 2)
			{
				name_size = sizeof(name);
				status = AIOUSB::QueryDeviceInfo(i, &pid, &name_size, name, nullptr, nullptr);
				if (!status)
				{
					AIOUSB::GetDeviceSerialNumber(i, &serial_number);
					printf("Device at index %d:\n", i);
					printf("\tPID: 0x%x\n", pid);
					printf("\tName: %s\n", name);
					printf("\tSerial: 0x%llx\n", serial_number);
				}
				else
				{
					printf("Device index reported unavailable device at index %d", i);
				}
			}
		}
		printf("press enter to rescan or ctrl-c to exit...\n"); getchar();
	};

	return 0;
}

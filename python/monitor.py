import time
import AIOUSB

if __name__ == "__main__":
    status = AIOUSB.AiousbInit()
    print("AiousbInit status:", status)
    if status != 0:
        exit(1)
    while 1:
        Devices = AIOUSB.GetDevices()
        print("Devices found:", Devices)
        # Using the aiousb library with Python requires that aiousb be built 
        # with the NO_HOTPLUG. When aiousb is build with NO_HOTPLUG, the call
        # to Aiousb.AiousbInit() can be repeated to simulate hotplug.
        status = AIOUSB.AiousbInit()
        print("AiousbInit status:", status)
        time.sleep(1)

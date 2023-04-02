* in scan_devices just always go to the maximum device instead of using aiousb_device_count
* In check_removed just go to maximum devvice instead of device_count
* For AiousbInit. Need a find_available_index() instead of device_count
* DeviceHandleByPath() maximum devices
* aiousb_handle_by_index_private(): 
* DeviceIndexByPath() maximum devices
* GetDevices() maximum devices
* need an is_device_index_valid() function
* need to test NO_HOTPLUG and init as hotplug
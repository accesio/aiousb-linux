set (AIOUSB_INCLUDES accesio_usb_ids.h accesio_usb_ioctl.h aiousb.h timespec-util.h)

install (FILES ${AIOUSB_INCLUDES} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} )

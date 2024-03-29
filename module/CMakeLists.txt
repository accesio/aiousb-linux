#https://cmake.org/pipermail/cmake/2007-August/015891.html

Set( MODULE_OUTPUT_FILES    accesio_usb.o
                            accesio_usb.mod.c
                            accesio_usb.mod.o
                            accesio_usb.ko
                            .accesio_usb.o.cmd
                            .accesio_usb.ko.cmd
                            .accesio_usb.mod.o.cmd
                            .tmp_versions/accesio_usb.mod
                            .tmp_versions/ )

Set( MODULE_SOURCE_FILES    accesio_usb.c )

Set( DRIVER_FILE        accesio_usb.ko )
Set( DRIVER_TARGET_NAME accesio_usb-module )
Set( DRIVER_BIN_FILE    ${PROJECT_BINARY_DIR}/${DRIVER_FILE} )
Set( MODULE_SOURCE_DIR  ${PROJECT_SOURCE_DIR}/module )

if (AIO_DEBUG)
  set (AIO_CFLAGS "-DAIO_DEBUG=1")
else()
  set (AIO_CFLAGS "-DAIO_DEBUG=0")
endif (AIO_DEBUG)

if (KDIR)
Set( KERNEL_DIR ${KDIR} )
else()
Set( KERNEL_DIR "/lib/modules/${CMAKE_SYSTEM_VERSION}/build" )
endif (KDIR)


Set( KBUILD_CMD ${CMAKE_MAKE_PROGRAM}
                -C ${KERNEL_DIR}
                AIO_CFLAGS=${AIO_CFLAGS}
                M=${MODULE_SOURCE_DIR}
                modules )

Add_Custom_Command( OUTPUT  ${DRIVER_BIN_FILE}
                            ${MODULE_OUTPUT_FILES}
                    COMMAND ${KBUILD_CMD}
                    COMMAND cp -f ${DRIVER_FILE} ${PROJECT_BINARY_DIR}/module/${DRIVER_FILE}
                    WORKING_DIRECTORY ${MODULE_SOURCE_DIR}
                    DEPENDS ${MODULE_SOURCE_FILES} VERBATIM )

Add_Custom_Target ( ${DRIVER_TARGET_NAME}
                    ALL
                    DEPENDS ${DRIVER_BIN_FILE} )

set (KBUILD_INSTALL_CMD ${KBUILD_CMD} modules_install)
set (DEPMOD depmod -a)

install (CODE "execute_process(COMMAND ${KBUILD_INSTALL_CMD})"
         CODE "execute_process(COMMAND ${DEPMOD})")
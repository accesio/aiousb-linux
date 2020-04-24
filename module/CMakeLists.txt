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
Set( DRIVER_BIN_FILE    ${CMAKE_BINARY_DIR}/${DRIVER_FILE} )
Set( MODULE_SOURCE_DIR  ${PROJECT_SOURCE_DIR}/module )

Set( KERNEL_DIR "/lib/modules/${CMAKE_SYSTEM_VERSION}/build" )
Set( KBUILD_CMD ${CMAKE_MAKE_PROGRAM}
                -C ${KERNEL_DIR}
                M=${MODULE_SOURCE_DIR}
                modules )

Add_Custom_Command( OUTPUT  ${DRIVER_BIN_FILE}
                            ${MODULE_OUTPUT_FILES}
                    COMMAND ${KBUILD_CMD}
                    COMMAND cp -f ${DRIVER_FILE} ${DRIVER_BIN_FILE}
                    WORKING_DIRECTORY ${MODULE_SOURCE_DIR}
                    DEPENDS ${MODULE_SOURCE_FILES} VERBATIM )

Add_Custom_Target ( ${DRIVER_TARGET_NAME}
                    ALL
                    DEPENDS ${DRIVER_BIN_FILE} )

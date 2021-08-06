include_directories(${AIOUSB_SOURCE_DIR}/include)

add_executable(bulk-test bulk-test.c)
add_executable(bulk-abort-test bulk-abort-test.c)
target_link_libraries(bulk-abort-test pthread)

add_executable(read-serial read-serial.c)

set(AIOUSB_LIBS m pthread udev)

add_executable(aiousb-test aiousb-test.cpp)
target_link_libraries(aiousb-test ${AIOUSB_BINARY_DIR}/aiousb/libaiousb.a ${AIOUSB_LIBS})

add_executable(aiousb-iiro-test aiousb-iiro-test.cpp)
target_link_libraries(aiousb-iiro-test ${AIOUSB_BINARY_DIR}/aiousb/libaiousb.a ${AIOUSB_LIBS})

add_executable(aiousb-dio-test aiousb-dio-test.cpp)
target_link_libraries(aiousb-dio-test ${AIOUSB_BINARY_DIR}/aiousb/libaiousb.a ${AIOUSB_LIBS})

Add_Custom_Target(aiousb-dio-32-test ALL COMMAND ${CMAKE_COMMAND} -E create_symlink aiousb-dio-test aiousb-dio32-test)
Add_Custom_Target(aiousb-dio-32i-test ALL COMMAND ${CMAKE_COMMAND} -E create_symlink aiousb-dio-test aiousb-dio32i-test)
Add_Custom_Target(aiousb-dio-96-test ALL COMMAND ${CMAKE_COMMAND} -E create_symlink aiousb-dio-test aiousb-dio96-test)

add_executable(offset-test offset-test.c)

add_executable(aiousb-adc-test aiousb-adc-test.cpp)
target_link_libraries(aiousb-adc-test ${AIOUSB_BINARY_DIR}/aiousb/libaiousb.a ${AIOUSB_LIBS})

add_executable(aiousb-adc-getscanv-test aiousb-adc-getscanv-test.cpp)
target_link_libraries(aiousb-adc-getscanv-test ${AIOUSB_BINARY_DIR}/aiousb/libaiousb.a ${AIOUSB_LIBS})

add_executable(aiousb-continuous-acquisition aiousb-continuous-acquisition.cpp)
target_link_libraries(aiousb-continuous-acquisition ${AIOUSB_BINARY_DIR}/aiousb/libaiousb.a ${AIOUSB_LIBS})

add_executable(aiousb-adc-getfastscanv-test aiousb-adc-getfastscanv-test.cpp)
target_link_libraries(aiousb-adc-getfastscanv-test ${AIOUSB_BINARY_DIR}/aiousb/libaiousb.a ${AIOUSB_LIBS})

add_executable(hotplug-testing hotplug-testing.cpp)
target_link_libraries(hotplug-testing ${AIOUSB_BINARY_DIR}/aiousb/libaiousb.a ${AIOUSB_LIBS})

add_executable(aiousb-eeprom-test aiousb-eeprom-test.cpp)
target_link_libraries(aiousb-eeprom-test ${AIOUSB_BINARY_DIR}/aiousb/libaiousb.a ${AIOUSB_LIBS})
target_link_libraries(aiousb-eeprom-test ${CMAKE_BINARY_DIR}/aiousb/libaiousb.a)
target_link_libraries(aiousb-eeprom-test m)
target_link_libraries(aiousb-eeprom-test pthread)

add_executable(aiousb-c-linkage-test aiousb-c-linkage-test.c)
target_link_libraries(aiousb-c-linkage-test ${CMAKE_BINARY_DIR}/aiousb/libaiousb.a ${AIOUSB_LIBS})
target_link_libraries(aiousb-c-linkage-test stdc++)

add_executable(aiousb-aio16-16f-test aiousb-aio16-16f-test.cpp)
target_link_libraries(aiousb-aio16-16f-test ${AIOUSB_BINARY_DIR}/aiousb/libaiousb.a ${AIOUSB_LIBS})

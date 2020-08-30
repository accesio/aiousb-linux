include_directories(${AIOUSB_SOURCE_DIR}/include)

add_executable(bulk-test bulk-test.c)
add_executable(bulk-abort-test bulk-abort-test.c)
target_link_libraries(bulk-abort-test pthread)

add_executable(read-serial read-serial.c)

add_executable(aiousb-test aiousb-test.cpp)
target_link_libraries(aiousb-test ${CMAKE_BINARY_DIR}/aiousb/libaiousb.a)
target_link_libraries(aiousb-test m)
target_link_libraries(aiousb-test pthread)

add_executable(aiousb-dio-test aiousb-dio-test.cpp)
target_link_libraries(aiousb-dio-test ${CMAKE_BINARY_DIR}/aiousb/libaiousb.a)
target_link_libraries(aiousb-dio-test m)
target_link_libraries(aiousb-dio-test pthread)

add_executable(offset-test offset-test.c)

add_executable(aiousb-adc-test aiousb-adc-test.cpp)
target_link_libraries(aiousb-adc-test ${CMAKE_BINARY_DIR}/aiousb/libaiousb.a)
target_link_libraries(aiousb-adc-test m)
target_link_libraries(aiousb-adc-test pthread)

add_executable(aiousb-adc-getscanv-test aiousb-adc-getscanv-test.cpp)
target_link_libraries(aiousb-adc-getscanv-test ${CMAKE_BINARY_DIR}/aiousb/libaiousb.a)
target_link_libraries(aiousb-adc-getscanv-test m)
target_link_libraries(aiousb-adc-getscanv-test pthread)

add_executable(aiousb-continuous-acquisition aiousb-continuous-acquisition.cpp)
target_link_libraries(aiousb-continuous-acquisition ${CMAKE_BINARY_DIR}/aiousb/libaiousb.a)
target_link_libraries(aiousb-continuous-acquisition m)
target_link_libraries(aiousb-continuous-acquisition pthread)

add_executable(aiousb-adc-getfastscanv-test aiousb-adc-getfastscanv-test.cpp)
target_link_libraries(aiousb-adc-getfastscanv-test ${CMAKE_BINARY_DIR}/aiousb/libaiousb.a)
target_link_libraries(aiousb-adc-getfastscanv-test m)
target_link_libraries(aiousb-adc-getfastscanv-test pthread)

add_executable(hotplug-testing hotplug-testing.cpp)
target_link_libraries(hotplug-testing ${CMAKE_BINARY_DIR}/aiousb/libaiousb.a)
target_link_libraries(hotplug-testing m)
target_link_libraries(hotplug-testing pthread)
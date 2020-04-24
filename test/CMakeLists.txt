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


include_directories(${AIOUSB_SOURCE_DIR}/include)
add_library(aiousb STATIC aiousb.cpp adc-threads.cpp)
add_library(aiousb_shared SHARED aiousb.cpp adc-threads.cpp)

ADD_CUSTOM_COMMAND(TARGET aiousb_shared POST_BUILD
                    COMMAND ${CMAKE_COMMAND} -E rename
                    $<TARGET_FILE:aiousb_shared>
                    libaiousb.so)

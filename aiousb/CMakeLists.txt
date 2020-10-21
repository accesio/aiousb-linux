include_directories(${AIOUSB_SOURCE_DIR}/include)
add_library(aiousb STATIC aiousb.cpp adc-threads.cpp)
add_library(aiousb_shared SHARED aiousb.cpp adc-threads.cpp)

if (CMAKE_BUILD_TYPE MATCHES Debug OR AIOUSB_DEBUG)
  target_compile_definitions (aiousb PRIVATE AIOUSB_DEBUG=1)
  target_compile_definitions (aiousb_shared PRIVATE AIOUSB_DEBUG=1)
else ()
  target_compile_definitions (aiousb PRIVATE AIOUSB_DEBUG=0)
  target_compile_definitions (aiousb_shared PRIVATE AIOUSB_DEBUG=0)
endif (CMAKE_BUILD_TYPE MATCHES Debug OR AIOUSB_DEBUG)

ADD_CUSTOM_COMMAND(TARGET aiousb_shared POST_BUILD
                    COMMAND ${CMAKE_COMMAND} -E rename
                    $<TARGET_FILE:aiousb_shared>
                    libaiousb.so)

set (LIBRARY_FILES ${CMAKE_CURRENT_BINARY_DIR}/libaiousb.so ${CMAKE_CURRENT_BINARY_DIR}/libaiousb.a)

install (FILES ${LIBRARY_FILES} DESTINATION ${CMAKE_INSTALL_LIBDIR})


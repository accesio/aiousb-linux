include_directories(${AIOUSB_SOURCE_DIR}/include)

set(AIOUSB_LIBS ${AIOUSB_BINARY_DIR}/aiousb/libaiousb.a m pthread udev)

add_executable(AiousbDioSample AiousbDioSample.cpp)
target_link_libraries(AiousbDioSample ${AIOUSB_LIBS})

add_executable(AiousbDacSample AiousbDacSample.cpp)
target_link_libraries(AiousbDacSample ${AIOUSB_LIBS})

add_executable(AiousbIiroSample AiousbIiroSample.cpp)
target_link_libraries(AiousbIiroSample ${AIOUSB_LIBS})

add_executable(AiousbAdcSample AiousbAdcSample.cpp)
target_link_libraries(AiousbAdcSample ${AIOUSB_LIBS})

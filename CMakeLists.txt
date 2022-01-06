cmake_minimum_required(VERSION 2.8)

add_compile_options(-std=c++11)


project (AIOUSB)
add_compile_options(-Wall)
include(GNUInstallDirs)

set(AIOUSB_LIBS m pthread)
if (NOT NO_HOTPLUG)
  set(AIOUSB_LIBS ${AIOUSB_LIBS} "udev")
endif (NOT NO_HOTPLUG)


add_subdirectory(aiousb)
add_subdirectory(module)
add_subdirectory(test)
add_subdirectory(include)
add_subdirectory(fw)
add_subdirectory(samples)

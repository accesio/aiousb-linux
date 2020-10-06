set(CMAKE_CXX_STANDARD 11)
project (AIOUSB)
add_compile_options(-Wall)


add_subdirectory(aiousb)
add_subdirectory(module)
add_subdirectory(test)
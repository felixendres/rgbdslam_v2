include(ExternalProject)
message(${CMAKE_CURRENT_SOURCE_DIR}/SiftGPU/)
message(${CMAKE_CURRENT_BINARY_DIR}/siftgpu_prefix)
ExternalProject_Add(siftgpu_proj
  SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/SiftGPU/
  PREFIX ${CMAKE_CURRENT_BINARY_DIR}/siftgpu_prefix
  BUILD_IN_SOURCE 1
  CONFIGURE_COMMAND ""
  BUILD_COMMAND make
  INSTALL_COMMAND ""
)

add_library(siftgpu STATIC IMPORTED)
set_property(TARGET siftgpu PROPERTY IMPORTED_LOCATION  ${CMAKE_CURRENT_SOURCE_DIR}/SiftGPU/bin/libsiftgpu.so)
add_dependencies(siftgpu_proj siftgpu)

#add_executable(myapp main.c)

#include_directories(${install_dir}/include/luajit-2.0)

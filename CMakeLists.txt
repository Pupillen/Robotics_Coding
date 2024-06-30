cmake_minimum_required(VERSION 3.10)
project(MyProject)
set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
#!!!!!!!!!!sanity check for compiler flags!!!!!!!!!!
add_compile_options(
  -Wall
  -Wextra
  -Weffc++
  -Werror=uninitialized
  -Werror=return-type
  -Wconversion
  -Werror=unused-result
  -Werror=suggest-override
  -Wzero-as-null-pointer-constant
  -Wmissing-declarations
  -Wold-style-cast
  -Wnon-virtual-dtor
  )
  #!!!!!!!!!!sanity check for compiler flags!!!!!!!!!!

# Include directories
include_directories(${PROJECT_SOURCE_DIR}/include)
set(SOURCE_FILES
    src/main.cpp
)
add_executable(MyProject ${SOURCE_FILES})
# target_link_libraries(MyProject PRIVATE some_library)

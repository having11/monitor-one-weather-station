cmake_minimum_required (VERSION 3.2)
project (sensirion-test)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

set(CMAKE_C_STANDARD 11)

enable_testing()

# Global defines for all tests
add_definitions(-DLOG_DISABLE)
add_definitions(-DRELEASE_BUILD)
add_definitions(-DUNIT_TEST)

if (CMAKE_CXX_COMPILER_ID MATCHES GNU)
  set(GCOV_ENABLE TRUE)
  add_compile_options(-Wall -Wextra)
endif()

if (GCOV_ENABLE)
  set(COVERAGE_LIBRARIES gcov)
  set(COVERAGE_CFLAGS -fno-inline -fprofile-arcs -ftest-coverage -O0 -g)
endif()

include_directories(src/ test/)

add_executable(sensirion-test test/test.cpp src/Sht3x.cpp src/Sts3x.cpp src/SensirionBase.cpp test/Particle.cpp)

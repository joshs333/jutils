project(jutils)
cmake_minimum_required(VERSION 3.1)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(include)

add_executable(kdtrie_test src/kdtrie_test.cpp)
add_executable(logging_test src/logging_test.cpp)
add_executable(gpr_test src/gpr_test.cpp)
target_link_libraries(gpr_test Eigen3::Eigen)
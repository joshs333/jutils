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

# Set extra flags, be pedantic
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

### OPTIONS
option(ENABLE_MATPLOTLIB "enable usage of matplotlib for plotting and visualization" ON)
option(ENABLE_EIGEN "enable the usage of eigen as the linear algebra backbone" ON)

if(ENABLE_EIGEN)
  find_package(Eigen3 3.3 REQUIRED NO_MODULE)
endif()
if(ENABLE_MATPLOTLIB)
  set(PYTHON_INCLUDES /usr/include/python2.7)
  set(PYTHON_LIBRARIES python2.7)
endif()

include_directories(include ${PYTHON_INCLUDES})

add_executable(kdtrie_test src/kdtrie_test.cpp)
add_executable(logging_test src/logging_test.cpp)
# add_executable(threadpool_test src/threadpool_test.cpp)
add_executable(time_test src/time_test.cpp)
target_link_libraries(time_test pthread)

if(ENABLE_EIGEN)
  add_executable(sparse_gpr_test src/sparse_gpr_test.cpp)
  target_link_libraries(sparse_gpr_test Eigen3::Eigen ${PYTHON_LIBRARIES})
endif()
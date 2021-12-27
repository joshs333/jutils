#####################################################################
####################### High-Level CMake Setup ######################
#####################################################################
cmake_minimum_required(VERSION 3.1)
if (POLICY CMP0048)
    cmake_policy(SET CMP0048 NEW)
endif (POLICY CMP0048)

if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

# Be extra pedantic ;)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Includes for installation
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
#####################################################################
####################### Misc. Project Setup #########################
#####################################################################
project(jutils VERSION 0.1 LANGUAGES CXX)

### OPTIONS
# Disabling these options might disable certain features
option(JUTILS_ENABLE_MATPLOTLIB "enable usage of matplotlib for plotting and visualization" ON)
option(JUTILS_ENABLE_EIGEN "enable the usage of eigen as the linear algebra backbone" ON)
option(JUTILS_BUILD_TESTS "build and run tests (TODO: use actual testing framework lolz)" ON)

if(JUTILS_ENABLE_EIGEN)
  find_package(Eigen3 3.3 REQUIRED NO_MODULE)
endif()
if(JUTILS_ENABLE_MATPLOTLIB)
  set(PYTHON_INCLUDES /usr/include/python2.7)
  set(PYTHON_LIBRARIES python2.7)
endif()

### JUTILS Target Definitions
add_library(${PROJECT_NAME} INTERFACE)

target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_14)
target_include_directories(${PROJECT_NAME} INTERFACE
    $<BUILD_INTERFACE:${${PROJECT_NAME}_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}>
)
if(JUTILS_ENABLE_EIGEN)
    target_link_libraries(${PROJECT_NAME} INTERFACE Eigen3::Eigen)
endif()

#####################################################################
####################### Tests #######################################
#####################################################################
add_executable(kdtrie_test src/kdtrie_test.cpp)
target_link_libraries(kdtrie_test ${PROJECT_NAME})

add_executable(logging_test src/logging_test.cpp)
target_link_libraries(logging_test ${PROJECT_NAME})
# add_executable(threadpool_test src/threadpool_test.cpp)
add_executable(time_test src/time_test.cpp)
target_link_libraries(time_test ${PROJECT_NAME} pthread)

if(JUTILS_ENABLE_EIGEN)
    add_executable(sparse_gpr_test src/sparse_gpr_test.cpp)
    if(JUTILS_ENABLE_MATPLOTLIB)
        target_include_directories(sparse_gpr_test PRIVATE ${PYTHON_INCLUDES})
    endif()
    target_link_libraries(sparse_gpr_test Eigen3::Eigen ${PROJECT_NAME} ${PYTHON_LIBRARIES})
endif()

#####################################################################
####################### Installation ################################
#####################################################################
install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}_Targets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

write_basic_package_version_file(
    "${PROJECT_NAME}ConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
  "${PROJECT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in"
  "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  INSTALL_DESTINATION lib/cmake/jutils
)

install(EXPORT ${PROJECT_NAME}_Targets
    FILE ${PROJECT_NAME}Targets.cmake
    DESTINATION lib/cmake/jutils
)

install(
    FILES "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
        "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    DESTINATION lib/cmake/jutils
)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/jutils DESTINATION include)

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
    set(CMAKE_CXX_STANDARD 17)
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
option(JUTILS_BUILD_TESTS "build tests" ON)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)


if(ENABLE_EIGEN)
  find_package(Eigen3 3.3 REQUIRED NO_MODULE)
endif()
if(ENABLE_MATPLOTLIB)
  set(PYTHON_INCLUDES /usr/include/python2.7)
  set(PYTHON_LIBRARIES python2.7)
endif()

### JControls Target Definitions
add_library(${PROJECT_NAME} INTERFACE)

# We do this instead of compile_features() becase cxx_std_17 feature is not happy with CUDA
target_compile_options(${PROJECT_NAME} INTERFACE -std=c++17)
target_include_directories(${PROJECT_NAME} INTERFACE
    $<BUILD_INTERFACE:${${PROJECT_NAME}_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}>
    ${PYTHON_INCLUDES}
)
target_link_libraries(${PROJECT_NAME} INTERFACE Eigen3::Eigen)

#####################################################################
####################### Executables #################################
#####################################################################


#####################################################################
####################### Tests #######################################
#####################################################################
if(${JUTILS_BUILD_TESTS})
    add_executable(jutils_test_kdtrie src/kdtrie_test.cpp)
    target_link_libraries(jutils_test_kdtrie jutils)

    add_executable(jutils_test_logging src/logging_test.cpp src/logging.cpp)
    target_link_libraries(jutils_test_logging jutils)

    add_executable(jutils_test_threadpool src/threadpool_test.cpp)
    target_link_libraries(jutils_test_threadpool jutils)

    add_executable(jutils_test_time src/time_test.cpp)
    target_link_libraries(jutils_test_time pthread jutils)

    if(ENABLE_EIGEN)
        add_executable(jutils_test_sparse_gpr src/sparse_gpr_test.cpp)
        target_link_libraries(jutils_test_sparse_gpr Eigen3::Eigen jutils ${PYTHON_LIBRARIES})
    endif()
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
  INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
)

install(EXPORT ${PROJECT_NAME}_Targets
    FILE ${PROJECT_NAME}Targets.cmake
    DESTINATION lib/cmake/${PROJECT_NAME}
)

install(
    FILES "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
        "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    DESTINATION lib/cmake/${PROJECT_NAME}
)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME} DESTINATION include)

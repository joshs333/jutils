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
include(cmake/jutilsTargetFunctions.cmake)

#####################################################################
####################### Misc. Project Setup #########################
#####################################################################

### OPTIONS
# Disabling these options might disable certain features
option(JUTILS_ENABLE_PLOTTING "enable usage of matplot++ for plotting and visualization" ON)
option(JUTILS_ENABLE_EIGEN "disable the usage of eigen as the linear algebra backbone and any targets requiring it" ON)
option(JUTILS_ENABLE_CUDA "disable the usage of CUDA in any targets and don't build any CUDA tests / examples" ON)
option(JUTILS_BUILD_TESTS "build tests" ON)

if(${JUTILS_ENABLE_CUDA})
    project(jutils VERSION 0.1 LANGUAGES CXX CUDA)
else()
    project(jutils VERSION 0.1 LANGUAGES CXX)
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

# Eigen Linking
if(${JUTILS_ENABLE_EIGEN})
    find_package(Eigen3 3.3 REQUIRED NO_MODULE)
    target_link_libraries(${PROJECT_NAME} INTERFACE Eigen3::Eigen)
endif()

# Matplot++ Linking
if(${JUTILS_ENABLE_PLOTTING})
    find_package(Matplot++ REQUIRED QUIET)

    # Remove cxx_std_17 feature from matplot, this conflicts with CUDA versioning
    get_target_property(EXTLIB_COMPILE_FLAGS Matplot++::matplot INTERFACE_COMPILE_FEATURES)
    list(REMOVE_ITEM EXTLIB_COMPILE_FLAGS cxx_std_17)
    set_target_properties(Matplot++::matplot PROPERTIES
        INTERFACE_COMPILE_FEATURES "${EXTLIB_COMPILE_FLAGS}"
    )

    target_link_libraries(${PROJECT_NAME} INTERFACE Matplot++::matplot)
endif()

if(${JUTILS_BUILD_TESTS})
    include(GoogleTest)
    enable_testing()
endif()

#####################################################################
####################### Executables #################################
#####################################################################


#####################################################################
####################### Tests #######################################
#####################################################################
if(${JUTILS_BUILD_TESTS})
    if(${JUTILS_ENABLE_CUDA})
        jutils_add_test(cuda_interface tests/cuda_interface.cpp)
        jutils_add_test(shared_ptr tests/shared_ptr.cpp)
        jutils_add_test(array tests/array.cpp)
        jutils_add_test(multi_array tests/multi_array.cpp)
        jutils_add_test(tensor_array tests/tensor_array.cpp)
    endif()

    jutils_add_test_no_cuda(cuda_interface_no_cuda tests/cuda_interface_no_cuda.cpp)
    jutils_add_test_no_cuda(shared_ptr_no_cuda tests/shared_ptr_no_cuda.cpp)
    jutils_add_test_no_cuda(array_no_cuda tests/array_no_cuda.cpp)
    jutils_add_test_no_cuda(multi_array_no_cuda tests/multi_array_no_cuda.cpp)

    # These aren't unit tests... more examples really
    jutils_add_executable_no_cuda(jutils_test_kdtrie src/kdtrie_test.cpp)
    jutils_add_executable_no_cuda(jutils_test_logging src/logging_test.cpp)
    jutils_add_executable_no_cuda(jutils_test_time src/time_test.cpp)

    if(${JUTILS_ENABLE_EIGEN})
        jutils_add_executable_no_cuda(jutils_test_sparse_gpr src/sparse_gpr_test.cpp)
        jutils_add_executable_no_cuda(jutils_test_gpr src/gpr_test.cpp)
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

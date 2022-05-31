###
# @brief functions to help define targets that link against jutils
#   the reason this is helpful is that the project is header only and
#   can be included in cpp files, this helps handle marking any sources
#   that need CUDA
# NOTE: the default jutils INTERFACE target does not specify CUDA,
# conversely, the functions here all assume cuda (unless marked _no_cuda)
###

## Mark sources as using CUDA
function(jutils_cuda_sources)
    set_source_files_properties(${ARGN} PROPERTIES LANGUAGE CUDA)
endfunction()

## Mark target as using CUDA (some subset of sources uses CUDA)
function(jutils_cuda_target target_name)
    # supresses some warnings from Eigen
    set(NVCC_ARGS ${NVCC_ARGS} -Xcudafe "--diag_suppress=20012 --display_error_number")

    if(${JUTILS_FORCE_GCC9})
        set(NVCC_ARGS ${NVCC_ARGS} -ccbin /usr/bin/g++-9)
    endif()

    set_property(TARGET ${target_name} PROPERTY CUDA_SEPARABLE_COMPILATION ON)
    target_compile_options(${target_name} PUBLIC $<$<COMPILE_LANGUAGE:CUDA>:${NVCC_ARGS}>)
    target_compile_definitions(${target_name} PUBLIC JUTILS_ENABLE_CUDA)
endfunction()

## add executable that doesn't use cuda
## Normal add_executable + link jutils, set plotting defintions
function(jutils_add_executable_no_cuda target_name)
    add_executable(${target_name} ${ARGN})
    target_link_libraries(${target_name} jutils)

    if(${JUTILS_ENABLE_PLOTTING})
        target_compile_definitions(${target_name} PUBLIC JUTILS_ENABLE_PLOTTING)
    endif()
endfunction()

## Adds executable_no_cude, but marks all sources as CUDA & target as
function(jutils_add_executable target_name)
    jutils_add_executable_no_cuda(${target_name} ${ARGN})

    jutils_cuda_target(${target_name})
    jutils_cuda_sources(${ARGN})
endfunction()

### Create an executable, link against controls and gtest_main
function(jutils_add_test_no_cuda target_name)
    add_executable(jutils_test_${target_name} ${ARGN})
    target_link_libraries(jutils_test_${target_name} jutils gtest gtest_main pthread)
    gtest_discover_tests(jutils_test_${target_name})

    if(${JUTILS_ENABLE_PLOTTING})
        target_compile_definitions(jutils_test_${target_name} PUBLIC JUTILS_ENABLE_PLOTTING)
    endif()
endfunction()

### Create an executable, link against controls and gtest_main, mark sources as CUDA
function(jutils_add_test target_name)
    jutils_add_test_no_cuda(${target_name} ${ARGN})

    jutils_cuda_target(jutils_test_${target_name})
    jutils_cuda_sources(${ARGN})
endfunction()

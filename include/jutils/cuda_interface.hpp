/**
 * @file cuda_interface.hpp
 * @author Joshua Spisak (jspisak@andrew.cmu.edu)
 * @brief a collection of macros and enums to help simplify GPU-interopability and interfacing
 * @date 2022-05-31
 */
#ifndef JUTILS_CUDA_INTERFACE_HPP_
#define JUTILS_CUDA_INTERFACE_HPP_

#include <memory>
#include <vector>

#ifdef JUTILS_ENABLE_CUDA
    #include <cuda.h>
    #include <cuda_runtime.h>
#endif

//
// This is where any defines or behavior for GPU parallization
// is defined as well as any switches to disable GPU behavior
// and any utilities for managing CUDA interfacing
//

// friendly interpretation of whether this is running on GPU
#ifdef __CUDA_ARCH__
    #define ON_GPU true
#else
    #define ON_GPU false
#endif

// friendly interpretation of whether CUDA is available
// We only mark CUDA_AVAILABLE if enabled through cmake & __CUDACC__defined
#if defined JUTILS_ENABLE_CUDA
    #ifndef __CUDACC__
        #warning("JUTILS_ENABLE_CUDA is defined, but __CUDACC__ is not defined")
    #endif
    #define CUDA_AVAILABLE true
#else
    #define CUDA_AVAILABLE false
#endif

// Define tags properly if CUDA is available
#if CUDA_AVAILABLE
    //! Mark a function as GPU only
    #define GPU_ONLY __device__
    //! Mark a function as GPU comptaible
    #define GPU_COMPATIBLE __device__ __host__
#else
    #define GPU_ONLY #warning("CUDA is not available, but function is marked as GPU_ONLY")
    #define GPU_COMPATIBLE
#endif

//! top-level utils namespace
namespace jutils {

//! Types of memory
enum class MemType {
    //! Host memory can only be used from the CPU
    HOST,
    //! Device memory can only be used from the GPU or device
    DEVICE,
    //! Unified can be referenced from CPU or GPU and also allows
    //! over-subscription beyond GPU memory limits
    UNIFIED
};

} /* namespace jutils */


#endif /* JUTILS_CUDA_INTERFACE_HPP_ */
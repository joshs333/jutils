#include "jutils/cuda_interface.hpp"
#include "jutils_testing/interopable_test_runner.hpp"

#include <gtest/gtest.h>

namespace cuda_interface_test {

class TestClass {
public:
    using TestResult = int;
    using Arguments = int;

    __device__ __host__
    static void test(const int& arg, int& result) {
        #ifdef __CUDA_ARCH__
            result = 1;
        #else
            result = 2;
        #endif
    }
};

class TestClass2 {
public:
    using TestResult = int;
    using Arguments = int;

    GPU_COMPATIBLE
    static void test(const int& arg, int& result) {
        #if ON_GPU
            result = 1;
        #else
            result = 2;
        #endif
    }
};


} /* namespace cuda_interface_test */

TEST(DirectDefTest, deviceShouldBeOne) {
    jutils_testing::InteropableTestRunner<cuda_interface_test::TestClass> tr;
    ASSERT_EQ(1, tr.device(0));
}

TEST(DirectDefTest, hostShouldBeTwo) {
    jutils_testing::InteropableTestRunner<cuda_interface_test::TestClass> tr;
    ASSERT_EQ(2, tr.host(0));
}

TEST(CudaInterfaceTest, deviceShouldBeOne) {
    jutils_testing::InteropableTestRunner<cuda_interface_test::TestClass2> tr;
    ASSERT_EQ(1, tr.device(0));
}

TEST(CudaInterfaceTest, hostShouldBeTwo) {
    jutils_testing::InteropableTestRunner<cuda_interface_test::TestClass2> tr;
    ASSERT_EQ(2, tr.host(0));
}

TEST(CudaInterfaceTest, cudaIsAvailable) {
    ASSERT_TRUE(CUDA_AVAILABLE);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
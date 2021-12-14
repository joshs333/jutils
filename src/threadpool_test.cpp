#include <jutils/threadpool.hpp>
#include <cstdio>
#include <iostream>


int main(int argc, char** argv) {
    (void) argc;
    (void) argv;

    jutils::ThreadPool<int> tp(10);

    return 1;
}
#include <jutils/time.hpp>
#include <cstdio>
#include <iostream>
#include <jutils/logging.hpp>


int main(int argc, char** argv) {
    (void) argc;
    (void) argv;

    auto n = jutils::Clock::now();
    std::cout << n + 10s << std::endl;

    if( jutils::Clock::now() - n < 1ns) {
        std::printf("this is bad.\n");
    }
    if( jutils::Clock::now() - n > 40000ns) {
        std::printf("this is bad.\n");
    }

    // jutils::sleep(0.1);
    // std::printf("After 0.1\n");
    // jutils::sleep(2);
    // std::printf("After 2\n");
    // jutils::sleep(2ms);
    // std::printf("After 2ms\n");

    jutils::SystemTimer t(10ms);
    // t.setPeriod(10ms);

    for(int i = 0; i < 10; ++i) {
        JLOG_INFO("Here");
        jutils::sleep(5ms);
        t.sleep();
    }

    t.spawn_executor([]() {
        JLOG_INFO("Now here!");
        return true;
    });

    jutils::sleep(0.5);
    t.stop_executor();

    return 1;
}
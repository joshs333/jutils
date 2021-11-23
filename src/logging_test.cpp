#include <jutils/logging.hpp>

using jutils::logging::level;

int main(int argc, char** argv) {
    (void) argc;
    (void) argv;

    JLOG_WARN("Hello :)");

    return 0;
}
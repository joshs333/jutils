#include <jutils/logging.hpp>

using jutils::logging::level;

void print_all(std::string message) {
    std::printf("%s\n", message.c_str());
    JLOG_DIRE("Dire message!");
    JLOG_FAIL("Fail message!");
    JLOG_WARN("Warn message!");
    JLOG_INFO("Info message :)");
    JLOG_NOTE("Note message");
}


void print_all_named(std::string message) {
    std::printf("%s\n", message.c_str());
    JLOGN_DIRE(message, "Dire message!");
    JLOGN_FAIL(message,"Fail message!");
    JLOGN_WARN(message,"Warn message!");
    JLOGN_INFO(message,"Info message :)");
    JLOGN_NOTE(message,"Note message");
}

int main(int argc, char** argv) {
    (void) argc;
    (void) argv;

    jutils::logging::add_consumer("test_logger", jutils::logging::FileConsumer::newPtr("test", false));
    // jutils::logging::delete_consumer("stdout");
    // jutils::logging::get_consumer("stdout")->set_levels(level::MAX, level::MAX);

    print_all("printing all");
    // jutils::logging::set_levels(level::WARN, level::DIRE);
    print_all("log level warn");
    print_all("log level std fail");
    // jutils::logging::set_levels(level::NOTE, level::DIRE);
    print_all_named("log level INFO");

    std::printf("%3.9f\n", static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count())*1e-9);
    std::printf("%3.9f\n", static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count())*1e-9);
    // std::cout << std::chrono::high_resolution_clock::now() << std::endl;


    return 0;
}
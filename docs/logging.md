# Logging Utility
The logging utility is inteded to provide easy macros to log to different output mediums at different levels.

It is not as efficient as it could be (tested again ROS logging macros which it was a few microseconds slower than on average). In the context of these utilities I wanted to have some centralized interface I could use for the utilities and then improve on later, which this fulfills.

## Required Headers
- [logging.hpp](include/jutils/logging.hpp)

## Example Usage
Most of the utilities use this as the logging core. You can find examples scattered throughout. A more centralized example is here:
- [logging_test.cpp](src/logging_test.cpp)

## Usage Notes
### Logging Calls
There are 5 logging levels. They are named so they each are 4 letters.

- DIRE (5)
- FAIL (4)
- WARN (3)
- INFO (2)
- NOTE (1)

Typically the lowest level would be DEBUG and would be for low level information that would not go to stdout for use debugging issues. NOTE is intended to fulfill this purpose.

There are macros intended to be the interface:
```
JLOG_<LEVEL>("mesage or format", format args... );
// eg
JLOG_WARN("Got expected calling function: %s", e.what());
```
There is also the concept of named loggers which can be accesed with a different macro, eg:
```
JLOGN_WARN("named", "error message");
```
This results in a slightly different format when printing to stdout but otherwise is the same.

This is very lightweight (read: few features lol), there is no way to change the format.

### Logging Level / Consumers
There is the concept of a logging consumer which is the entity that takes a log and outputs it somewhere. There are two types of consumers currently implemented, the `StdoutConsumer` and the `FileConsumer`. By default a consumer of type `StdoutConsumer` with name `stdout` is added. Each consumer has a low and high logging level, it will process logs higher than or equal to the low leve and lower than the high level.

To set logging levels for a consumer use one of the following calls:
```
// Allow NOTE logs to pass to stdout
jutils::logging::get_consumer("stdout")->set_level_low(jutils::logging::level::NOTE);
// Don't allow DIRE logs to pass to stdout
jutils::logging::get_consumer("stdout")->set_level_high(jutils::logging::level::DIRE);
// Allow all logs to pass to stdout
jutils::logging::get_consumer("stdout")->set_levels(jutils::logging::level::NOTE, jutils::logging::level::MAX);
// Don't pass any logs to stdout
jutils::logging::get_consumer("stdout")->set_levels(jutils::logging::level::MAX, jutils::logging::level::MAX);
```

To add a consumer with a given name use the following calls:
```
// logs to a file named "test.txt", you can then call get_consumer("test_logger") to set the levels
// going to this file
jutils::logging::add_consumer("test_logger", jutils::logging::FileConsumer::newPtr("test", false));
```

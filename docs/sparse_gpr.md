# Spare Gaussian Process Regression
This utility implements gaussian process regression using a spare subset of data approach.
This is currently a work in progress... more to come later.

## Required headers
- [logging.hpp](../include/jutils/logging.hpp)
- [kdtrie.hpp](../include/jutils/kdtrie.hpp)
- [sparse_gpr.hpp](../include/jutils/sparse_gpr.hpp)

## Example Usage
- [sparse_gpr_test.cpp](../src/sparse_gpr_test.cpp): does some testing approximating a function.

## Usage Notes
### Imlementation
This will be rather sparse as this is a WIP - however, the TLDR is that the idea is we use a subset of data gaussian process which selects data dynamically based on the distance to the inference point. If the data is more relevant we consider it more. By cutting down the set of points being considered in an inference step we can take make each inference step take significantly less time.

### Logging
The macro `JUTILS_SPARSE_GPR_DEBUG_LOGGING` determines whether or not debug logging is enabled. To enable debug logging simply add the line `#define JUTILS_KDTREE_DEBUG_LOGGING` before you have `#include <jutils/sparse_gpr.hpp>` in your source. Note that the logging level is `NOTE` which does not go to stdout by default. See the logging documentation for how to set logging levels or output those logs to a file instead. WARNING: there is an extensive amout of logs that this enables which can add a significant amount of computation.

## External Dependencies
This uses `Eigen3` as the linear algebra backend.
```
sudo apt install libeigen3-dev
```
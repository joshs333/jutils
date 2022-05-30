# JUtils (Just Utils)
Common utils for copy-paste use in other projects. The utilities that applications will want to access should all be in the `jutils` namespace. There is also a `jutils::_internal` namespace which is used to segment out parts of the code that other applications shouldn't need to touch directly.

A high-level list of utilities is here, there is more extensive docs for each utilitie in the [docs/](docs/) folder.

## [Logging Utility](docs/logging.md)
This utility provides a slightly fancier logging interface than just printing to stdout. It has concepts of logging levels / logging to files rather than stdout. Is pretty simple and could use improvement but is good enough for the testing and profiling I want to do.

## [KDTrie Utility](docs/kdtrie.md)
This is intended to provide efficient spatial for nearest neighbor searches over different types of spatial data centered at a given point.

## [GPR Utility (WIP)](docs/sparse_gpr.md)
Gaussian Process Regression - essentially a data based function approximator.

## Build
While the utils are intended to be header-only, [cmake](https://cmake.org/) is used to build test / example scripts.

To build run:
```
mkdir build
cd build
cmake <options> ..
make
```

## Build Options
There are a few build options, mostly to enable / disable the usage of different libraries and the utils / tests that use them.
### [Eigen3](https://eigen.tuxfamily.org/index.php)
This is used as the linear algebra backend for the following:
- [GPR Utility (WIP)](docs/sparse_gpr.md)

To install run: `sudo apt install libeigen3-dev`

To disable add the following option to the cmake command: `-DENABLE_EIGEN=OFF`

### [Matplotlib-cpp](https://github.com/lava/matplotlib-cpp)
Some tests use matplotlib to plot resuts, specfically:
- [GPR Utility (WIP) Test](src/gpr_test.cpp)

To install run:
```
python2.7 python2.7-dev python-tk
sudo /bin/bash -c "curl https://bootstrap.pypa.io/pip/2.7/get-pip.py | python2.7"
sudo -H pip install matplotlib
```
To Disable add the following option to the cmake command: `-DENABLE_MATPLOTLIB=OFF`

## Doxygen
Documentation can automatically be generated using doxygen and the doxyfile (`doxygen .doxyfile`).
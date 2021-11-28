# JUtils (Just Utils)
Common utils for copy-paste use in other projects. The utilities that applications will want to access should all be in the `jutils` namespace. There is also a `jutils::_internal` namespace which is used to segment out parts of the code that other applications shouldn't need to touch directly.

A high-level list of utilities is here, there is more extensive docs for each utilitie in the [docs/](docs/) folder.

## [Logging Utility](docs/logging.md)
This utility provides a slightly fancier logging interface than just printing to stdout. It has concepts of logging levels / logging to files rather than stdout. Is pretty simple and could use improvement but is good enough for the testing and profiling I want to do.

## [KDTrie Utility](docs/kdtrie.md)
This is intended to provide efficient spatial for nearest neighbor searches over different types of spatial data centered at a given point.
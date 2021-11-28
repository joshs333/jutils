# KDTrie Spatial Utility
This is intended to function similar to a KDTree type data structure - efficient and fast . However it is more like a K-dimensional [octtree](https://en.wikipedia.org/wiki/Octree). When k=2 it behaves exactly like a quadtree, when k=3 it behaves exactly like an octtree.

## Required headers
- [logging.hpp](../include/jutils/logging.hpp)
- [kdtrie.hpp](../include/jutils/kdtrie.hpp)

## Example Usage
- [kdtrie_test.cpp](../src/kdtrie_test.cpp): does some profiling.

## Usage Notes
### Imlementation
The KDTrie can also be thought of as a spatial sorting mechanism - where points are sorted into spatially referenced containers of some base width. The default implementation stores a list of all points added in that base container (the KDTrieLeaf). Each point also has associated data of some type (the default is std::nullptr_t - no data). The default implementation is templated so that can be overrided - however it assumes that it is a pointer of some type and has the default value for associate arguments in the insert method be `nullptr`.

To override this behavior one can implement new leaf containers that respect the same interface as the base KDTrieLeaf container. What containers the KDTrie class uses internally is determined by the template parameters, which can be overwritten to use your Leaf Container. This can allow you to perform logic in the leaf container when new values are added and change how the inserted points are stored as well.

### Logging
The macro `JUTILS_KDTREE_DEBUG_LOGGING` determines whether or not [logging.hpp](../include/jutils/logging.hpp) is included and debug logging is enabled. To enable debug logging simply add the line `#define JUTILS_KDTREE_DEBUG_LOGGING` before you have `#include <jutils/kdtrie.hpp>` in your source. Note that the logging level is `NOTE` which does not go to stdout by default. See the logging documentation for how to set logging levels or output those logs to a file instead. WARNING: there is an extensive amout of logs that this enables which can add a significant amount of computation.


## Profiling Notes
Profiling with 10,000 points :
| Test | Time | Sorted Points |
| ------ | ---- |-----|
| Sort all points | 21ms | 10,000 |
| Iterate all points / select Closest | 0.9ms | 1 |
| KDTrie Closest Point | .3ms | 1 |
| KDTrie 10 Closest Points | .5ms | 10 |
| KDTrie Points within 1m | .8ms | 51 |

Profiling with 100,000 points :
| Test | Time | Sorted Points |
| ------ | ---- |-----|
| Sort all points | 224ms | 100,000 |
| Iterate all points / select Closest | 9.7ms | 1 |
| KDTrie Closest Point | .2ms | 1 |
| KDTrie 10 Closest Points | 0.5ms | 10 |
| KDTrie Points within 1m | 3.4ms | 383 |

Profiling with 1,000,000 points :
| Test | Time | Sorted Points |
| ------ | ---- |-----|
| Sort all points | 2,436ms | 1,000,000 |
| Iterate all points / select Closest | 91ms | 1 |
| KDTrie Closest Point | .2ms | 1 |
| KDTrie 10 Closest Points | 0.8ms | 10 |
| KDTrie Points within 1m | 21ms | 4096 |

As you can see from the above, closest point / k closest point operations take about the same time regardless of how many points there are. Sorting all points by distance and iterating all points & selecting the closest don't scale well.
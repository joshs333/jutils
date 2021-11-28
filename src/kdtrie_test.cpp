
// #define JUTILS_KDTREE_DEBUG_LOGGING
#include <jutils/kdtrie.hpp>

#include <algorithm>
#include <iostream>
#include <bitset>
#include <random>

// bool (const DistV& a, const DistV& b) {
//                 return a.distance < b.distance;
//             }

int main(int argc, char** argv) {
    (void) argc;
    (void) argv;


    // jutils::logging::add_consumer("test_logger", jutils::logging::FileConsumer::newPtr("test", false));
    // jutils::logging::get_consumer("stdout")->set_level_low(jutils::logging::level::NOTE);

    std::mt19937_64 rng_;
    std::uniform_real_distribution<double> unif_(0,10);

    std::seed_seq ss{0xffffffff, 0xffffffff};
    // uint64_t timeSeed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    // std::seed_seq ss{uint32_t(timeSeed & 0xffffffff), uint32_t(timeSeed>>32)};
    rng_.seed(ss);


    jutils::BaseKDTrie tree(3, 0.1);

    jutils::BaseKDTrie::Point pt({-1.43, 0.43, 1.3});
    tree.insert(pt);

    std::printf("Inserting points....\n");
    std::vector<jutils::BaseKDTrie::Point> points;
    for(int i = 0; i < 1000000; ++i) {
        jutils::BaseKDTrie::Point npt(3);
        for(int i = 0; i < 3; ++i) {
            npt[i] = pt[i] - 5 + unif_(rng_);
        }
        points.push_back(npt);
        tree.insert(npt);
    }
    std::printf("Points inserted....\n");


    // // tree.nn(na, pt, 2.0);
    // std::printf("Points (%lu):\n", na.size());
    // for(const auto& p : na) {
    //     std::printf(" %3.2f, %s\n",
    //         p.distance,
    //         jutils::string_format_point("", p.point).c_str()
    //     );
    // }


    jutils::BaseKDTrie::Point pt2({-1.43, 1.43, 1.5});
    // Test searching for a single closest point
    auto before = std::chrono::high_resolution_clock::now();

    jutils::BaseKDTrie::NearAssociates na;
    tree.nn(na, pt2, (std::size_t)1);
    auto after = std::chrono::high_resolution_clock::now();
    auto len = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(after - before).count())*1e-6;
    std::printf("Tree Closest Point: %3.4fs (%lu)\n", len, na.size());

    // Search for points within a distance
    before = std::chrono::high_resolution_clock::now();
    tree.nn(na, pt2, 1.0);

    after = std::chrono::high_resolution_clock::now();
    len = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(after - before).count())*1e-6;
    std::printf("Tree Closest Point(s) within dist: %3.4fs (%lu)\n", len, na.size());

    // Test searching for several closest points
    before = std::chrono::high_resolution_clock::now();
    tree.nn(na, pt2, (std::size_t)10);

    after = std::chrono::high_resolution_clock::now();
    len = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(after - before).count())*1e-6;
    std::printf("Tree Closest Point(s): %3.4fs (%lu)\n", len, na.size());


    // Test finding just the closest point
    before = std::chrono::high_resolution_clock::now();
    // std::size_t idx = 0;
    auto dist = jutils::_internal::L2NormDist<double, std::vector<double>>()(points[0], pt2);
    for(std::size_t i = 1; i < points.size(); ++i) {
        auto ndist = jutils::_internal::L2NormDist<double, std::vector<double>>()(points[0], pt2);
        if(ndist < dist) {
            dist = ndist;
            // idx = i;
        }
    }
    after = std::chrono::high_resolution_clock::now();
    len = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(after - before).count())*1e-6;
    std::printf("Vector Search Closest Point: %3.4fs (%d)\n", len, 1);

    // Test sorting all
    before = std::chrono::high_resolution_clock::now();
    jutils::BaseKDTrie::NearAssociates na2;
    for(std::size_t i = 0; i < points.size(); ++i) {
        auto nan = jutils::BaseKDTrie::NearAssociate::fromPoints(points[i], nullptr, pt2);
        na2.push_back(nan);
    }
    std::sort(na2.begin(), na2.end());
    after = std::chrono::high_resolution_clock::now();
    len = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(after - before).count())*1e-6;
    std::printf("Sort all: %3.4fs (%lu)\n", len, na2.size());

    // std::cout << std::bitset<18>(1 << 2) << std::endl;

    return 0;
}
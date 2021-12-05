/**
 * @brief aa kdtrie for sptial processing
 * @date 11/28/2021
 * @author Joshua Spisak <jspisak@andrew.cmu.edu>
 * 
 * Copyright 2021 Joshua Spisak
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 **/
#ifndef JUTILS_KDTRIE_HPP_
#define JUTILS_KDTRIE_HPP_
#define JUTILS_KDTRIE_VERSION_MAJOR 0
#define JUTILS_KDTRIE_VERSION_MINOR 1

#include <jutils/logging.hpp>
#ifdef JUTILS_KDTREE_DEBUG_LOGGING
    #define KDTRIE_DEBUG(...) JLOGN_NOTE("kdtrie", ##__VA_ARGS__)
#else
    #define KDTRIE_DEBUG(...)
#endif

/**
 * @brief the KDTrie is a dataastructure similar to an oct or quad tree
 *  that uses tiered containers that split state space in half in each
 *  dimension each layer futher into the datastructure until it reaches
 *  a base width.
 *
 *  It uses some number of branch nodes where each subsection of space
 *  goes into another branch until the branch that is base-width is a
 *  leaf that can contain a list of the points and associated data.
 *
 *  Each container has a base point which is the corner of the container,
 *  The container also has a width which extends the container that width
 *  from the base point along each dimension. This is split into 2^d
 *  sub-containers which have width/2.
 **/

#include <vector>
#include <cstddef>
#include <cmath>
#include <memory>
#include <bitset>
#include <algorithm>
#include <unordered_map>

namespace jutils {

/**
 * @brief formats a point to a string
 * @param[in] name the name of the point for formatting
 * @param[in] point the point to format
 * @param PointT template parameter so different containers can be used
 **/
template<typename PointT=std::vector<double>>
std::string string_format_point(const std::string& name, const PointT& point) {
    std::string result = name + "(";
    for(std::size_t i = 0; i < point.size(); ++i) {
        if(i != 0)
            result += ", ";
        result += string_format("%3.2f", point[i]);
    }
    return result + ")";
}


template<typename PointT=std::vector<double>>
std::string string_format_point(const PointT& point) {
    return string_format_point("", point);
}

//! Used internal to KDTrie, not for direct acces by application
namespace _internal {
    /**
     * @brief used to perform closest point functions, performs
     *  an l2 norm function on two points (eg: euclidian distance)
     * @param DistT the type used to express distance
     * @param PointT the type used to express a single point
     **/
    template<
        typename DistT = double,
        typename PointT = std::vector<double>
    >
    struct L2NormDist {
        /**
         * @brief performs an l2 norm computation on two points
         * @param[in] a the first point
         * @param[in] b the second point
         * @return the l2 norm distance between points a and b
         **/
        DistT operator()(const PointT& a, const PointT& b) {
            double result = 0.0;
            for(std::size_t i = 0; i < a.size() && i < b.size(); ++i) {
                result += std::pow(a[i] - b[i], 2.0);
            }
            return std::sqrt(result);
        };
    };

    /**
    * @brief used to process closes point requests, put in internal because
    *   I doubt it will be directly useful to end applications. This might not
    *   be the best way to structure this.
    * @param DistT the type used to express distance
    * @param PointT the type containing a point
    * @param AssocateT the data associated with the point
    * @param DistFunc the distance function to use (default l2 norm)
    **/
    template<
        typename DistT = double,
        typename PointT = std::vector<double>,
        typename AssociateT = std::nullptr_t
    >
    struct DistV {
        //! The distance between point and some target that is not stored
        DistT distance;
        //! The point value
        PointT point;
        //! The assocated value with this point
        AssociateT associate;

        /**
         * @brief instantiates a DistV
         * @param[in] point the point value
         * @param[in] associate the associated value
         * @param[in] target the point to calculate the distance from
         * @param DistFunc template parameter that defines what
         *  distance function to use
         **/
        template<typename DistFunc = L2NormDist<DistT, PointT>>
        static DistV fromPoints(
            const PointT& _point,
            const AssociateT& _associate,
            const PointT& _target
        ) {
            DistV r;
            r.point = _point;
            r.associate = _associate;
            r.distance = DistFunc()(_point, _target);
            return r;
        }

        //! Comparator function for use with std::sort
        bool operator<(const DistV& a) {
            return distance < a.distance;
        }

        //! Comparator struct for DistV
        struct Compare {
            bool operator()(const DistV& a, const DistV& b) {
                return a.distance < b.distance;
            }
        };
    };

    /**
     * @brief finds the closest base multiple of a given width to a point
     * @param[in] point the point to try to be closest to
     * @param[in] mul the multple value we are trying to align to
     * @return the aligned base value
     **/
    template<typename PointT = std::vector<double>>
    inline PointT align_point(const PointT& point, const double& mul) {
        PointT res(point.size());
        for(std::size_t i = 0; i < point.size(); ++i) {
            res[i] = point[i] - std::remainder(point[i], mul);
        }
        return res;
    }

    /**
     * @brief grows a root towards a target value
     * @param[in] target the target that we want to be contained in a root
     * @param[in] root a base-width aligned root
     * @param[in] width the width of the container at root
     * @return a new root with width = input width*2.0 grown towards the target
     **/
    template<typename PointT = std::vector<double>>
    inline PointT grow_root_towards(const PointT& target, const PointT& root, const double& width) {
        PointT res(root.size());
        for(std::size_t i = 0; i < target.size() && i < root.size(); ++i) {
            if(target[i] < root[i]) {
                res[i] = root[i] - width;
            } else {
                res[i] = root[i];
            }
        }
        return res;
    }

    /**
     * @brief intended to be the leaf on the end of the KDTrie
     *  that contains all the points and associated data. The idea
     *  is that end-users can implement something with the same
     *  interface to get custom behavior at the leaf's
     **/
    template<
        typename AssociateT=std::nullptr_t,
        typename PointDimT=double,
        typename PointT=std::vector<double>
    >
    class KDTrieLeaf {
    public:
        typedef std::shared_ptr<KDTrieLeaf> Ptr;
        typedef std::weak_ptr<KDTrieLeaf> WeakPtr;
        typedef std::vector<std::pair<PointT,AssociateT>> Data;

        /**
         * @brief templated function to construct a KDTrieLeaf::Ptr
         **/
        template<typename ... Args>
        static Ptr newPtr(Args ... args) {
            return std::make_shared<KDTrieLeaf>(args ...);
        }

        KDTrieLeaf(const PointT& root, const double& width):
            root_(root),
            width_(width)
        {}

        /**
         * @brief adds a point to this leaf
         * @param[in] point the point to add
         * @param[in] associate the assocated data 
         **/
        bool insert(const PointT& point, const AssociateT& associate) {
            KDTRIE_DEBUG("  KDTrieLeaf.insert(%s), data.size() = %lu",
                string_format_point("",point).c_str(), data_.size()
            );
            data_.push_back({point, associate});
            return true;
        }

        /**
         * @brief alows access to the data internal to this leaf
         *  allowing the upper level branch to execute nearest neighbor
         *  and other searches
         **/
        const Data& data() const {
            return data_;
        }

    private:
        std::vector<std::pair<PointT,AssociateT>> data_;
        PointT root_;
        double width_;

    };


    template<
        typename AssociateT=std::nullptr_t,
        typename PointDimT=double,
        typename PointT=std::vector<double>,
        typename BitStringT=uint8_t,
        typename LeafContainerT=_internal::KDTrieLeaf<AssociateT,PointDimT,PointT>
    >
    class KDTrieBranch {
    public:
        typedef std::shared_ptr<KDTrieBranch> Ptr;
        typedef std::weak_ptr<KDTrieBranch> WeakPtr;

        typedef PointT Point;
        typedef AssociateT Associate;
        typedef _internal::DistV<PointDimT, PointT, AssociateT> NearAssociate;
        typedef _internal::DistV<PointDimT, PointT, typename LeafContainerT::Ptr> NearLeaf;
        typedef _internal::DistV<PointDimT,PointT, Ptr> NearBranch;
        typedef std::vector<NearAssociate> NearAssociates;
        typedef std::vector<NearLeaf> NearLeafs;
        typedef std::vector<NearBranch> NearBranches;

        template<typename ... Args>
        static Ptr newPtr(Args ... args) {
            return std::make_shared<KDTrieBranch>(args ...);
        }

        KDTrieBranch(PointT root, double width, double base_width):
            root_(root), width_(width), base_width_(base_width)
        {
            base_ = std::abs(width - base_width) < 1e-8;
            if(base_) {
                leaf_ = LeafContainerT::newPtr(root, base_width);
            }
            midpt_.resize(root.size());
            edgept_.resize(root.size());
            for(std::size_t i = 0; i < root.size(); ++i) {
                midpt_[i] = root[i] + width_ * 0.5;
                edgept_[i] = root[i] + width_;
            }

            KDTRIE_DEBUG("  New KDTrieBranch(%s, %3.2f, %3.2f)%s",
                string_format_point("",root).c_str(), width, base_width_, base_ ? " base" : ""
            );
        }

        /**
         * @brief adds a point to this branch, this assumes that the parent
         *  has checked that the point belongs in this branch and does not
         *  perform extra checks.
         * @param[in] point the point to add
         * @param[in] associate any data associated with the point
         * @return whether or not the addition was successful
         **/
        bool insert(const PointT& point, const AssociateT& associate) {
            KDTRIE_DEBUG("  KDTrieBranch(%s, %3.2f, %3.2f)",
                string_format_point("",root_).c_str(), width_, base_width_
            );
            KDTRIE_DEBUG("    insert(%s) base=%s",
                string_format_point("",point).c_str(), base_ ? "true" : "false"
            );

            if(base_) {
                leaf_->insert(point, associate);
            } else {
                BitStringT id_ = 0;
                for(std::size_t i = 0; i < point.size(); ++i) {
                    if(point[i] > midpt_[i] || std::fabs(point[i] - midpt_[i]) < 1e-6) {
                        id_ |= (1 << i);
                    }
                }
                KDTRIE_DEBUG("    %s %s", string_format_point("midpt=", midpt_).c_str(),
                    std::bitset<sizeof(BitStringT)*8>(id_).to_string().c_str());

                auto it = branches_.find(id_);
                KDTrieBranch::Ptr branch;
                if(it == branches_.end()) {
                    PointT sub_root = root_;
                    for(std::size_t i = 0; i < point.size(); ++i) {
                        if(point[i] > midpt_[i] || std::fabs(point[i] - midpt_[i]) < 1e-6) {
                            sub_root[i] = midpt_[i];
                        }
                    }

                    branch = KDTrieBranch::newPtr(sub_root, width_*0.5, base_width_);
                    branches_[id_] = branch;
                } else {
                    branch = it->second;
                }
                branch->insert(point, associate);
            }

            return true;
        } /* insert() */

        /**
         * @brief runs a nearest neighbor search into the tree
         * @param DistanceFunction template parameter that determines
         *  how distance is calculated
         * @param[out] neighbors neighbors are added to this vector
         * @param[in] target the point to measure distance relative to
         * @param[in] distance the max distance to add neighbors
         * @param[in] num the max number of neighbors to add
         **/
        template<typename DistanceFunction = _internal::L2NormDist<PointDimT, PointT>>
        void nearestNeighbors(
            NearAssociates& neighbors,
            const PointT& target,
            const PointDimT& distance,
            const std::size_t num
        ) {
            if(base_) {
                for(const auto& n : leaf_->data()) {
                    KDTRIE_DEBUG("  Checking point %s", string_format_point("", n.first).c_str());
                    auto na = NearAssociate::fromPoints(n.first, n.second, target);
                    if(na.distance < distance) {
                        if(neighbors.size() >= num) {
                            if(na.distance > neighbors[neighbors.size() - 1].distance)
                                continue;
                            neighbors[neighbors.size() - 1] = na;
                            std::sort(neighbors.begin(), neighbors.end());
                        } else {
                            neighbors.push_back(na);
                        }
                    }
                }
                return;
            }

            NearBranches nbs;
            for(const auto& it : branches_) {
                PointT p;
                it.second->near_edge(target, p);
                auto nb = NearBranch::fromPoints(p, it.second, target);
                nbs.push_back(nb);
            }
            std::sort(nbs.begin(), nbs.end());

            for(const auto& nb : nbs) {
                KDTRIE_DEBUG("  expanding %s at %3.2f",
                    string_format_point("", nb.point).c_str(), nb.distance);
                // We have sorted the branches by distance so once we
                // reach one that is too foar the rest are too far as well
                if(nb.distance > distance) {
                    KDTRIE_DEBUG("    expansion completed (%3.2f > %3.2f)", nb.distance, distance);
                    break;
                }
                if(neighbors.size() >= num && nb.distance >= neighbors[num - 1].distance) {
                    KDTRIE_DEBUG("    expansion completed (%d >= %d, %3.2f >= %3.2f)",
                        neighbors.size(), num, nb.distance, neighbors[num - 1].distance);
                    break;
                }
                nb.associate->nearestNeighbors(neighbors, target, distance, num);
            }
        } /* nearestNeighbors() */

        /**
         * @brief allows branches to be grafted in during expansion of the tree
         * @param[in] branch the branch to graft in
         * @return whether the graft was successful
         **/
        bool graft(const KDTrieBranch::Ptr& child_branch) {
            auto child_root = child_branch->root();

            KDTRIE_DEBUG("    graft(%s)", string_format_point(child_root).c_str());
            BitStringT id = 0;
            for(std::size_t i = 0; i < child_root.size(); ++i) {
                if(std::fabs(child_root[i] - midpt_[i]) < 1e-6) {
                    id |= (1 << i);
                } else if(std::fabs(child_root[i] - root_[i]) > 1e-6) {
                    KDTRIE_DEBUG("    rejecting graft, not aligned with child edges.");
                    return false;
                }
            }

            KDTRIE_DEBUG("    successful id %s", std::bitset<sizeof(BitStringT)*8>(id).to_string().c_str());
            branches_[id] = child_branch;
            return true;
        }

        /**
         * @brief checks whether a point is in the range of this branch
         * @param[in] point the point to check
         * @return true if contained, false if not
         **/
        bool contains(const PointT& point) {
            bool contains = true;
            for(std::size_t i = 0; i < point.size() && i < root_.size(); ++i) {
                if(point[i] < root_[i] || point[i] >= edgept_[i])
                    contains = false;
            }
            return contains;
        }

        /**
         * @brief gets the closest point that could possibly be in this branch
         * @param[in] target the point we should find the closest
         *  possible contained point to
         * @param[out] edge the resulting closest possible point, naturally this lies on
         *  an edge or the surface of this container
         **/
        void near_edge(const PointT& target, PointT& edge) const {
            edge.resize(target.size());
            for(std::size_t i = 0; i < target.size(); ++i) {
                if(target[i] < root_[i]) {
                    edge[i] = root_[i];
                } else if(target[i] >= edgept_[i]) {
                    edge[i] = edgept_[i];
                } else {
                    edge[i] = target[i];
                }
            }
        }

        /**
         * @brief gets the closest point that could possibly be in this branch
         * @param[in] target the point we should find the closest
         *  possible contained point to
         * @param[out] edge the resulting closest possible point, naturally this lies on
         *  an edge or the surface of this container
         **/
        void far_edge(const PointT& target, PointT& edge) const {
            edge.resize(target.size());
            for(std::size_t i = 0; i < target.size(); ++i) {
                if(std::abs(target[i] - root_[i]) > std::abs(target[i] - edgept_[i])) {
                    edge[i] = root_[i];
                } else {
                    edge[i] = edgept_[i];
                }
            }
        }

        //! The root point of this container
        const PointT& root() const {
            return root_;
        }

        //! The width of this container
        const double& width() const {
            return width_;
        }

        //! The base widths of this container as a whole
        const double& base_width() const {
            return base_width_;
        }

    private:
        //! The root point (base edge of this container)
        PointT root_;
        //! The mid point of this container
        PointT midpt_;
        //! The far edge of this container opposite root
        PointT edgept_;
        //! Width of the container
        double width_;
        //! Base width of the KDTrie
        double base_width_;
        //! Whether or not this container is at the base (width == base_width)
        bool base_;

        //! The sub-branches of this branch (only if !base_)
        std::unordered_map<BitStringT,Ptr> branches_;
        //! The leaf at the end (only if base_)
        typename LeafContainerT::Ptr leaf_;
    };
}

/**
 * @brief KDTrie is a k-dimensional oct/quadtree, it is named Trie
 *  instead of Tree because Tree was already taken.
 * 
 * Template parameters ordered by subsumability / liklihood of being changed
 * @param AssociateT type of data that is passed associated to a point,
 *  currently we assume this to be a pointer type and default to nullptr
 *  internally.
 * @param PointDimT type representing the value of a dimension in space
 * @param PointT the type used to represent a k-dimensional point
 * @param BitStringT the type to use for the underlying bitstring, the bitsize
 *          of this template param determines the max dimensionality of PointT
 * @param LeafContainerT the container used at the leaves to store points
 * @param BranchContainerT the container used in branches of the trie
 *                         can point at branches or leaves
 **/
template<
    typename AssociateT = std::nullptr_t,
    typename PointDimT      = double,
    typename PointT         = std::vector<PointDimT>,
    typename BitStringT     = uint8_t,
    typename LeafContainerT = _internal::KDTrieLeaf<AssociateT,PointDimT,PointT>,
    typename BranchContainerT=
            _internal::KDTrieBranch<AssociateT,PointDimT,PointT,BitStringT,LeafContainerT>
>
class KDTrie {
public:
    typedef typename BranchContainerT::Point Point;
    typedef typename BranchContainerT::Associate Associate;
    typedef typename BranchContainerT::NearAssociate NearAssociate;
    typedef typename BranchContainerT::NearLeaf NearLeaf;
    typedef typename BranchContainerT::NearAssociates NearAssociates;
    typedef typename BranchContainerT::NearLeafs NearLeafs;
    typedef std::shared_ptr<KDTrie> Ptr;

    /**
     * @brief instantiates a KDTrie with k-dimensions
     **/
    KDTrie(int k, PointDimT base_width):
        k_(k),
        base_width_(base_width)
    {
        KDTRIE_DEBUG("KDtrie(k=%d, bw=%3.2f)", k, base_width);
        if( static_cast<std::size_t>(k) > sizeof(BitStringT) * 8 ) {
            throw std::runtime_error(string_format("bitstring is not large enough :( (%d > %lu)", k, sizeof(BitStringT)));
        }
    }

    /**
     * @brief insert a point and it's associated data into the KDTrie
     * @param[in] point the point to add
     * @param[in] associate the data associated with this point (not required)
     * @return true if success, false if not
     **/
    bool insert(const PointT& point, const AssociateT& associate = nullptr) {
        if(point.size() != static_cast<std::size_t>(k_)) {
            throw std::runtime_error(string_format("%s has size %d, tree expects size %d",
                string_format_point("", point).c_str(), point.size(), k_
            ));
        }

        KDTRIE_DEBUG("KDtrie(%d, %3.2f).insert(%s)", 
            k_, base_width_, string_format_point("",point).c_str());

        if(!base_branch_) {
            root_ =_internal::align_point(point, base_width_);
            KDTRIE_DEBUG("  Creating root with %s", string_format_point("base=",root_).c_str());
            base_branch_ = BranchContainerT::newPtr(root_, base_width_, base_width_);
        }

        while(!base_branch_->contains(point)) {
            auto width = base_branch_->width();
            if(width > std::pow(2, 32)) {
                std::runtime_error(string_format("cannot expand tree further (width %3.2f > %3.2f", width, std::pow(2., 32.)));
            }
            root_ = _internal::grow_root_towards(point, base_branch_->root(), width);
            KDTRIE_DEBUG("  new %s, width=%3.2f", string_format_point("root=", root_).c_str(), width * 2.0);
            auto child = base_branch_;
            base_branch_ = BranchContainerT::newPtr(root_, width * 2.0, base_width_);
            base_branch_->graft(child);
        }

        base_branch_->insert(point, associate);
        return true;
    }

    /**
     * @brief runs a nearest neighbor search into the tree
     * @param DistanceFunction template parameter that determines
     *  how distance is calculated
     * @param[out] neighbors neighbors are added to this vector
     * @param[in] point the point to measure distance relative to
     * @param[in] distance the max distance to add neighbors
     * @param[in] num the max number of neighbors to add
     **/
    template<typename DistanceFunction = _internal::L2NormDist<PointDimT, PointT>>
    void nn(
        NearAssociates& neighbors,
        const PointT& point,
        const PointDimT& distance,
        const std::size_t num
    ) {
        if(!base_branch_) {
            return;
        }
        KDTRIE_DEBUG("KDtrie(%d, %3.2f).nn(%s, %s, %s)", 
            k_, base_width_, string_format_point("",point).c_str(),
            distance == std::numeric_limits<PointDimT>::max() ?
                "max" : string_format("%3.2f", distance).c_str(),
            num == std::numeric_limits<std::size_t>::max() ?
                "max" : string_format("%lu", num).c_str()
        );
        neighbors.clear();
        base_branch_->nearestNeighbors(neighbors, point, distance, num);
        std::sort(neighbors.begin(), neighbors.end());
        KDTRIE_DEBUG("  found %d neighbors", neighbors.size());
    }

    /**
     * @brief runs a nearest neighbor search into the tree (max distance)
     * @param DistanceFunction template parameter that determines
     *  how distance is calculated
     * @param[out] neighbors neighbors are added to this vector
     * @param[in] point the point to measure distance relative to
     * @param[in] num the max number of neighbors to add
     **/
    template<typename DistanceFunction = _internal::L2NormDist<PointDimT, PointT>>
    void nn(
        NearAssociates& neighbors,
        const PointT& point,
        const std::size_t num
    ) {
        nn(neighbors, point, std::numeric_limits<PointDimT>::max(), num);
    }

    /**
     * @brief runs a nearest neighbor search into the tree (max num results)
     * @param DistanceFunction template parameter that determines
     *  how distance is calculated
     * @param[out] neighbors neighbors are added to this vector
     * @param[in] point the point to measure distance relative to
     * @param[in] distance the max distance to add neighbors
     **/
    template<typename DistanceFunction = _internal::L2NormDist<PointDimT, PointT>>
    void nn(
        NearAssociates& neighbors,
        const PointT& point,
        const PointDimT& distance
    ) {
        nn(neighbors, point, distance, std::numeric_limits<std::size_t>::max());
    }

private:
    int k_;
    double base_width_;

    PointT root_;
    typename BranchContainerT::Ptr base_branch_;

}; /* class KDTrie */

typedef KDTrie<> BaseKDTrie;
typedef _internal::KDTrieBranch<>   BaseKDTrieBranch;
typedef _internal::KDTrieLeaf<>     BaseKDTrieLeaf;

} /* namespace jutils */

#endif /* JUTILS_KDTRIE_H_ */

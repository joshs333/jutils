#ifndef JUTILS_KDTRIE_H_
#define JUTILS_KDTRIE_H_

#include <vector>
#include <cstddef>
#include <memory>

namespace jutils {

//! Used internal to KDTrie, not for direct acces by application
namespace _internal {
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

    };

    template<
        typename AssociateT=std::nullptr_t,
        typename PointDimT=double,
        typename PointT=std::vector<double>,
        typename LeafContainerT=_internal::KDTrieLeaf<AssociateT,PointDimT,PointT>
    >
    class KDTrieBranch {
    public:
        typedef std::shared_ptr<KDTrieBranch> Ptr;
        typedef std::weak_ptr<KDTrieBranch> WeakPtr;

        KDTrieBranch(PointT root, double width, double base_width) {

        }
    };
}

/**
 * @brief KDTrie is a k-dimensional oct/quadtree, it is named Trie
 *  instead of Tree because Tree was already taken.
 * 
 * Template parameters ordered by subsumability / liklihood of being changed
 * @param AssociateT type of data that is passed assocated to a point
 * @param PointDimT type representing the value of a dimension in space
 * @param PointT the type used to represent a k-dimensional point
 * @param LeafContainerT the container used at the leaves to store points
 * @param BranchContainerT the container used in branches of the trie
 *                         can point at branches or leaves
 **/
template<
    typename AssociateT=std::nullptr_t,
    typename PointDimT=double,
    typename PointT=std::vector<PointDimT>,
    typename LeafContainerT=_internal::KDTrieLeaf<AssociateT,PointDimT,PointT>,
    typename BranchContainerT=
            _internal::KDTrieBranch<AssociateT,PointDimT,PointT,LeafContainerT>
>
class KDTrie {
public:
    typedef PointT Point;
    typedef AssociateT Associate;
    typedef std::shared_ptr<KDTrie> Ptr;

    /**
     * @brief instantiates a KDTrie with k-dimensions
     **/
    KDTrie(int k, PointDimT base_width):
        k_(k),
        base_width_(base_width)
    {}

    /**
     * @brief insert a point and it's associated data into the KDTrie
     * @param[in] point the point to add
     * @param[in] assocate the data assocated with this point (not required)
     * @return true if success, false if not
     **/
    bool insert(const PointT& point, const AssociateT& assocate) {

        return false;
    }

private:
    int k_;
    double base_width_;

    PointT root_;
    typename BranchContainerT::Ptr base_branch_;

}; /* class KDTrie */

typedef KDTrie<> BaseKDTrie;

} /* namespace jutils */

#endif /* JUTILS_KDTRIE_H_ */
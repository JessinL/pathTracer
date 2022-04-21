#ifndef BVH_TREE_H_
#define BVH_TREE_H_

#include "bvhnode.h"
#include <algorithm>
#include <set>

class bvhTree_{
public:
    bvhTree_() {}
    ~bvhTree_() {}

    int setobject(const std::vector<double>& vertices, const std::vector<int>& facets);

    int setmaxfacetsnum(const int& maxfacetsnum);

    int setmaxdepth(const int& maxdepth);

    bvhNode_* getroot() const;

    int getdepth() const;

    int getmaxdepth() const;

    int buildtree();

    int dividenode(bvhNode_* node, const int& divideaxis);

    int rayobjectintersectionfinder(
        const Vector& raystartingpoint, const Vector& raydirection,
        std::vector<int>& possibleintersectionfacets
    ) const;

    int raynodeintersectionfinder(
        const Vector& raystartingpoint, const Vector& raydirection,
        bvhNode_* node,
        std::vector<int>& possibleintersectionfacets
    ) const;

    int rayboxintersectionfinder(
        const Vector& raystartingpoint, const Vector& raydirection,
        const Vector& boxminxyz, const Vector& boxmaxxyz
    ) const;

protected:
    bvhNode_* root_{nullptr};
    int depth_{0};

    int maxdepth_{10};

    // max facets num of every node
    int maxfacetsnum_{0};

    // triangle facet object
    std::vector<double> vertices_;
    std::vector<int> facets_;
};

#endif // BVH_TREE_H_
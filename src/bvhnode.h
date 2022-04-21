#ifndef BVH_NODE_H_
#define BVH_NODE_H_

#include "vector.h"
#include <vector>

class bvhNode_{
public:
    bvhNode_() {}
    ~bvhNode_() {}
    bvhNode_(const bvhNode_& bvhNode);

    int setminxyz(const Vector& minxyz);
    int setmaxxyz(const Vector& maxxyz);

    int setleaf();
    int setinternal();

    int setleftchild(bvhNode_* child);
    int setrightchild(bvhNode_* child);

    int setdepth(const int& depth);

    /**
     * @brief set some facets to a leaf node
     * @note only leaf node could be set
     * 
     * @param facetids 
     * @return int 
     */
    int setfacets(const std::vector<int>& facetids);

    Vector getminxyz();
    Vector getmaxxyz();
    
    bool isleaf();
    bool isinternal();

    bvhNode_* getleftchild();
    bvhNode_* getrightchild();

    int getdepth();

    std::vector<int> getfacetids();

    int divide(
        const int& divideaxis, 
        const Vector& leftminxyz, const Vector& leftmaxxyz,
        const Vector& rightminxyz, const Vector& rightmaxxyz,
        const std::vector<int>& leftfacetids, const std::vector<int>& rightfacetids
    );

private:
    Vector aabbminxyz_;
    Vector aabbmaxxyz_;
    bool isleaf_{true};

    // for internal nodes
    bvhNode_* leftchild_{nullptr};
    bvhNode_* rightchild_{nullptr};
    int divideaxis_{-1};
    int depth_{-1};

    // for leaf nodes
    std::vector<int> facetids_;
};

#endif // BVH_NODE_H_
#include "bvhnode.h"

bvhNode_::bvhNode_(const bvhNode_& bvhNode){
    aabbminxyz_ = bvhNode.aabbminxyz_;
    aabbmaxxyz_ = bvhNode.aabbmaxxyz_;
    isleaf_ = bvhNode.isleaf_;

    leftchild_ = bvhNode.leftchild_;
    rightchild_ = bvhNode.rightchild_;
    divideaxis_ = bvhNode.divideaxis_;

    facetids_ = bvhNode.facetids_;
}

int bvhNode_::setmaxxyz(const Vector& maxxyz){
    aabbmaxxyz_ = maxxyz;
    return 0;
}

int bvhNode_::setminxyz(const Vector& minxyz){
    aabbminxyz_ = minxyz;
    return 0;
}

int bvhNode_::setleaf(){
    isleaf_ = true;
    return 0;
}

int bvhNode_::setinternal(){
    isleaf_ = false;
    return 0;
}

int bvhNode_::setleftchild(bvhNode_* child){
    leftchild_ = child;
    return 0;
}

int bvhNode_::setrightchild(bvhNode_* child){
    rightchild_ = child;
    return 0;
}

int bvhNode_::setdepth(const int& depth){
    depth_ = depth;
    return 0;
}

int bvhNode_::setfacets(const std::vector<int>& facetids){
    if(!isleaf_)
        return 1;
    facetids_ = facetids;
    return 0;
}

Vector bvhNode_::getminxyz(){
    return aabbminxyz_;
}

Vector bvhNode_::getmaxxyz(){
    return aabbmaxxyz_;
}

bool bvhNode_::isleaf(){
    return isleaf_;
}

bool bvhNode_::isinternal(){
    return !isleaf_;
}

int bvhNode_::getdepth(){
    return depth_;
}

bvhNode_* bvhNode_::getleftchild(){
    return leftchild_;
}

bvhNode_* bvhNode_::getrightchild(){
    return rightchild_;
}

std::vector<int> bvhNode_::getfacetids(){
    return facetids_;
}

int bvhNode_::divide(
    const int& divideaxis, 
    const Vector& leftminxyz, const Vector& leftmaxxyz,
    const Vector& rightminxyz, const Vector& rightmaxxyz,
    const std::vector<int>& leftfacetids, const std::vector<int>& rightfacetids
){
    if(!isleaf_)
        return 1;
    isleaf_ = false;

    facetids_.clear();
    std::vector<int>(0).swap(facetids_);

    bvhNode_* leftchild = new bvhNode_();
    leftchild->setmaxxyz(leftmaxxyz);
    leftchild->setminxyz(leftminxyz);
    leftchild->setfacets(leftfacetids);
    leftchild->setdepth(getdepth() + 1);

    bvhNode_* rightchild = new bvhNode_();
    rightchild->setmaxxyz(rightmaxxyz);
    rightchild->setminxyz(rightminxyz);
    rightchild->setfacets(rightfacetids);
    rightchild->setdepth(getdepth() + 1);

    setleftchild(leftchild);
    setrightchild(rightchild);

    return 0;
}


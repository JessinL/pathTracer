#include "bvhtree.h"

int bvhTree_::setobject(const std::vector<double>& vertices, const std::vector<int>& facets){
    vertices_ = vertices;
    facets_ = facets;
    return 0;
}

int bvhTree_::setmaxfacetsnum(const int& maxfacetsnum){
    maxfacetsnum_ = maxfacetsnum;
    return 0;
}

int bvhTree_::setmaxdepth(const int& maxdepth){
    maxdepth_ = maxdepth;
    return 0;
}

bvhNode_* bvhTree_::getroot() const{
    return root_;
}

int bvhTree_::getdepth() const{
    return depth_;
}

int bvhTree_::getmaxdepth() const{
    return maxdepth_;
}

int bvhTree_::buildtree(){
    if(root_ != nullptr)
        return 1;
    if(vertices_.empty())
        return 1;
    if(facets_.empty())
        return 1;
    if(maxfacetsnum_ <= 0)
        return 1;
    
    Vector aabbminxyz(vertices_[0], vertices_[1], vertices_[2]);
    Vector aabbmaxxyz(vertices_[0], vertices_[1], vertices_[2]);
    for(int vid = 1; vid < vertices_.size() / 3; vid++){
        if(vertices_[3*vid] < aabbminxyz.x)
            aabbminxyz.x = vertices_[3*vid];
        if(vertices_[3*vid] > aabbmaxxyz.x)
            aabbmaxxyz.x = vertices_[3*vid];
        if(vertices_[3*vid + 1] < aabbminxyz.y)
            aabbminxyz.y = vertices_[3*vid + 1];
        if(vertices_[3*vid + 1] > aabbmaxxyz.y)
            aabbmaxxyz.y = vertices_[3*vid + 1];
        if(vertices_[3*vid + 2] < aabbminxyz.z)
            aabbminxyz.z = vertices_[3*vid + 2];
        if(vertices_[3*vid + 2] > aabbmaxxyz.z)
            aabbmaxxyz.z = vertices_[3*vid + 2];
    }

    root_ = new bvhNode_();
    std::vector<int> ff(facets_.size() / 3);
    for(int i = 0; i < ff.size(); i++)
        ff[i] = i;
    root_->setfacets(ff);
    root_->setmaxxyz(aabbmaxxyz);
    root_->setminxyz(aabbminxyz);
    root_->setdepth(0);

    dividenode(root_, 0);

    return 0;
}

int bvhTree_::dividenode(bvhNode_* node, const int& divideaxis){
    if(node->isinternal())
        return 1;
    
    if(node->getfacetids().size() <= maxfacetsnum_)
        return 0;
    
    if(divideaxis < 0 || divideaxis > 2)
        return 1;

    if(node->getdepth() >= maxdepth_)
        return 0;

    std::vector< std::pair<double, int> > sortedfacets; 
    std::vector<int> facets = node->getfacetids();
    
    for(int fid = 0; fid < facets.size(); fid++){
        double mincoord = 0;
        if(divideaxis == 0){
            mincoord = vertices_[3*facets_[3*facets[fid]]];
            mincoord = std::min(mincoord, vertices_[3*facets_[3*facets[fid] + 1]]);
            mincoord = std::min(mincoord, vertices_[3*facets_[3*facets[fid] + 2]]);
        }
        else if(divideaxis == 1){
            mincoord = vertices_[3*facets_[3*facets[fid]] + 1];
            mincoord = std::min(mincoord, vertices_[3*facets_[3*facets[fid] + 1] + 1]);
            mincoord = std::min(mincoord, vertices_[3*facets_[3*facets[fid] + 2] + 1]);
        }
            
        else{
            mincoord = vertices_[3*facets_[3*facets[fid]] + 2];
            mincoord = std::min(mincoord, vertices_[3*facets_[3*facets[fid] + 1] + 2]);
            mincoord = std::min(mincoord, vertices_[3*facets_[3*facets[fid] + 2] + 2]);
        }
        sortedfacets.push_back(std::pair<double, int>(mincoord, facets[fid]));    
    }
    std::sort(sortedfacets.begin(), sortedfacets.end());

    std::vector<int> leftfacetids;
    std::vector<int> rightfacetids;
    int middle = sortedfacets.size() / 2;
    for(int f = 0; f < middle; f++){
        leftfacetids.push_back(sortedfacets[f].second);
    }
    for(int f = middle; f < sortedfacets.size(); f++){
        rightfacetids.push_back(sortedfacets[f].second);
    }

    std::set<int> leftverticesid; 
    for(int lf = 0; lf < leftfacetids.size(); lf++){
        int& fid = leftfacetids[lf];
        int& vaid = facets_[3*fid];
        int& vbid = facets_[3*fid + 1];
        int& vcid = facets_[3*fid + 2];
        leftverticesid.insert(vaid);
        leftverticesid.insert(vbid);
        leftverticesid.insert(vcid);
    }
    Vector leftminxyz(1e3, 1e3, 1e3); Vector leftmaxxyz(1e-3, 1e-3, 1e-3);
    for(auto i = leftverticesid.begin(); i != leftverticesid.end(); i++){
        if(vertices_[3*(*i)] > leftmaxxyz.x)
            leftmaxxyz.x = vertices_[3*(*i)];
        if(vertices_[3*(*i) + 1] > leftmaxxyz.y)
            leftmaxxyz.y = vertices_[3*(*i) + 1];
        if(vertices_[3*(*i) + 2] > leftmaxxyz.z)
            leftmaxxyz.z = vertices_[3*(*i) + 2];
        if(vertices_[3*(*i)] < leftminxyz.x)
            leftminxyz.x = vertices_[3*(*i)];
        if(vertices_[3*(*i) + 1] < leftminxyz.y)
            leftminxyz.y = vertices_[3*(*i) + 1];
        if(vertices_[3*(*i) + 2] < leftminxyz.z)
            leftminxyz.z = vertices_[3*(*i) + 2];
    }

    std::set<int> rightverticesid;
    for(int rf = 0; rf < rightfacetids.size(); rf++){
        int& fid = rightfacetids[rf];
        int& vaid = facets_[3*fid];
        int& vbid = facets_[3*fid + 1];
        int& vcid = facets_[3*fid + 2];
        rightverticesid.insert(vaid);
        rightverticesid.insert(vbid);
        rightverticesid.insert(vcid);
    }
    Vector rightminxyz(1e3, 1e3, 1e3); Vector rightmaxxyz(1e-3, 1e-3, 1e-3);
    for(auto i = rightverticesid.begin(); i != rightverticesid.end(); i++){
        if(vertices_[3*(*i)] > rightmaxxyz.x)
            rightmaxxyz.x = vertices_[3*(*i)];
        if(vertices_[3*(*i) + 1] > rightmaxxyz.y)
            rightmaxxyz.y = vertices_[3*(*i) + 1];
        if(vertices_[3*(*i) + 2] > rightmaxxyz.z)
            rightmaxxyz.z = vertices_[3*(*i) + 2];
        if(vertices_[3*(*i)] < rightminxyz.x)
            rightminxyz.x = vertices_[3*(*i)];
        if(vertices_[3*(*i) + 1] < rightminxyz.y)
            rightminxyz.y = vertices_[3*(*i) + 1];
        if(vertices_[3*(*i) + 2] < rightminxyz.z)
            rightminxyz.z = vertices_[3*(*i) + 2];
    }

    // Vector leftminxyz(vertices_[3*facets_[leftfacetids[0]]], vertices_[3*facets_[leftfacetids[0]] + 1], vertices_[3*facets_[leftfacetids[0]] + 2]);
    // Vector leftmaxxyz(vertices_[3*facets_[leftfacetids[0]]], vertices_[3*facets_[leftfacetids[0]] + 1], vertices_[3*facets_[leftfacetids[0]] + 2]);
    // Vector rightminxyz(vertices_[3*facets_[rightfacetids[0]]], vertices_[3*facets_[rightfacetids[0]] + 1], vertices_[3*facets_[rightfacetids[0]] + 2]);
    // Vector rightmaxxyz(vertices_[3*facets_[rightfacetids[0]]], vertices_[3*facets_[rightfacetids[0]] + 1], vertices_[3*facets_[rightfacetids[0]] + 2]);

    // for(int lf = 0; lf < leftfacetids.size(); lf++){
    //     int fid = leftfacetids[lf];
    //     Vector va(vertices_[3*facets_[leftfacetids[lf]]], vertices_[3*facets_[leftfacetids[lf]] + 1], vertices_[3*facets_[leftfacetids[lf]] + 2]);
    //     Vector vb(vertices_[3*facets_[leftfacetids[lf] + 1]], vertices_[3*facets_[leftfacetids[lf] + 1] + 1], vertices_[3*facets_[leftfacetids[lf] + 1] + 2]);
    //     Vector vc(vertices_[3*facets_[leftfacetids[lf] + 2]], vertices_[3*facets_[leftfacetids[lf] + 2] + 1], vertices_[3*facets_[leftfacetids[lf] + 2] + 2]);

    //     leftminxyz.x = std::min(leftminxyz.x, va.x);
    //     leftminxyz.y = std::min(leftminxyz.y, va.y);
    //     leftminxyz.z = std::min(leftminxyz.z, va.z);
    //     leftmaxxyz.x = std::max(leftmaxxyz.x, va.x);
    //     leftmaxxyz.y = std::max(leftmaxxyz.y, va.y);
    //     leftmaxxyz.z = std::max(leftmaxxyz.z, va.z);

    //     leftminxyz.x = std::min(leftminxyz.x, vb.x);
    //     leftminxyz.y = std::min(leftminxyz.y, vb.y);
    //     leftminxyz.z = std::min(leftminxyz.z, vb.z);
    //     leftmaxxyz.x = std::max(leftmaxxyz.x, vb.x);
    //     leftmaxxyz.y = std::max(leftmaxxyz.y, vb.y);
    //     leftmaxxyz.z = std::max(leftmaxxyz.z, vb.z);

    //     leftminxyz.x = std::min(leftminxyz.x, vc.x);
    //     leftminxyz.y = std::min(leftminxyz.y, vc.y);
    //     leftminxyz.z = std::min(leftminxyz.z, vc.z);
    //     leftmaxxyz.x = std::max(leftmaxxyz.x, vc.x);
    //     leftmaxxyz.y = std::max(leftmaxxyz.y, vc.y);
    //     leftmaxxyz.z = std::max(leftmaxxyz.z, vc.z);        
    // }

    // for(int rf = 0; rf < rightfacetids.size(); rf++){
    //     Vector va(vertices_[3*facets_[rightfacetids[rf]]], vertices_[3*facets_[rightfacetids[rf]] + 1], vertices_[3*facets_[rightfacetids[rf]] + 2]);
    //     Vector vb(vertices_[3*facets_[rightfacetids[rf] + 1]], vertices_[3*facets_[rightfacetids[rf] + 1] + 1], vertices_[3*facets_[rightfacetids[rf] + 1] + 2]);
    //     Vector vc(vertices_[3*facets_[rightfacetids[rf] + 2]], vertices_[3*facets_[rightfacetids[rf] + 2] + 1], vertices_[3*facets_[rightfacetids[rf] + 2] + 2]);

    //     rightminxyz.x = std::min(rightminxyz.x, va.x);
    //     rightminxyz.y = std::min(rightminxyz.y, va.y);
    //     rightminxyz.z = std::min(rightminxyz.z, va.z);
    //     rightmaxxyz.x = std::max(rightmaxxyz.x, va.x);
    //     rightmaxxyz.y = std::max(rightmaxxyz.y, va.y);
    //     rightmaxxyz.z = std::max(rightmaxxyz.z, va.z);

    //     rightminxyz.x = std::min(rightminxyz.x, vb.x);
    //     rightminxyz.y = std::min(rightminxyz.y, vb.y);
    //     rightminxyz.z = std::min(rightminxyz.z, vb.z);
    //     rightmaxxyz.x = std::max(rightmaxxyz.x, vb.x);
    //     rightmaxxyz.y = std::max(rightmaxxyz.y, vb.y);
    //     rightmaxxyz.z = std::max(rightmaxxyz.z, vb.z);

    //     rightminxyz.x = std::min(rightminxyz.x, vc.x);
    //     rightminxyz.y = std::min(rightminxyz.y, vc.y);
    //     rightminxyz.z = std::min(rightminxyz.z, vc.z);
    //     rightmaxxyz.x = std::max(rightmaxxyz.x, vc.x);
    //     rightmaxxyz.y = std::max(rightmaxxyz.y, vc.y);
    //     rightmaxxyz.z = std::max(rightmaxxyz.z, vc.z);        
    // }

    node->divide(divideaxis, leftminxyz, leftmaxxyz, rightminxyz, rightmaxxyz, leftfacetids, rightfacetids);

    int depth = node->getdepth() + 1;
    if(depth > depth_)
        depth_ = depth;
    if(depth >= maxdepth_)
        return 0;

    int nextdivideaxis = 0;
    if(divideaxis == 0)
        nextdivideaxis = 1;
    else if(divideaxis == 1)
        nextdivideaxis = 2;
    
    dividenode(node->getleftchild(), nextdivideaxis);
    dividenode(node->getrightchild(), nextdivideaxis);

    return 0;
}

int bvhTree_::rayboxintersectionfinder(
    const Vector& raystartingpoint, const Vector& raydirection,
    const Vector& boxminxyz, const Vector& boxmaxxyz
) const{
    double t1, t2;
    t1 = (boxminxyz.x - raystartingpoint.x)/raydirection.x;
    t2 = (boxmaxxyz.x - raystartingpoint.x)/raydirection.x;
    double tminx = std::min(t1, t2);
    double tmaxx = std::max(t1, t2);

    t1 = (boxminxyz.y - raystartingpoint.y)/raydirection.y;
    t2 = (boxmaxxyz.y - raystartingpoint.y)/raydirection.y;
    double tminy = std::min(t1, t2);
    double tmaxy = std::max(t1, t2);

    t1 = (boxminxyz.z - raystartingpoint.z)/raydirection.z;
    t2 = (boxmaxxyz.z - raystartingpoint.z)/raydirection.z;
    double tminz = std::min(t1, t2);
    double tmaxz = std::max(t1, t2);

    double tmin = std::max(tminx, tminy);
    tmin = std::max(tmin, tminz);
    double tmax = std::min(tmaxx, tmaxy);
    tmax = std::min(tmax, tmaxz);

    if(tmax >= 0 && tmax > tmin){
        return 1;
    }
    return 0;
}

int bvhTree_::raynodeintersectionfinder(
    const Vector& raystartingpoint, const Vector& raydirection,
    bvhNode_* node,
    std::vector<int>& possibleintersectionfacets
) const{
    Vector boxminxyz = node->getminxyz();
    Vector boxmaxxyz = node->getmaxxyz();

    // 对当前节点包围盒进行相交检测
    if(rayboxintersectionfinder(raystartingpoint, raydirection, boxminxyz, boxmaxxyz)){
        // 若有相交，则判断当前节点是否为叶节点
        if(node->isleaf()){
            // 若为叶节点，将当前节点中的所有三角面片插入返回值
            std::vector<int> possiblefacets = node->getfacetids();
            for(int fid = 0; fid < possiblefacets.size(); fid++){
                possibleintersectionfacets.push_back(possiblefacets[fid]);
            }
        }
        else{
            // 若为内部节点，则递归地对其孩子节点进行相交检测
            bvhNode_* lnode = node->getleftchild();
            bvhNode_* rnode = node->getrightchild();
            raynodeintersectionfinder(raystartingpoint, raydirection, lnode, possibleintersectionfacets);
            raynodeintersectionfinder(raystartingpoint, raydirection, rnode, possibleintersectionfacets);
        }
    }
    else{
        return 0;
    }
    return 0;
}

int bvhTree_::rayobjectintersectionfinder(
    const Vector& raystartingpoint, const Vector& raydirection,
    std::vector<int>& possibleintersectionfacets
) const{
    bvhNode_* node = root_;

    raynodeintersectionfinder(raystartingpoint, raydirection, node, possibleintersectionfacets);

    return 0;
}
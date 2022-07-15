#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;
    if(this->splitMethod == BVHAccel::SplitMethod::NAIVE){
        root = recursiveBuild(primitives);
    }
    else if(this->splitMethod == BVHAccel::SplitMethod::SAH){
        root = recursiveBuild_SAH(primitives);
    }
    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    // 给定当前的objects求最大的包围盒
    Bounds3 bounds;
    for (int i = 0; i < (int)objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    // 划分到一个时停止，即叶子节点
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    // 划分到两个，左右划分
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        //包围盒的划分：以最大维度划分
        Bounds3 centroidBounds;
        for (int i = 0; i < (int)objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        int dim = centroidBounds.maxExtent();
        switch (dim) {
        case 0:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                       f2->getBounds().Centroid().x;
            });
            break;
        case 1:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y <
                       f2->getBounds().Centroid().y;
            });
            break;
        case 2:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().z <
                       f2->getBounds().Centroid().z;
            });
            break;
        }

        auto beginning = objects.begin();
        auto middling = objects.begin() + (objects.size() / 2);
        auto ending = objects.end();

        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}

/*
实现BVH中SAH的划分方法
*/
BVHBuildNode* BVHAccel::recursiveBuild_SAH(std::vector<Object*>objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    // 给定当前的objects求最大的包围盒
    Bounds3 bounds;
    for (int i = 0; i < (int)objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    // 划分到一个时停止，即叶子节点
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    // 划分到两个，左右划分
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        
        return node;
    }
    else {
        //包围盒的划分：以最大维度划分
        Bounds3 centroidBounds;
        for (int i = 0; i < (int)objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        int dim = centroidBounds.maxExtent();
        switch (dim) {
        case 0:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                       f2->getBounds().Centroid().x;
            });
            break;
        case 1:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y <
                       f2->getBounds().Centroid().y;
            });
            break;
        case 2:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().z <
                       f2->getBounds().Centroid().z;
            });
            break;
        }
        std::vector<double> time_ash;
        std::vector<Object*> leftshapes,rightshapes;
        for(int i=1; i < objects.size(); i++){
            auto beginning = objects.begin();
            auto middling = objects.begin() + i;
            auto ending = objects.end();
            leftshapes = std::vector<Object*>(beginning, middling);
            rightshapes = std::vector<Object*>(middling,ending);
            assert(objects.size() == (leftshapes.size() + rightshapes.size()));
            //计算左右节点包围盒
            Bounds3 bound1,bound2;
            for(int i=0;i<leftshapes.size();i++)
                bound1 = Union(bound1,leftshapes[i]->getBounds());
            for(int i=0;i<rightshapes.size();i++)
                bound2 = Union(bound2,rightshapes[i]->getBounds());
            //计算SAH时间代价
            double S0 = bounds.SurfaceArea();
            double S1 = bound1.SurfaceArea();
            double S2 = bound2.SurfaceArea();
            double time = (S1/S0)*leftshapes.size() + (S2/S0)*rightshapes.size();
            time_ash.push_back(time);
        }
        auto Min_ash = std::min_element(time_ash.begin(),time_ash.end());
        auto beginning = objects.begin();
        auto middling = objects.begin() + *Min_ash;
        auto endding = objects.end();
        leftshapes = std::vector<Object*>(beginning,middling);
        rightshapes = std::vector<Object*>(middling,endding);
        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;    
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    // TODO Traverse the BVH to find intersection
    // 遍历BVH加速结构，找到最近的相交点
    std::array<int,3> dirisNeg={int(ray.direction.x>0),int(ray.direction.y>0),int(ray.direction.z>0)};
    if(node->bounds.IntersectP(ray,ray.direction_inv,dirisNeg))
    {
        //遍历到叶子节点
        if(node->left==nullptr && node->right==nullptr){
            return node->object->getIntersection(ray);
        }
        else {
        //遍历到中间节点，返回当前left/right最近的相交点
            Intersection interleft = getIntersection(node->left,ray);
            Intersection interright = getIntersection(node->right,ray);
            return interleft.distance < interright.distance ? interleft:interright;   
        }
    }
    else 
    {
        return Intersection();
    }
}
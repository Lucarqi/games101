//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

// 直接做光源的采样
void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    // 发光面积
    // 那这里多个光源怎么办?
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    // 首先求交点
    Intersection inter = Scene::intersect(ray);
    if(inter.happened){
        //直接打到发光材质上
        if(inter.happened && inter.m->hasEmission()){
            // 直接返回光强 ?
            //float distance = std::pow((inter.coords - ray.origin).norm() ,2);
            return inter.emit;
        }
        //打到物体P点上
        else {
            Vector3f L_dir(0,0,0), L_indir(0,0,0);
            Intersection p = inter, p_light;
            float pdf_light;
            Scene::sampleLight(p_light, pdf_light);
            Vector3f tolight_dir = normalize(p_light.coords - p.coords);
            Ray to_light(p.coords, tolight_dir);
            Intersection middle = Scene::intersect(to_light);
            float distance = 0.0, distance_ = 0.0;
            if(middle.happened && middle.m->hasEmission()){
                distance_ = (middle.coords - p_light.coords).norm();
                if(distance_ > EPSILON)
                    L_indir = 0.0;
                else
                {
                    distance = (p_light.coords - p.coords).norm();
                    distance = std::pow(distance,2);
                    // 中间无物体
                    L_dir = p_light.emit * 
                            p.m->eval( ray.direction, tolight_dir,p.normal) * 
                            dotProduct(tolight_dir, p.normal) * 
                            std::max(0.f,dotProduct( -tolight_dir, p_light.normal)) 
                            / distance / pdf_light;
                }
            }
            float ksi = get_random_float();
            // 计算间接光照
            if(ksi <= RussianRoulette){
                // 采样出射方向wi
                Vector3f wi = p.m->sample(ray.direction, p.normal);
                wi = normalize(wi);
                Ray indir_path(p.coords, wi);
                middle = Scene::intersect(indir_path);
                // 打到不发光物体
                if(middle.happened && !middle.m->hasEmission()){
                    L_indir = castRay(indir_path,depth) * 
                              p.m->eval( ray.direction, wi,p.normal) *
                              dotProduct(wi, p.normal) / 
                              p.m->pdf( ray.direction, wi, p.normal) / RussianRoulette;
                }
            }
            return L_dir + L_indir;
        }
    }
    else return Vector3f(0,0,0);
}
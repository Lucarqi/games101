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
    Intersection p = Scene::intersect(ray);
    // hit nothing
    if(!p.happened)
        return Vector3f();
    // hit light directly
    if(p.obj->hasEmit()){
        return p.emit;
    }
    // hit object p
    // direct light 
    Vector3f L_dir, L_indir;
    Vector3f wo = -ray.direction, wi;
    Intersection q;
    float pdf_light;
    sampleLight(q, pdf_light);
    wi = normalize(q.coords - p.coords);
    Ray p_2_q(p.coords, wi);
    Intersection middle = intersect(p_2_q);
    if(middle.happened && middle.obj->hasEmit()){
        // test distance between middle and q
        float distance_ = (middle.coords - q.coords).norm();
        if(distance_ < EPSILON*10){
            // direct illumination exists
            Vector3f L_i = q.emit;
            float distance = (q.coords - p.coords).norm();
            Vector3f f_dir = p.m->eval(wi,wo,p.normal);
            L_dir = L_i * f_dir * std::max(0.f, dotProduct(wi,p.normal))
                    * std::max(0.f, dotProduct(-wi, q.normal))
                    / std::pow(distance, 2.f) / (pdf_light+EPSILON);
        }
    }
    // indirect light
    float ksi = get_random_float();
    if(ksi < RussianRoulette){
        // test ray hit no-emit object
        wi = p.m->sample(wo,p.normal);
        Ray ray_next(p.coords - EPSILON, normalize(wi));
        q = intersect(ray_next);
        if(q.happened && !q.obj->hasEmit()){
            Vector3f f_indir = p.m->eval(wi,wo,p.normal);
            float pdf_hemi = p.m->pdf(wi,wo,p.normal);
            L_indir = castRay(ray_next,depth) * f_indir * 
                      std::max(0.f, dotProduct(wi,p.normal))
                      / (pdf_hemi+EPSILON) / RussianRoulette;
        }
    }
    return L_dir + L_indir;
}
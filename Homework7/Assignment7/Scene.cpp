//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"

void Scene::buildBVH()
{
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()) {
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()) {
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum) {
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(const Ray &ray, const std::vector<Object *> &objects, float &tNear, uint32_t &index,
                  Object **hitObject)
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
    Intersection intersection = Scene::intersect(ray);
    if (!intersection.happened)  //这里意味着没有交点
        return {};
    if (intersection.m->hasEmission())  //这里意味着这个人直接看到了光源
        return intersection.m->getEmission();

    Vector3f L_dir, L_indir;
    //接下来正式应用Path Tracing
    Intersection light_pos;
    float light_pdf = 0.0f;
    Scene::sampleLight(light_pos, light_pdf);  // li得到
    L_dir = {0, 0, 0};
    Vector3f collisionlight = light_pos.coords - intersection.coords;
    float dis = dotProduct(collisionlight, collisionlight);  //计算得到距离
    Vector3f collisionlight_dir = collisionlight.normalized();
    // float pdf = intersection.m->pdf(ray.direction, collisionlight_dir, intersection.normal);
    auto fr = intersection.m->eval(ray.direction, collisionlight_dir, intersection.normal);
    Ray light_to_obj(intersection.coords, collisionlight_dir);
    Intersection light_obj = Scene::intersect(light_to_obj);
    if (light_obj.distance - collisionlight.norm() > -0.005) {
        L_dir = light_pos.emit * fr * dotProduct(collisionlight_dir, intersection.normal) *
                dotProduct(-collisionlight_dir, light_pos.normal) / dis / light_pdf;
    }
    if (get_random_float() > RussianRoulette)  //打到物体后对半圆随机采样使用RR算法
        return L_dir;
    L_indir = {0, 0, 0};

    Vector3f wi = intersection.m->sample(ray.direction, intersection.normal).normalized();
    Ray object_to_object_ray(intersection.coords, wi);
    Intersection hits = Scene::intersect(object_to_object_ray);
    if (hits.happened && !hits.m->hasEmission()) {
        float pdf_hemi = intersection.m->pdf(object_to_object_ray.direction, wi, intersection.normal);
        fr = intersection.m->eval(object_to_object_ray.direction, wi, intersection.normal);
        L_indir = castRay(object_to_object_ray, depth + 1) * fr * dotProduct(wi, intersection.normal) / pdf_hemi /
                  RussianRoulette;
    }
    return L_dir + L_indir;
    // TO DO Implement Path Tracing Algorithm here
}
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

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
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
    
    Intersection intersection=intersect(ray);

    if (!intersection.happened)
    {
        return Vector3f(0.0,0.0,0.0);
    }
  
    if (depth==0&&intersection.obj->hasEmit())
    {
        return intersection.emit;

    }
    
    else if (depth==1&&intersection.obj->hasEmit())
    {
        return Vector3f(0.0,0.0,0.0);
    }
    
    
    Vector3f& p=intersection.coords;
    Vector3f wo=-normalize(ray.direction);
    Vector3f N=intersection.normal;
    Material*& m=intersection.m;

    //Contribution from light
    Intersection lightInter;
    float lightPdf;
    sampleLight(lightInter,lightPdf);
    Vector3f& x=lightInter.coords;
    Vector3f ws=(x-p).normalized();
    Vector3f NN=lightInter.normal;
    Vector3f emit=lightInter.emit;
    float lightDistance=(x-p).norm();
    Intersection blockInter=bvh->Intersect(Ray(p,ws));
    bool isBlocked=blockInter.happened?lightDistance>blockInter.distance+0.001:false;
       
    Vector3f L_dir=(1-isBlocked)*emit*m->eval(ws,wo,N)*std::max(.0f,dotProduct(ws,N))*std::max(.0f,dotProduct(-ws,NN))/(dotProduct(x-p,x-p)*lightPdf);
    
    //Contribution from reflectors
    Vector3f L_indir;
    if (get_random_float()<RussianRoulette)
    {
        Vector3f wi=m->sample(wo,N).normalized();
        L_indir=castRay(Ray(p,wi),1)*m->eval(wi,wo,N)*std::max(.0f,dotProduct(wi,N))/(m->pdf(wi,wo,N)*RussianRoulette);
    }
    
    return L_dir+L_indir;
}
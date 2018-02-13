//
// Created by chris on 19.08.17.
//

#ifndef RAYTRACER_PRIMITIVES_H
#define RAYTRACER_PRIMITIVES_H


#include <memory>
#include <string>
#include <iostream>
#include <sstream>

#include "material.h"
#include "container.h"

/** Generic base class for all objects which shall be rendered
 *
 */
struct Primitive {
    Material material;

    explicit Primitive(const Color& color) : material(color) {}
    explicit Primitive(const Material& material = Material{}) : material(material) {}
    virtual bool intersect(const Ray& ray, Float& dist, Vec3f& normal) const = 0;
    virtual Vec3f getNormal(const Vec3f& vec) const = 0;

};


struct Plane : public Primitive {
    Float a, b, c, d;  // implicit description: ax + by + cz + d = 0

    Plane(Float a, Float b, Float c, Float d, const Color& color)
            : a(a), b(b), c(c), d(d), Primitive(color)
    {
        // normalize normal vector
        Vec3f pn{a, b, c};
        pn.normalize();
        this->a = pn[0];
        this->b = pn[1];
        this->c = pn[2];
    }

    Plane(Float a, Float b, Float c, Float d, const Material& material)
            : a(a), b(b), c(c), d(d), Primitive(material) {
        // normalize normal vector
        Vec3f pn{a, b, c};
        pn.normalize();
        this->a = pn[0];
        this->b = pn[1];
        this->c = pn[2];
    }

    Plane(Vec3f normal, Float dist, const Color& color) : d{dist}, Primitive{color} {
        normal.normalize(); // make sure normal is normalized
        a = normal[0];
        b = normal[1];
        c = normal[2];
    }

    Plane(Vec3f normal, Float dist, const Material& material) : d{dist}, Primitive{material} {
        normal.normalize(); // make sure normal is normalized
        a = normal[0];
        b = normal[1];
        c = normal[2];
    }

    bool intersect(const Ray& ray, Float& dist, Vec3f& normal) const override;

    Vec3f getNormal(const Vec3f& vec) const override;
};

struct Triangle : public Primitive {
    Vec3f v0, v1, v2, n0, n1, n2, faceNormal;

    Triangle(const Vec3f& v0, const Vec3f& v1, const Vec3f& v2, const Color& color)
            : v0{v0}, v1{v1}, v2{v2}, Primitive{color}, n0{calcNormal()}, n1{calcNormal()}, n2{calcNormal()},
              faceNormal{calcNormal()}
    { }
    Triangle(const Vec3f& v0, const Vec3f& v1, const Vec3f& v2, const Material &material=Material{})
            : v0{v0}, v1{v1}, v2{v2}, Primitive{material}, n0{calcNormal()}, n1{calcNormal()}, n2{calcNormal()},
              faceNormal{calcNormal()}
    { }
    Triangle(const Vec3f& v0, const Vec3f& v1, const Vec3f& v2,
             const Vec3f& n0, const Vec3f& n1, const Vec3f& n2,
             const Material material=Material{})
            : v0{v0}, v1{v1}, v2{v2}, Primitive{material}, n0{n0}, n1{n1}, n2{n2}, faceNormal{calcNormal()}{ }

    bool intersect(const Ray& ray, Float& dist, Vec3f& normal) const override;

    Vec3f getNormal(const Vec3f& vec) const override;

    Vec3f calcNormal();

    void scale(Float s);

    Triangle& operator+=(const Float t);

    Triangle& operator+=(const Vec3f& v);

    Triangle& operator*=(const Mat3d& mat);

    friend Triangle operator+(Triangle lhs, const Float t) {
        return lhs += t;
    }

    friend Triangle operator+(Triangle lhs, const Vec3f& v) {
        return lhs += v;
    }

};

/** The most common object to be rendered in a raytracer
 *
 */
struct Sphere : public Primitive {
    // define location + radius
    Vec3f center;
    Float radius;

    Sphere(const Vec3f& c, Float r, const Color& color) : center{c}, radius{r}, Primitive{color} {}
    Sphere(const Vec3f& c, Float r, const Material& material) : center{c}, radius{r}, Primitive{material} {}
    Sphere() : Sphere({0,0,0}, 1, Material{}) {}

    bool intersect(const Ray& ray, Float& dist, Vec3f& normal) const override;

    Vec3f getNormal(const Vec3f& P) const override;
};

#endif //RAYTRACER_PRIMITIVES_H

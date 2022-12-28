//
// Created by chris on 19.08.17.
//
#pragma once

#include <memory>
#include <string>
#include <iostream>
#include <sstream>
#include <utility>

#include "material.h"
#include "container.h"

/** Generic base class for all objects which shall be rendered
 *
 */
struct Primitive {
    Material material;

    explicit Primitive(const Color& color) : material(color) {}
    explicit Primitive(Material material = Material{}) : material(std::move(material)) {}
    virtual bool intersect(const Ray& ray, Float& dist, Vec3f& normal) const = 0;
    virtual Vec3f getNormal(const Vec3f& vec) const = 0;

    virtual ~Primitive() = default;

};


struct Plane : public Primitive {
    Float a, b, c, d;  // implicit description: ax + by + cz + d = 0

    Plane(Float a, Float b, Float c, Float d, const Color& color)
            : Plane{a, b, c, d, Material{color}} { }

    Plane(Float a, Float b, Float c, Float d, const Material& material)
            : Primitive{material}, a(a), b(b), c(c), d(d) {
        // normalize normal vector
        Vec3f pn{a, b, c};
        pn.normalize();
        this->a = pn[0];
        this->b = pn[1];
        this->c = pn[2];
    }

    Plane(const Vec3f& normal, Float dist, const Color& color) : Plane{normal, dist, Material{color}} {
    }

    Plane(Vec3f normal, Float dist, const Material& material) : Primitive{material}, d{dist} {
        normal.normalize(); // make sure normal is normalized
        a = normal[0];
        b = normal[1];
        c = normal[2];
    }

    bool intersect(const Ray& ray, Float& dist, Vec3f& normal) const override;

    Vec3f getNormal(const Vec3f& vec) const override;

    ~Plane() override = default;
};

struct Triangle : public Primitive {
    Vec3f v0, v1, v2, n0, n1, n2, faceNormal;

    Triangle(const Vec3f& v0, const Vec3f& v1, const Vec3f& v2, const Color& color)
            : Primitive{color}, v0{v0}, v1{v1}, v2{v2}, n0{calcNormal()}, n1{calcNormal()}, n2{calcNormal()},
              faceNormal{calcNormal()}
    { }
    Triangle(const Vec3f& v0, const Vec3f& v1, const Vec3f& v2, const Material &material=Material{})
            : Primitive{material}, v0{v0}, v1{v1}, v2{v2}, n0{calcNormal()}, n1{calcNormal()}, n2{calcNormal()},
              faceNormal{calcNormal()}
    { }
    Triangle(const Vec3f& v0, const Vec3f& v1, const Vec3f& v2,
             const Vec3f& n0, const Vec3f& n1, const Vec3f& n2,
             const Material& material=Material{})
            : Primitive{material}, v0{v0}, v1{v1}, v2{v2}, n0{n0}, n1{n1}, n2{n2}, faceNormal{calcNormal()}{ }

    bool intersect(const Ray& ray, Float& dist, Vec3f& normal) const override;

    Vec3f getNormal(const Vec3f& vec) const override;

    Vec3f calcNormal();

    void scale(Float s);

    Triangle& operator+=(Float t);

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

    Sphere(const Vec3f& c, Float r, const Color& color) : Primitive{color}, center{c}, radius{r} {}
    Sphere(const Vec3f& c, Float r, const Material& material) : Primitive{material}, center{c}, radius{r} {}
    Sphere() : Sphere({0,0,0}, 1, Material{}) {}

    bool intersect(const Ray& ray, Float& dist, Vec3f& normal) const override;

    Vec3f getNormal(const Vec3f& P) const override;
};

struct Model : Primitive {
    std::vector<Triangle> faces;

    /** Bounding box volume used for optimization. */
    Sphere bbvol;

    Model(const Color& color, const std::vector<Triangle>& faces) : Primitive(color), faces(faces), bbvol() { updateBBVol(); }
    Model(const Material& material, const std::vector<Triangle>& faces) : Primitive(material), faces(faces),
                                                                     bbvol() { updateBBVol(); }
    explicit Model(const Color& color) : Primitive(color), bbvol() {}
    explicit Model(const Material& material) : Primitive(material), bbvol() {}
    explicit Model() : bbvol() {}

    void updateBBVol();

    static Model* load_ply(const std::string& fname, const Material& material, bool calcNormal=false);

    bool intersect(const Ray& ray, Float& dist, Vec3f& normal) const override;

    Model& scale(Float s);

    Model& translate(const Vec3f& t);

    Vec3f getNormal(const Vec3f& vec) const override;

    Model& operator*=(const Mat3d& mat);

    Model& operator+=(const Vec3f& rhs);

    friend Model operator+(Model lhs, const Vec3f& rhs) {
        return lhs += rhs;
    }

    friend Model operator*(Model lhs, const Mat3d& rhs) {
        return lhs *= rhs;
    }


};

//
// Created by chris on 13.02.18.
//

#ifndef RAYTRACER_MODEL_H
#define RAYTRACER_MODEL_H

#include "primitives.h"
#include "container.h"

struct Model {
    std::vector<std::shared_ptr<Triangle>> faces;
    Material material;

    /** Bounding volume used for optimization. */
    Sphere bvol;

    Model(const Color& color, const std::vector<std::shared_ptr<Triangle>>& faces) : material(color), faces(faces), bvol() { updateBBVol(); }
    Model(const Material& material, const std::vector<std::shared_ptr<Triangle>>& faces) : material(material), faces(faces),
                                                                          bvol() { updateBBVol(); }
    explicit Model(const Color& color) : material(color), bvol() {}
    explicit Model(const Material& material) : material(material), bvol() {}
    explicit Model() : bvol() {}

    void updateBBVol();

    static Model* load_ply(const std::string& fname, const Material& material, bool calcNormal=false);

    bool intersect(const Ray& ray, Float& dist, Vec3f& normal) const;

    Model& scale(Float s);

    Model& translate(const Vec3f& t);

    Model& operator*=(const Mat3d& mat);

    Model& operator+=(const Vec3f& rhs);

    friend Model operator+(Model lhs, const Vec3f& rhs) {
        return lhs += rhs;
    }

    friend Model operator*(Model lhs, const Mat3d& rhs) {
        return lhs *= rhs;
    }


};



#endif //RAYTRACER_MODEL_H

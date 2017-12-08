//
// Created by chris on 10.10.17.
//

#ifndef RAYTRACER_SCENE_H
#define RAYTRACER_SCENE_H

#include "primitives.h"
#include <vector>
#include <memory>
#include <string>


struct Light {
    Vec3f pos;
    Color color;

    explicit Light(const Vec3f &pos) : Light{pos, color} {}
    Light(const Vec3f &pos, const Color& color) : pos{pos}, color{color} {}

};

class Scene {
public:
    explicit Scene(const std::string& filename);

    const std::vector<std::unique_ptr<Primitive>> &getObjects() const;
    const std::vector<Light> &getLights() const;

private:
    std::vector<std::unique_ptr<Primitive>> objects;
    std::vector<Light> lights;
};

#endif //RAYTRACER_SCENE_H

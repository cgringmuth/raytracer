//
// Created by chris on 10.10.17.
//

#ifndef RAYTRACER_SCENE_H
#define RAYTRACER_SCENE_H

#include "primitives.h"
#include <vector>
#include <memory>

class Scene {
public:
    Scene();

    const std::vector<std::shared_ptr<Primitive>> &getObjects() const;

private:
    std::vector<std::shared_ptr<Primitive>> objects;
};

#endif //RAYTRACER_SCENE_H

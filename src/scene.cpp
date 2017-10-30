//
// Created by chris on 10.10.17.
//

#include "scene.h"

Scene::Scene() {
    // TODO: loading from YAML file

}

const std::vector<std::shared_ptr<Primitive>> &Scene::getObjects() const {
    return objects;
}

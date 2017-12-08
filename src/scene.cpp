//
// Created by chris on 10.10.17.
//
#include "opencv2/opencv.hpp"
#include "scene.h"
#include <string>

Scene::Scene(const std::string& filename) {
    // TODO: loading from YAML file
    cv::FileStorage fs(filename, cv::FileStorage::READ);


    fs.release();
}

const std::vector<std::unique_ptr<Primitive>> &Scene::getObjects() const {
    return objects;
}

const std::vector<Light> &Scene::getLights() const {
    return lights;
}

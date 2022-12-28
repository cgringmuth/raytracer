//
// Created by chris on 10.10.17.
//
#include "opencv2/opencv.hpp"
#include "scene.h"
#include "color.h"
#include "material.h"
#include <string>
#include <map>

template <typename T>
std::ostream &operator<<(std::ostream& os, const std::pair<const std::string, T>& pt) {
    return os << "[" << pt.first << "]: " << pt.second;
}

template <typename T>
std::ostream &operator<<(std::ostream& os, const std::map<std::string, T>& mapT) {
    for (const auto & elem : mapT) {
        os << elem << '\n';
    }

    return os;
}

template <typename T>
inline void read(const cv::FileNode& node, Vec3<T>& vec) {
    vec[0] = node[0];
    vec[1] = node[1];
    vec[2] = node[2];
}

void parseColors(const cv::FileStorage& fs, std::map<std::string, Color>& colors) {
    std::map<std::string, Color> colorsTmp;

    const cv::FileNode colorsNode = fs["colors"];
    for (const auto & colorNode : colorsNode) {
        const std::string name = colorNode.name();
        Color color;
        if (colorNode.isSeq()) {
            Vec3f vec;
            read(colorNode, vec);
            color = Color{vec};
        } else {
            color = Color{colorNode};
        }

        colorsTmp[name] = color;
    }

    std::swap(colorsTmp, colors);
}

std::map<std::string, Color> parseColors(const cv::FileStorage& fs) {
    std::map<std::string, Color> colors;
    parseColors(fs, colors);
    return colors;
}

void parseMaterial(const cv::FileStorage& fs,
                  const std::map<std::string, Color>& colors,
                  std::map<std::string, Material>& materials
) {
    std::map<std::string, Material> materialsTmp;
    const cv::FileNode materialsNode = fs["materials"];
    for (const auto & materialNode : materialsNode) {
        const std::string name = materialNode.name();

        Material material;
        for (const auto & elem : materialNode) {
            const std::string elemName = elem.name();
            if (elemName == "color") {
                const Color color = colors.find(std::string(elem))->second;     //FIXME: assumes color exists
                material.color = color;
            } else if (elemName == "ka") {
                material.ka = elem;
            } else if (elemName == "kd") {
                material.kd = elem;
            } else if (elemName == "ks") {
                material.ks = elem;
            } else if (elemName == "kr") {
                material.kr = elem;
            } else if (elemName == "kt") {
                material.kt = elem;
            } else if (elemName == "specRefExp") {
                material.specRefExp = elem;
            } else if (elemName == "refractiveIdx") {
                material.refractiveIdx = elem;
            } else if (elemName == "reflective") {
                material.reflective = std::string(elem) == "true";
            } else if (elemName == "refractive") {
                material.refractive = std::string(elem) == "true";
            } else {
                std::cerr << elemName << " not supported";
                throw std::runtime_error{"Not supported type"};
            }
        }
        materialsTmp[name] = material;
    }

    std::swap(materialsTmp, materials);
}


std::map<std::string, Material> parseMaterial(const cv::FileStorage& fs,
                                              const std::map<std::string, Color>& colors) {
    std::map<std::string, Material> materials;
    parseMaterial(fs, colors, materials);
    return materials;
}


void parseLight(const cv::FileStorage& fs, const std::map<std::string, Color>& colors, std::vector<Light>& lights) {
    std::vector<Light> lightsTmp;

    const cv::FileNode lightsNode = fs["lights"];
    for (const auto & lightNode : lightsNode) {
        Vec3f pos;
        read(lightNode["pos"], pos);
        float factor;
        cv::read(lightNode["factor"], factor, 1.0);
        const Color color = colors.find(std::string(lightNode["color"]))->second * factor;     //FIXME: assumes color exists
        const Light light = Light{pos, color};
        lightsTmp.push_back(light);
    }

    std::swap(lightsTmp, lights);
}

std::vector<Light> parseLight(const cv::FileStorage& fs, const std::map<std::string, Color>& colors) {
    std::vector<Light> lights;
    parseLight(fs, colors, lights);
    return lights;
}


void parsePrimitives(const cv::FileStorage& fs, const std::map<std::string, Material>& materials,
                     std::vector<std::unique_ptr<Primitive>>& primitives) {
    const cv::FileNode primitivesNode = fs["primitives"];
    std::vector<std::unique_ptr<Primitive>> primitivesTmp;

    for (const auto& primitiveNode : primitivesNode) {
        const std::string type = std::string(primitiveNode["type"]);
        const Material material = materials.find(primitiveNode["material"])->second;
        std::unique_ptr<Primitive> prim;
        if (type == "sphere") {
            const float radius = static_cast<float>(primitiveNode["radius"]);
            Vec3f center;
            read(primitiveNode["center"], center);
            prim.reset(new Sphere{center, radius, material});
        } else if (type == "mesh") {
            Vec3f translation, rotation;
            read(primitiveNode["translation"], translation);
            read(primitiveNode["rotation"], rotation);
            const auto scale = static_cast<double>(primitiveNode["scale"]);
            const std::string filename = std::string(primitiveNode["filename"]);
            Model* model = Model::load_ply(filename, material, true);
            if (model == nullptr) {
                std::cerr << "Couldn't load model: " << filename << '\n';
                throw std::runtime_error{"Couldn't load model."};
            }
            *model *= Mat3d::rotation(rotation);
            model->scale(scale);
            *model += translation;
            prim.reset(model);
        } else if (type == "plane") {
            Vec3f normal;
            read(primitiveNode["normal"], normal);
            const auto d = static_cast<double>(primitiveNode["distance"]);
            prim.reset(new Plane{normal, d, material});
        } else {
            std::cerr << "type: " << type << " is not supported\n";
            throw std::runtime_error{"Type is not supported"};
        }
        primitivesTmp.push_back(std::move(prim));
    }

    std::swap(primitivesTmp, primitives);
}



Scene::Scene(const std::string& filename) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);

    const std::map<std::string, Color> colors = parseColors(fs);
    const std::map<std::string, Material> materials = parseMaterial(fs, colors);
    lights = parseLight(fs, colors);
    parsePrimitives(fs, materials, objects);
}

const std::vector<std::unique_ptr<Primitive>> &Scene::getObjects() const {
    return objects;
}

const std::vector<Light> &Scene::getLights() const {
    return lights;
}

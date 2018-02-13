//
// Created by chris on 13.02.18.
//

#include "model.h"
#include "common.h"
#include <fstream>
#include <limits>


void Model::updateBBVol() {
    Vec3f center{0,0,0};
    Float radius{0}, tmp{0};
    const unsigned int numVertices{faces.size()*3};
    for (const auto& f : faces) {
        const Vec3f v0{f->v0};
        const Vec3f v1{f->v1};
        const Vec3f v2{f->v2};
        center += v0;
        center += v1;
        center += v2;
    }
    center /= numVertices;
    for (const auto& f : faces) {
        const Vec3f v0{f->v0};
        const Vec3f v1{f->v1};
        const Vec3f v2{f->v2};
        tmp = (center - v0).length();
        if (tmp > radius) {
            radius = tmp;
        }
        tmp = (center - v1).length();
        if (tmp > radius) {
            radius = tmp;
        }
        tmp = (center - v2).length();
        if (tmp > radius) {
            radius = tmp;
        }
    }
    bvol = Sphere{center, radius, Color{0}};
}

Model* Model::load_ply(const std::string &fname, const Material &material, bool calcNormal) {
    std::cout << "... loading model: " << fname << std::endl;

    std::ifstream ifs{fname};
    if (!ifs.is_open())
        throw std::runtime_error{"Model "+fname+" could not be loaded. File does not exists."};

    std::string line, key;
    unsigned int val;
    std::getline(ifs, line);
    bool ply_type{false};

    unsigned int nfaces, nvertices;

    while (true) {
        std::stringstream ss{line};
        ss >> key;
        if (key == "ply") {
            ply_type = true;
        } else if (key == "comment") {
            // ignore for now
        } else if (key == "end_header") {
            break;
        } else if (key == "element") {
            ss >> key >> val;
            if (key == "vertex") {
                nvertices = val;
            } else if (key == "face") {
                nfaces = val;
            }
        }
        // ignore all other keys for now
        std::getline(ifs, line);
    }

    std::cout << OUTPUT(nfaces) << std::endl;

    // assume coordinates x, y, z come first and ignore all other following values
    std::vector<Vec3f> vertices, normals;
    std::vector<std::vector<unsigned int>> face_idx;
    std::vector<std::shared_ptr<Triangle>> faces;

    // read vertices
    for (int i = 0; i < nvertices; ++i) {
        std::getline(ifs, line);
        std::stringstream ss{line};
        Float x, y, z;
        ss >> x >> y >> z;
        vertices.emplace_back(Vec3f{x, y, z});
    }

    const bool center_model = true;
    if (center_model) {
        Vec3f center{0,0,0};
        for (const auto& v : vertices) {
            center += v;
        }
        center /= nvertices;
        std::cout << "center: " << center << '\n';

        for (auto& v : vertices) {
            v -= center;
        }

        center = {0,0,0};
        for (const auto& v : vertices) {
            center += v;
        }
        center /= nvertices;
        std::cout << "center: " << center << '\n';

    }

    // read faces indices
    for (int i = 0; i < nfaces; ++i) {
        getline(ifs, line);
        std::stringstream ss{line};
        unsigned int num, iv0, iv1, iv2;
        ss >> num >> iv0 >> iv1 >> iv2;
        if (num != 3) {
            throw std::runtime_error{"Only triangles are supported"};
        }
        face_idx.emplace_back(std::vector<unsigned int>{iv0, iv1, iv2});
    }


    if (calcNormal) {
        // interpolate normals between all vertices
        normals.resize(vertices.size());
        for (const auto& fidx : face_idx) {
            const Vec3f v0{vertices[fidx[0]]};
            const Vec3f v1{vertices[fidx[1]]};
            const Vec3f v2{vertices[fidx[2]]};
            const Vec3f v0v1 = Vec3f::getVec(v0, v1);
            const Vec3f v0v2 = Vec3f::getVec(v0, v2);
            const Vec3f faceNormal = Vec3f::getNormal(v0v1, v0v2);
            normals[fidx[0]] += faceNormal;
            normals[fidx[1]] += faceNormal;
            normals[fidx[2]] += faceNormal;
        }
        // normalize all normals
        unsigned int index = 0;
        for (auto& normal : normals) {
            normal.normalize();
            if (raytracer::isnan(normal)) {
                std::cerr << "Normal is nan:" << normal << " index: " << index << '\n';
            }
            ++index;
        }
    }

    // create faces
    for (const auto& fixd : face_idx) {
        const unsigned int iv0{fixd[0]};
        const unsigned int iv1{fixd[1]};
        const unsigned int iv2{fixd[2]};

        if (calcNormal) {
            faces.emplace_back(new Triangle{vertices[iv0], vertices[iv1], vertices[iv2],
                                        normals[iv0], normals[iv1], normals[iv2], material});
        } else {
            faces.emplace_back(new Triangle{vertices[iv0], vertices[iv1], vertices[iv2], material});
        }
    }

    return new Model(material, faces);
}

bool Model::intersect(const Ray &ray, Float &dist, Vec3f &normal) const {
    // todo: through all faces and give closest distance which is not negative and return normal also (for all)
    Float tmpdist;
    Vec3f tmpnormal;
    dist = std::numeric_limits<Float>::max();
    bool hit{false};
    if (!bvol.intersect(ray, tmpdist, tmpnormal)) {
        return false;
    }
//        hit = true;
//        normal = tmpnormal;
//        dist = tmpdist;
    for (const auto& f : faces) {
        if (f->intersect(ray, tmpdist, tmpnormal) && tmpdist < dist) {
            dist = tmpdist;
            normal = tmpnormal;
            hit = true;
        }
    }
    return hit && dist > EPS;
}

Model &Model::scale(Float s) {
    for (auto& f : faces) {
        f->scale(s);
    }
//        bvol.radius *= s;
    updateBBVol();
    return *this;
}

Model &Model::translate(const Vec3f &t) {
    return *this += t;
}

Model &Model::operator*=(const Mat3d &mat) {
    for (auto& f : faces) {
        *f *= mat;
    }
    return *this;
}

Model &Model::operator+=(const Vec3f &rhs) {
    for (auto& f : faces) {
        *f += rhs;
    }
    bvol.center += rhs;
    return *this;
}
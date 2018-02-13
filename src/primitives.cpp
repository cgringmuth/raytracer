#include <limits>
#include "primitives.h"
#include "container.h"
#include "common.h"
#include <fstream>
#include <assert.h>

//
// Created by chris on 19.08.17.
//
bool Plane::intersect(const Ray &ray, Float &dist, Vec3f &normal) const {
    //src: https://www.scratchapixel.com/lessons/3d-basic-rendering/ray-tracing-rendering-a-triangle/ray-triangle-intersection-geometric-solution
    normal = Vec3f{a, b, c};
    const Float vd{normal.dotProduct(ray.direction)};

    if (abs(vd) < EPS)  // check if vd is 0 -> plane and ray parallel
        false;
    if (vd > 0)     // normal of plane is pointing away from camera (maybe handle differently)
        false;

    const Float vo{normal.dotProduct(ray.origin) + d};
    dist = -vo / vd;

    return dist > EPS;
}

Vec3f Plane::getNormal(const Vec3f &vec) const {
    return Vec3f{a, b, c};
}

/** Ray/Triangle intersection
 * Check if ray intersects triangle and calculates distance and normal (which is interpolated from all triangle
 * vertices)
 *
 * //TODO: optimization: Ray-Triangle Intersection Algorithm for Modern CPU Architectures. https://cadxfem.org/inf/Paper_46.pdf
 *
 * @param ray   The ray, which will be checked for intersection (r(t) = o + dt)
 * @param dist  The is "t" of the ray.
 * @param normal    The normal at the intersection point
 * @return  True when ray intersects triangle. Only then normal and distance are meaningful.
 */
bool Triangle::intersect(const Ray &ray, Float &dist, Vec3f &normal) const {
    normal = n1;
#if MT_TRIANGLE_INTERSECT==1
    // "Fast, minimum storage ray/triangle intersection" by
    // src: http://www.cs.virginia.edu/%7Egfx/Courses/2003/ImageSynthesis/papers/Acceleration/Fast%20MinimumStorage%20RayTriangle%20Intersection.pdf
    // Any point on the triangle can be described by (in the uv-space)
    // P = v0 + u(v1-v0) + v(v2-v0)
    // P can also be described by ray equation (when hit triangle)
    // P = o + td
    // Note: u and v can be used for texture mapping and normal interpolation
    const Vec3f v0v1{v1-v0};    // edge 1 from v0 to v1
    const Vec3f v0v2{v2-v0};    // edge 2 from v0 to v2
    const Vec3f pVec{ray.direction.cross_product(v0v2)};
    const Float det{pVec.dotProduct(v0v1)};

    // if determinant is negative, triangle is backfacing
    // if determinant is close to 0, ray is missing triangle
#if CULLING == 1
    if (det < EPS)
        return false;
#else
    if (abs(det) < EPS)
        return false;
#endif
    const Float invDet = 1 / det;

    // calc u
    const Vec3f tVec{ray.origin-v0};
    const Float u{invDet * pVec.dotProduct(tVec)};
    if (u < 0 || u > 1)
        return false;

    // calc v
    const Vec3f qVec{tVec.cross_product(v0v1)};
    const Float v{invDet * qVec.dotProduct(ray.direction)};
    if (v < 0 || u+v > 1)
        return false;

    // calc dist
    dist = invDet * qVec.dotProduct(v0v2);

    normal = (1-u-v)*n0 + u*n1 + v*n2;  // interpolate normal based on intersection point
//        normal.normalize();
#else
//        cout << normal << endl;
//        cout << "v0: " << v0 << "v1: " << v1 << "v2: " << v2 << endl;
    Plane plane{normal,
                -normal.dotProduct(v1),  // todo: check if this is correct (seems more as an workaround)
                Color::white()};
//        cout << ray.direction << endl;

    if (!plane.intersect(ray, dist, normal)) {
        return false;
    }

    const Vec3f hit{ray.getPoint(dist)}; // get point on plane

//        cout << "hit: " << hit << " hit length:" << hit.length() << endl;

    // do the "inside-outside" test
    // check if intersection point is on left side of each edge

    const Vec3f edge0{v1 - v0};
    const Vec3f vp0{hit - v0};
//        cout << "edge0: " << edge0 << " vp0: " << vp0 << endl;
    if (dotProduct(cross_product(edge0, vp0), normal) < 0) {
        return false;
    }

    const Vec3f edge1{v2 - v1};
    const Vec3f vp1{hit - v1};
//        cout << "edge1: " << edge1 << " vp1: " << vp1 << endl;
    if (dotProduct(cross_product(edge1, vp1), normal) < 0) {
        return false;
    }

    const Vec3f edge2{v0 - v2};
    const Vec3f vp2{hit - v2};
//        cout << "edge2: " << edge2 << " vp2: " << vp2 << endl;
    if (dotProduct(cross_product(edge2, vp2), normal) < 0) {
        return false;
    }

//        cout << "dist: " << eps << endl;

#endif
    return dist > EPS;
}

Vec3f Triangle::getNormal(const Vec3f &vec) const {
    return faceNormal;  // todo: depending on shading mode (flat shading etc.) it should return faceNormal or interpolate
}

Vec3f Triangle::calcNormal() {
    // todo: with one normal for each vertex this is not correct anymore
    const Vec3f v0v1 = Vec3f::getVec(v0, v1);
    const Vec3f v0v2 = Vec3f::getVec(v0, v2);
    n0 = cross_product(v0v1, v0v2);
    n0.normalize();
    return n0;
}

void Triangle::scale(Float s) {
    v0 *= s;
    v1 *= s;
    v2 *= s;
}

Triangle &Triangle::operator+=(const Float t) {
    v0 += t;
    v1 += t;
    v2 += t;
    return *this;
}

Triangle &Triangle::operator+=(const Vec3f &v) {
    v0 += v;
    v1 += v;
    v2 += v;
    return *this;
}

Triangle &Triangle::operator*=(const Mat3d &mat) {
    v0 *= mat;
    v1 *= mat;
    v2 *= mat;
    n0 *= mat;
    n1 *= mat;
    n2 *= mat;
    return *this;
}

// get intersection with ray: refer: https://en.wikipedia.org/wiki/Line%E2%80%93sphere_intersection
// more details: https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-sphere-intersection
bool Sphere::intersect(const Ray &ray, Float &dist, Vec3f &normal) const {
    // (l * (o - c))^2 - || o - c ||^2 + r^2
    const Vec3f temp{ray.origin - center};
    const Float val1 = temp.dotProduct(ray.direction);
    const Float val2 = temp.length();
    const Float val3 = val1 * val1 - val2 * val2 + radius * radius;

    if (val3 < 0) {
        return false;
    }

    // compute distance
    const Float dist1 = -val1 + sqrt(val3);
    const Float dist2 = -val1 - sqrt(val3);

    if (dist1 < 0 || dist2 < 0) {
        dist = std::max(dist1, dist2);
    } else if (dist1 > 0 && dist2 > 0) {
        dist = std::min(dist1, dist2);
    }
    normal = getNormal(ray.getPoint(dist));

    return dist > EPS;      //  neg. dist are behind ray; eps is for not hitting itself
}

Vec3f Sphere::getNormal(const Vec3f &P) const {
    // src: https://cs.colorado.edu/~mcbryan/5229.03/mail/110.htm
    Vec3f n{P - center};
    n /= radius;
    return n;
}

void Model::updateBBVol() {
    Vec3f center{0,0,0};
    Float radius{0}, tmp{0};
    const unsigned int numVertices{faces.size()*3};
    for (const auto& f : faces) {
        const Vec3f v0{f.v0};
        const Vec3f v1{f.v1};
        const Vec3f v2{f.v2};
        center += v0;
        center += v1;
        center += v2;
    }
    center /= numVertices;
    for (const auto& f : faces) {
        const Vec3f v0{f.v0};
        const Vec3f v1{f.v1};
        const Vec3f v2{f.v2};
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
    bbvol = Sphere{center, radius, Color{0}};
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

    // assume coordinates x, y, z come first and ignore all other following values
    std::vector<Vec3f> vertices, normals;
    std::vector<std::vector<unsigned int>> face_idx;
    std::vector<Triangle> faces;

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
            faces.emplace_back(Triangle{vertices[iv0], vertices[iv1], vertices[iv2],
                                        normals[iv0], normals[iv1], normals[iv2], material});
        } else {
            faces.emplace_back(Triangle{vertices[iv0], vertices[iv1], vertices[iv2], material});
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
    if (!bbvol.intersect(ray, tmpdist, tmpnormal)) {
        return false;
    }
//        hit = true;
//        normal = tmpnormal;
//        dist = tmpdist;
    for (const auto& f : faces) {
        if (f.intersect(ray, tmpdist, tmpnormal) && tmpdist < dist) {
            dist = tmpdist;
            normal = tmpnormal;
            hit = true;
        }
    }
    return hit && dist > EPS;
}

Model &Model::scale(Float s) {
    for (auto& f : faces) {
        f.scale(s);
    }
//        bbvol.radius *= s;
    updateBBVol();
    return *this;
}

Model &Model::translate(const Vec3f &t) {
    return *this += t;
}

Vec3f Model::getNormal(const Vec3f &vec) const {
    return Vec3f{};     // fixme: currently broken
}

Model &Model::operator*=(const Mat3d &mat) {
    for (auto& f : faces) {
        f *= mat;
    }
    return *this;
}

Model &Model::operator+=(const Vec3f &rhs) {
    for (auto& f : faces) {
        f += rhs;
    }
    bbvol.center += rhs;
    return *this;
}

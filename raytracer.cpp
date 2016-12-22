#include <iostream>
#include <math.h>
#include <fstream>
#include <vector>
#include <memory>
#include <limits>


// url: https://www.scratchapixel.com/lessons/3d-basic-rendering/introduction-to-ray-tracing
// operator overloading: http://stackoverflow.com/a/4421719/1959528
// 3D meshes: https://graphics.stanford.edu/data/3Dscanrep/
// Idea: Create the cornell box as test scene: https://en.wikipedia.org/wiki/Cornell_box


/**
 * TODOs
 *
 * - Implement triangle class with intersect and normal
 * - Add material with different reflection models (diffuse, specular, refraction etc.)
 * - Implement ply file loading
 *
 */




using namespace std;

/** 3D vector in cartesian space.
 *
 */
struct Vec3d {
    double x, y, z;

    Vec3d() : x{0}, y{0}, z{0} {}

    Vec3d(double x, double y, double z) : x{x}, y{y}, z{z} {}

    Vec3d(const Vec3d& v) : x{v.x}, y{v.y}, z{v.z} {}

    Vec3d& operator-=(const Vec3d& rhs) {
        x -= rhs.x;
        y -= rhs.y;
        z -= rhs.z;
        return *this;
    }

    Vec3d& operator/=(const double v) {
        x /= v;
        y /= v;
        z /= v;
        return *this;
    }

    Vec3d& operator+=(const Vec3d& rhs) {
        x += rhs.x;
        y += rhs.y;
        z += rhs.z;
        return *this;
    }

    Vec3d& operator*=(double val) {
        x *= val;
        y *= val;
        z *= val;
        return *this;
    }

    double& operator[](size_t idx) {
        switch (idx) {
            case 1:
                return x;
            case 2:
                return y;
            case 3:
                return z;
        }
    }

    const double& operator[](size_t idx) const {
        switch (idx) {
            case 0:
                return x;
            case 1:
                return y;
            case 2:
                return z;
        }
    }

    double dotProduct(const Vec3d& vec2) const {
        return (x * vec2.x) +
               (y * vec2.y) +
               (z * vec2.z);
    }

    double length() const {
        return sqrt(dotProduct(*this));
    }

    Vec3d& normalize() {
        *this /= length();
        return *this;
    }

    Vec3d cross_product(const Vec3d& v) const;
};

Vec3d cross_product(const Vec3d& u, const Vec3d& v) {
    return Vec3d{
            u[1] * v[2] - u[2] * v[1],
            u[2] * v[0] - u[0] * v[2],
            u[0] * v[1] - u[1] * v[0]
    };
}

Vec3d Vec3d::cross_product(const Vec3d& v) const {
    return ::cross_product(*this, v);
}


Vec3d operator-(Vec3d lhs, const Vec3d& rhs) {
    return lhs -= rhs;
}

Vec3d operator/(Vec3d lhs, const double v) {
    return lhs /= v;
}

Vec3d operator+(Vec3d lhs, const Vec3d& rhs) {
    return lhs += rhs;
}

Vec3d operator*(Vec3d lhs, const double val) {
    return lhs *= val;
}


/** Clips the value to min and max.
 * If the input value lies in [min,max], it will not be changed. Otherwise it will be set to min if val < min or to
 * max if val > max.
 * @param min Min value
 * @param max Max value
 * @param val Input value
 * @return The clipped value
 */
template<typename T>
T clamp(T min, T max, T val) {
    val = val < min ? min : val;
    val = val > max ? max : val;
    return val;
}

/** Container to save pixel color information in rgb format.
 *
 */
struct Color {
    double r, g, b;

    Color() : r{0}, g{0}, b{0} {};

    Color(double r_, double g_, double b_) : r{r_}, g{g_}, b{b_} {};

    Color& operator*=(double d) {
        r *= d;
        g *= d;
        b *= d;
        return *this;
    }

    Color& operator*=(Color c) {
        r *= c.r;
        g *= c.g;
        b *= c.b;
        return *this;
    }

    Color& operator+=(const Color& rhs) {
        r += rhs.r;
        g += rhs.g;
        b += rhs.b;
        return *this;
    }

    Color& mult(double d) {
        r *= d;
        g *= d;
        b *= d;
        return *this;
    }

    Color& clamp(double min, double max) {
        r = ::clamp(min, max, r);
        g = ::clamp(min, max, g);
        b = ::clamp(min, max, b);
        return *this;
    }

    Color& round() {
        r = ::round(r);
        g = ::round(g);
        b = ::round(b);
        return *this;
    }

    static Color white() {
        return Color(1, 1, 1);
    }

    static Color black() {
        return Color();
    }

    static Color red() {
        return Color(1, 0, 0);
    }
};

Color operator+(Color lhs, const Color& rhs) {
    return lhs += rhs;
}

Color operator*(Color lhs, const double d) {
    return lhs *= d;
}

Color operator*(Color lhs, const Color& rhs) {
    return lhs *= rhs;
}


/** Line which will be used for back raytracing
 *
 */
struct Ray {
    // origin
    Vec3d origin;
    // direction
    Vec3d direction;

    Ray(Vec3d o, Vec3d d) : origin{o}, direction{d} {}

    Vec3d getPoint(double dist) const {
        return origin + direction * dist;
    }
};


/** Generic base class for all objects which shall be rendered
 *
 */
struct Object {
    Color color;

    Object(const Color& color) : color(color) {}

    Object() : color{} {}

    virtual bool intersect(const Ray& ray, double& dist, Vec3d& normal) const = 0;

    virtual Vec3d getNormal(const Vec3d& vec) const = 0;

};


struct Plane : public Object {
    double a, b, c, d;  // implicit description: ax + by + cz + d = 0

    Plane(double a, double b, double c, double d, const Color& color)
            : a(a), b(b), c(c), d(d), Object(color) {}

    bool intersect(const Ray& ray, double& dist, Vec3d& normal) const override {
        return false;
    }

    Vec3d getNormal(const Vec3d& vec) const override {
        return Vec3d{};
    }
};

struct Triangle : public Object {
    Vec3d v1, v2, v3;

    Triangle(const Vec3d& tv1, const Vec3d& tv2, const Vec3d& tv3, const Color& color)
            : v1{tv1}, v2{tv2}, v3{tv3}, Object{color} {}

    virtual bool intersect(const Ray& ray, double& dist, Vec3d& normal) const override {

    }

    virtual Vec3d getNormal(const Vec3d& vec) const override {

    }
};

struct Model : Object {
    vector<Triangle> faces;

    Model(const Color& color, const vector<Triangle>& faces) : Object(color), faces(faces) {}

    Model(const Color& color) : Object(color) {}

    Model() {}

    bool intersect(const Ray& ray, double& dist, Vec3d& normal) const override {
        // todo: through all faces and give closest distance which is not negative and return normal also (for all)
        return false;
    }

    Vec3d getNormal(const Vec3d& vec) const override {
        return Vec3d{};
    }

};

namespace ply {
    Model load_model(string filename);
}

Model ply::load_model(string filename) {
    Model model;

    // parse header

    // parse content

    return model;
}

/** The most common object to be rendered in a raytracer
 *
 */
struct Sphere : public Object {
    // define location + radius
    Vec3d center;
    double radius;

    Sphere(const Vec3d& c, double r, const Color& color) : center{c}, radius{r}, Object{color} {}

    // get intersection with ray: refer: https://en.wikipedia.org/wiki/Line%E2%80%93sphere_intersection
    // more details: https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-sphere-intersection
    virtual bool
    intersect(const Ray& ray, double& dist, Vec3d& normal) const override {
        constexpr double eps = 0.00001;
        // (l * (o - c))^2 - || o - c ||^2 + r^2
        double val1, val2, val3, dist1, dist2;
        const Vec3d temp{ray.origin - center};
        val1 = temp.dotProduct(ray.direction);
        val2 = temp.length();
        val3 = val1 * val1 - val2 * val2 + radius * radius;

        if (val3 < 0) {
            return false;
        }

        // compute distance
        dist1 = -temp.dotProduct(ray.direction) + sqrt(val3);
        dist2 = -temp.dotProduct(ray.direction) - sqrt(val3);

        if (dist1 < 0 || dist2 < 0) {
            dist = max(dist1, dist2);
        } else if (dist1 > 0 && dist2 > 0) {
            dist = min(dist1, dist2);
        }

        return dist > eps;      //  neg. dist are behind ray; eps is for not hitting itself
    }

    virtual Vec3d
    getNormal(const Vec3d& P) const override {
        // src: https://cs.colorado.edu/~mcbryan/5229.03/mail/110.htm
        Vec3d n{P - center};
        n /= radius;
        return n;
    }
};

struct Light {
    Vec3d pos;
    Color color;

    Light(Vec3d pos_, Color color_) : pos{pos_}, color{color_} {}

    Light(Vec3d pos_) : pos{pos_}, color{Color::white()} {}
};


ostream&
operator<<(ostream& os, const Color& c) {
    os << c.r << " " << c.g << " " << c.b << " ";
    return os;
}

ostream&
operator<<(ostream& os, const Vec3d& v) {
    os << v.x << " " << v.y << " " << v.z << " ";
    return os;
}


/** Converts angle in degree into rad.
 *
 * @param ang angle in degree
 * @return angle in rad
 */
double deg2rad(double ang) {
    return ang * M_PI / 180;
}


void check_op_overloading() {
    Vec3d v1, v2{1, 2, 3};

    v1 += v2;
    cout << v1 << " == 1 2 3\n";
    cout << v1 * 2 << " == 2 4 6\n";
//    cout << v1+2 << " == 3 4 5\n";

    Color c1, c2{1, 2, 3};
    c1 += c2;
    cout << c1 << " == 1 2 3\n";
    cout << c1 * c2 << " == 1 4 9\n";
    cout << c1 * 2 << " == 2 4 6\n";

    const Vec3d e1{Vec3d{1, 0, 0}};
    const Vec3d e2{Vec3d{0, 1, 0}};
    const Vec3d e3{Vec3d{0, 0, 1}};
    const Vec3d cp{cross_product(e1, e2)};
    cout << "cross product: " << cp << " == 0 0 1\n";
    cout << "cross product angle should be 90 deg => 1): " << cos(cp.dotProduct(e1)) << " == 1 \n";


}


int
main(int argc, char** argv) {
    cout << "... start ray tracer" << endl;
    check_op_overloading();

    constexpr unsigned int H = 500;
    constexpr unsigned int W = 800;
    constexpr unsigned int MAX_VAL = 255;
    constexpr double ASPECT_RATIO = (double) W / H;
    constexpr double FOV = 100;

    ofstream ofs{"out4.ppm"};    // http://netpbm.sourceforge.net/doc/ppm.html
    ofs << "P3\n"
        << to_string(W) << " " << to_string(H) << "\n"
        << to_string(MAX_VAL) << "\n";

    Color background{0, 0.5, 0.5};
    Color scolor = Color::red();


    vector<shared_ptr<Object>> objects;
    objects.push_back(make_shared<Sphere>(Vec3d{0, 0, -20}, 5, scolor));
//    objects.push_back(make_shared<Sphere>(Vec{10,0,-20}, 5, scolor));
    objects.push_back(make_shared<Sphere>(Vec3d{2, 1, -15}, 1, Color{1, 1, 0}));
    objects.push_back(make_shared<Sphere>(Vec3d{4, 4, -22}, 2.5, Color{0, 1, 0}));
    objects.push_back(make_shared<Sphere>(Vec3d{80, -6, -150}, 5, Color{0, 0, 1}));
    objects.push_back(make_shared<Sphere>(Vec3d{-4, 4, -5}, 2.5, Color{1, 0, 1}));

//    s.radius = 10;

    Color* img = new Color[W * H];
    double* zbuff = new double[W * H];
    Color* img_ptr = img;
    const Vec3d origin{0, 0, 0};  // center of projection

    vector<Light> lights;
    lights.emplace_back(Light{Vec3d{30, 30, -2}, Color::white()});
//    lights.emplace_back(Light{Vec{-30,-20,1}});

    img_ptr = img;
    for (unsigned int y = 0; y < H; ++y) {
        for (unsigned int x = 0; x < W; ++x) {

            const double px_ndc = (x + 0.5) / W;
            const double py_ndc = (y + 0.5) / H;
            const double cam_x = (2 * px_ndc - 1) * ASPECT_RATIO * tan(deg2rad(FOV) / 2);
            const double cam_y = (1 - 2 * py_ndc) * tan(deg2rad(FOV) / 2);

            Vec3d d{cam_x, cam_y, -1};
            d.normalize();
            const Ray ray{origin, d};

            double dist{std::numeric_limits<double>::max()}, tmpdist;
            Color px;
            Vec3d tmpnormal, normal;
            bool intersect{false};
            Object* cur_obj{nullptr};


            // check intersections
            for (const auto& o : objects) {
                if (o->intersect(ray, tmpdist, tmpnormal)) {
                    if (tmpdist < dist) {
                        dist = tmpdist;
                        normal = tmpnormal;
                        cur_obj = &(*o);
                    }
                }
            }

            if (cur_obj != nullptr) {
                Vec3d phit{ray.getPoint(dist)};
                const Vec3d n{cur_obj->getNormal(phit)};

                for (const auto& l : lights) {
                    Vec3d lv{l.pos - phit};
//                    Vec lv{l.pos};
                    lv.normalize();
                    bool inShadow{false};

                    // check if other object is blocking light
                    const Ray shadow_ray{phit, lv};
                    for (const auto& o : objects) {
                        if (o->intersect(shadow_ray, tmpdist, tmpnormal)) {
                            inShadow = true;
                            break;
                        }
                    }
                    if (inShadow) {
                        continue;
                    }

                    const double diff_factor{n.dotProduct(lv)};
                    px += cur_obj->color * l.color * diff_factor;
                }


                px.clamp(0, 1);
                px = px + cur_obj->color * 0.1;
                px.clamp(0, 1);
                intersect = true;
            }

            if (!intersect) {
                px = background;
            }

            *(img_ptr++) = px;
        }
    }


    // write image to file
    img_ptr = img;
    for (unsigned int i = 0; i < W * H; ++i) {
        Color c = *(img_ptr++) * MAX_VAL;
        c.round();
        ofs << c;
    }


    delete[] img;
    delete[] zbuff;
}
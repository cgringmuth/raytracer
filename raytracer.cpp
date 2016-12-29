#include <iostream>
#include <sstream>
#include <math.h>
#include <fstream>
#include <vector>
#include <memory>
#include <limits>
#include <stdlib.h>
#include <iomanip>
#include <thread>
#include <exception>

#include <opencv2/opencv.hpp>

// url: https://www.scratchapixel.com/lessons/3d-basic-rendering/introduction-to-ray-tracing
// operator overloading: http://stackoverflow.com/a/4421719/1959528
// 3D meshes: https://graphics.stanford.edu/data/3Dscanrep/
// Idea: Create the cornell box as test scene: https://en.wikipedia.org/wiki/Cornell_box


/**
 * TODOs
 *
 * - Add material with different reflection models (diffuse, specular, refraction etc.)
 * - Implement ply file loading -> very first draft implemented
 *
 */




using namespace std;


bool processing{true};
const string winName{"image"};
typedef unsigned char ImageType;

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

    Vec3d& operator+=(const double d) {
        x += d;
        y += d;
        z += d;
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
            case 0:
                return x;
            case 1:
                return y;
            case 2:
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

Vec3d operator+(Vec3d lhs, const double d) {
    return lhs += d;
}

Vec3d operator*(Vec3d lhs, const double val) {
    return lhs *= val;
}


double dotProduct(const Vec3d& v1, const Vec3d& v2) {
    return v1.dotProduct(v2);
}


ostream&
operator<<(ostream& os, const Vec3d& v) {
    os << v.x << " " << v.y << " " << v.z << " ";
    return os;
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

    Color(const double v) : r{v}, g{v}, b{v} {};

    Color(double r, double g, double b) : r{r}, g{g}, b{b} {};

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

    static Color green() {
        return Color(0, 1, 0);
    }

    static Color blue() {
        return Color(0, 0, 1);
    }

    static Color gray() {
        return Color(0.5);
    }

    static Color light_gray() {
        return Color(0.75);
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
        return origin + (direction * dist);
    }
};



struct Material {
    Color color;
    double ka;  // ambient reflectance
    double kd;  // diffuse reflectance

    double ks;  // specular reflectance
    double specRefExp; // specular-reflection exponent

    Material(const Color& color, double ka=0.15, double kd=0.7, double ks=0.3, double specRefExp=8)
            : color(color), ka(ka), kd(kd), ks(ks), specRefExp(specRefExp) {}

    Material() : color{} {}
};


/** Generic base class for all objects which shall be rendered
 *
 */
struct Object {
    Material material;

    Object(const Color& color) : material(color) {}
    Object(const Material& material) : material(material) {}
    Object() : material{} {}

    virtual bool intersect(const Ray& ray, double& dist, Vec3d& normal) const = 0;

    virtual Vec3d getNormal(const Vec3d& vec) const = 0;

};


struct Plane : public Object {
    double a, b, c, d;  // implicit description: ax + by + cz + d = 0

    Plane(double a, double b, double c, double d, const Color& color)
            : a(a), b(b), c(c), d(d), Object(color) {
        // normalize normal vector
        Vec3d pn{a, b, c};
        pn.normalize();
        a = pn[0];
        b = pn[1];
        c = pn[2];
    }

    Plane(Vec3d normal, double dist, const Color& color) : d{dist}, Object{color} {
        normal.normalize(); // make sure normal is normalized
        a = normal[0];
        b = normal[1];
        c = normal[2];
    }

    bool intersect(const Ray& ray, double& dist, Vec3d& normal) const override {
        //src: https://www.scratchapixel.com/lessons/3d-basic-rendering/ray-tracing-rendering-a-triangle/ray-triangle-intersection-geometric-solution
        const double eps{0.00001};
        normal = Vec3d{a, b, c};
        const double vd{normal.dotProduct(ray.direction)};

        if (abs(vd) < eps)  // check if vd is 0 -> plane and ray parallel
            false;
        if (vd > 0)     // normal of plane is pointing away from camera (maybe handle differently)
            false;

        const double vo{normal.dotProduct(ray.origin) + d};
        dist = -vo / vd;

        return dist > eps;
    }

    Vec3d getNormal(const Vec3d& vec) const override {
        return Vec3d{a, b, c};
    }
};

struct Triangle : public Object {
    Vec3d v0, v1, v2;

    Triangle(const Vec3d& v0, const Vec3d& v1, const Vec3d& v2, const Color& color)
            : v0{v0}, v1{v1}, v2{v2}, Object{color} {}

    Triangle(const Vec3d& v0, const Vec3d& v1, const Vec3d& v2)
            : v0{v0}, v1{v1}, v2{v2} {}

    virtual bool intersect(const Ray& ray, double& dist, Vec3d& normal) const override {
        const double eps{0.00001};
        normal = getNormal(v0);
//        cout << normal << endl;
//        cout << "v0: " << v0 << "v1: " << v1 << "v2: " << v2 << endl;
        Plane plane{normal,
                    -normal.dotProduct(v1),  // todo: check if this is correct (seems more as an workaround)
                    Color::white()};
//        cout << ray.direction << endl;

        if (!plane.intersect(ray, dist, normal)) {
            return false;
        }

        const Vec3d hit{ray.getPoint(dist)}; // get point on plane

//        cout << "hit: " << hit << " hit length:" << hit.length() << endl;

        // do the "inside-outside" test
        const Vec3d edge0{v1 - v0};
        const Vec3d vp0{hit - v0};
//        cout << "edge0: " << edge0 << " vp0: " << vp0 << endl;
        if (dotProduct(cross_product(edge0, vp0), normal) < 0) {
            return false;
        }

        const Vec3d edge1{v2 - v1};
        const Vec3d vp1{hit - v1};
//        cout << "edge1: " << edge1 << " vp1: " << vp1 << endl;
        if (dotProduct(cross_product(edge1, vp1), normal) < 0) {
            return false;
        }

        const Vec3d edge2{v0 - v2};
        const Vec3d vp2{hit - v2};
//        cout << "edge2: " << edge2 << " vp2: " << vp2 << endl;
        if (dotProduct(cross_product(edge2, vp2), normal) < 0) {
            return false;
        }

//        cout << "dist: " << eps << endl;

        return dist > eps;
    }

    virtual Vec3d getNormal(const Vec3d& vec) const override {
        const Vec3d edge1{v2 - v0};
        const Vec3d edge0{v1 - v0};
        Vec3d normal{cross_product(edge0, edge1)};
        normal.normalize();
//        cout << "normal: " << normal << endl;
        return normal;
    }

    void scale(double s) {
        v0 *= s;
        v1 *= s;
        v2 *= s;
    }

    Triangle& operator+=(const double t) {
        v0 += t;
        v1 += t;
        v2 += t;
        return *this;
    }

    Triangle& operator+=(const Vec3d& v) {
        v0 += v;
        v1 += v;
        v2 += v;
        return *this;
    }
};

Triangle operator+(Triangle lhs, const double t) {
    return lhs += t;
}

Triangle operator+(Triangle lhs, const Vec3d& v) {
    return lhs += v;
}

struct Model : Object {
    vector<Triangle> faces;

    Model(const Color& color, const vector<Triangle>& faces) : Object(color), faces(faces) {}

    Model(const Color& color) : Object(color) {}

    Model() {}

    static shared_ptr<Model> load_ply(const string& fname) {
        shared_ptr<Model> model{make_shared<Model>()};
        Color color{Color::white()};

        ifstream ifs{fname};
        if (!ifs.is_open())
            throw runtime_error{"Model "+fname+" could not be loaded. File does not exists."};

        string line, key;
        unsigned int val;
        getline(ifs, line);
        bool ply_type{false};

        unsigned int nfaces, nvertices;

        while (true) {
            stringstream ss{line};
            ss >> key;
            if (key == "ply") {
                ply_type = false;
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
            getline(ifs, line);
        }

        // assume coordinates x, y, z come first and ignore all other following values
        vector<Vec3d> vertices;
        vector<Triangle> faces;

        // read vertices
        for (int i = 0; i < nvertices; ++i) {
            getline(ifs, line);
            stringstream ss{line};
            double x, y, z;
            ss >> x >> y >> z;
            vertices.emplace_back(Vec3d{x, y, z});
        }

        // read faces
        for (int i = 0; i < nfaces; ++i) {
            getline(ifs, line);
            stringstream ss{line};
            int num, iv0, iv1, iv2;
            ss >> num >> iv0 >> iv1 >> iv2;
            if (num != 3) {
                cerr << "Only triangles supported\n";
                exit(1);
            }
            faces.emplace_back(Triangle{vertices[iv0], vertices[iv1], vertices[iv2]});
        }

        model->faces = faces;
        model->material = Material(color);
        model->material.ks = 0.6;

        return model;
    }

    bool intersect(const Ray& ray, double& dist, Vec3d& normal) const override {
        // todo: through all faces and give closest distance which is not negative and return normal also (for all)
        double tmpdist;
        Vec3d tmpnormal;
        dist = std::numeric_limits<double>::max();
        bool hit{false};
        const double eps{0.00001};
        for (const auto& f : faces) {
            if (f.intersect(ray, tmpdist, tmpnormal) && tmpdist < dist) {
                dist = tmpdist;
                normal = tmpnormal;
                hit = true;
            }
        }
        return hit && dist > eps;
    }

    Model& scale(double s) {
        for (auto& f : faces) {
            f.scale(s);
        }
        return *this;
    }

    Model& translate(const Vec3d& t) {
        for (auto& f : faces) {
            f += t;
        }
        return *this;
    }

    Vec3d getNormal(const Vec3d& vec) const override {
        return Vec3d{};     // fixme: currently broken
    }

};


/** The most common object to be rendered in a raytracer
 *
 */
struct Sphere : public Object {
    // define location + radius
    Vec3d center;
    double radius;

    Sphere(const Vec3d& c, double r, const Color& color) : center{c}, radius{r}, Object{color} {}
    Sphere(const Vec3d& c, double r, const Material& material) : center{c}, radius{r}, Object{material} {}

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
        normal = getNormal(ray.getPoint(dist));

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

    Light(Vec3d pos, Color color) : pos{pos}, color{color} {}

    Light(Vec3d pos) : pos{pos}, color{Color::white()} {}
};


ostream&
operator<<(ostream& os, const Color& c) {
    os << c.r << " " << c.g << " " << c.b << " ";
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


void
create_box(vector<shared_ptr<Object>>& objects) {
    const int y_offset{-3};
    const int x_offset{2};
    const int z_offset{-9};

    const int x_width_half{1};
    const int y_width_half{1};
    const int z_width_half{1};

    /**
     * adopted from: http://stackoverflow.com/a/8142461/1959528
     *       6-------5
     *      /|      /|
     *    /  |     / |
     *   0-------3   |
     *   |   7---|---4
     *   |  /    |  /
     *   |/      |/
     *   1-------2
     */

    // front
    const Vec3d v0{x_offset - x_width_half, y_offset + y_width_half, z_offset + z_width_half};
    const Vec3d v1{x_offset - x_width_half, y_offset - y_width_half, z_offset + z_width_half};
    const Vec3d v2{x_offset + x_width_half, y_offset - y_width_half, z_offset + z_width_half};
    const Vec3d v3{x_offset + x_width_half, y_offset + y_width_half, z_offset + z_width_half};
    // back
    const Vec3d v5{x_offset + x_width_half, y_offset + y_width_half, z_offset - z_width_half};
    const Vec3d v4{x_offset + x_width_half, y_offset - y_width_half, z_offset - z_width_half};
    const Vec3d v6{x_offset - x_width_half, y_offset + y_width_half, z_offset - z_width_half};
    const Vec3d v7{x_offset - x_width_half, y_offset - y_width_half, z_offset - z_width_half};

//    const Color color{192.0 / 255, 155.0 / 255, 94.0 / 255};
    const Color color = Color::blue();
    vector<Triangle> triangles;


    // front
    // 0---3
    // | \ |
    // 1---2
    triangles.emplace_back(Triangle{v0, v1, v2, color});
    triangles.emplace_back(Triangle{v0, v2, v3, color});
//    objects.push_back(make_shared<Triangle>( v0, v1, v2, color));
//    objects.push_back(make_shared<Triangle>( v0, v2, v3, color));
    // right
    // 3---5
    // | \ |
    // 2---4
    triangles.emplace_back(Triangle{v3, v2, v4, color});
    triangles.emplace_back(Triangle{v3, v4, v5, color});
//    objects.push_back(make_shared<Triangle>( v3, v2, v4, color));
//    objects.push_back(make_shared<Triangle>( v3, v4, v5, color));
    // back
    // 5---6
    // | \ |
    // 4---7
    triangles.emplace_back(Triangle{v5, v4, v7, color});
    triangles.emplace_back(Triangle{v5, v7, v6, color});
//    objects.push_back(make_shared<Triangle>( v5, v4, v7, color));
//    objects.push_back(make_shared<Triangle>( v5, v7, v6, color));
    // left
    // 6---0
    // | \ |
    // 7---1
    triangles.emplace_back(Triangle{v6, v7, v1, color});
    triangles.emplace_back(Triangle{v6, v1, v0, color});
//    objects.push_back(make_shared<Triangle>( v6, v7, v1, color));
//    objects.push_back(make_shared<Triangle>( v6, v1, v0, color));
    // top
    // 6---5
    // | \ |
    // 0---3
    triangles.emplace_back(Triangle{v6, v0, v3, color});
    triangles.emplace_back(Triangle{v6, v3, v5, color});
//    objects.push_back(make_shared<Triangle>( v6, v0, v3, color));
//    objects.push_back(make_shared<Triangle>( v6, v3, v5, color));
    // bottom
    // 7---4
    // | \ |
    // 1---2
    triangles.emplace_back(Triangle{v7, v1, v2, color});
    triangles.emplace_back(Triangle{v7, v1, v2, color});
//    objects.push_back(make_shared<Triangle>( v7, v1, v2, color));
//    objects.push_back(make_shared<Triangle>( v7, v1, v2, color));
    shared_ptr<Model> model{make_shared<Model>(color, triangles)};
    objects.push_back(model);
}


void
preview(cv::Mat& img, double* progressArr, int numProg, int delay = 1000)
{
    int key;
    double progress;
    while(processing)
    {
        cv::Mat tmpimg{img.clone()};
        progress = 0;
        for (int n=0; n<numProg; ++n) {
            progress += progressArr[n];
        }
        progress /= numProg;
        progress *= 100;

        const string text{to_string((int)progress) + "%"};
        const int fontFace{CV_FONT_HERSHEY_SIMPLEX};
        const double fontScale{1};
        const cv::Scalar color{cv::Scalar{0,255,0}};
        const int thickness{1};
        int baseLine{0};
        const cv::Size textSize{cv::getTextSize(text, fontFace, fontScale, thickness+2, &baseLine)};
        const int x{10};
        const int y{10};
        const int border{12};
        const double alpha{0.2};

        // draw box;
        cv::Rect roi{x, y, textSize.width+border, textSize.height+border};
        cv::Mat foo{roi.height, roi.width, tmpimg.type(), cv::Scalar::all(130)};
        cv::Mat cropped{tmpimg(roi)};
        cv::putText(foo, text, cv::Point{border/2, textSize.height+border/2-2}, fontFace, fontScale, color, thickness+2);
        cv::addWeighted(cropped, alpha, foo, 1-alpha, 0, cropped);
        cv::putText(cropped, text, cv::Point{border/2, textSize.height+border/2-2}, fontFace, fontScale, color*0.5, thickness);

        cv::imshow(winName, tmpimg);
        key = cv::waitKey(delay);
        if ((key & 0xff) == 27) {
            cout << "... rendering canceled" << endl;
            exit(0);
        }
    }
}

void
render(ImageType* img, unsigned int x_start, unsigned int y_start, unsigned int cH, unsigned int cW, const unsigned int H, const unsigned int W,
       const double ASPECT_RATIO, const double FOV, const Vec3d& origin, const vector<shared_ptr<Object>>& objects,
       const vector<Light>& lights, const Color& background, double& progress) {
    progress = 0;
    const unsigned int sumPix{cH*cW};
    unsigned int curPix{0};

    for (unsigned int y = y_start; y < cH+y_start; ++y) {
        ImageType* img_ptr{img + 3 * (y*W + x_start)};
        for (unsigned int x = x_start; x < cW+x_start; ++x) {
            const double px_ndc = (x + 0.5) / W;
            const double py_ndc = (y + 0.5) / H;
            const double cam_x = (2 * px_ndc - 1) * ASPECT_RATIO * tan(deg2rad(FOV) / 2);
            const double cam_y = (1 - 2 * py_ndc) * tan(deg2rad(FOV) / 2);

            Vec3d d{cam_x, cam_y, -1};  // camera looks in negative z direction
            d.normalize();
            const Ray ray{origin, d};

            double dist{std::numeric_limits<double>::max()}, tmpdist;
            Color px_color;
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
                Material& cmat = cur_obj->material;
                Vec3d phit{ray.getPoint(dist)};
//                cout << typeid(*cur_obj).name() << " ";

                for (const auto& l : lights) {
                    Vec3d lv{l.pos - phit};
//                    Vec lv{l.pos};
                    const double ldist{lv.length()};
                    lv.normalize();
                    bool inShadow{false};

                    // cast shadow ray and check if other object is blocking light
                    const Ray shadow_ray{phit, lv};
                    for (const auto& o : objects) {
                        const bool hit{o->intersect(shadow_ray, tmpdist, tmpnormal)};
                        if (hit && tmpdist < ldist) {
                            inShadow = true;
                            break;
                        }
                    }
                    if (inShadow) {
                        continue;
                    }

                    const double cosPhi{
                            std::max(normal.dotProduct(lv), 0.0)};    // todo: check why we have to clip negative values
//                    const double cosPhi{n.dotProduct(lv)};

                    // diffuse shading
                    px_color += cmat.color * l.color * cmat.kd * (cosPhi / (ldist * ldist));
                    // specular shading
                    const Vec3d reflectRay{normal*2*dotProduct(normal,lv) -lv};
                    const double cosAlpha{
                            std::max(reflectRay.dotProduct(lv), 0.0)};    // todo: check why we have to clip negative values
                    px_color += cmat.color * l.color * cmat.ks * (pow(cosAlpha, cmat.specRefExp) / (ldist * ldist));

                    px_color.clamp(0, 1);
                }
                px_color = px_color + cmat.color * cmat.ka;
                px_color.clamp(0, 1);
                intersect = true;
            }

            if (!intersect) {
                px_color = background;
            }

            *(img_ptr++) = (ImageType) (px_color.b * 255);
            *(img_ptr++) = (ImageType) (px_color.g * 255);
            *(img_ptr++) = (ImageType) (px_color.r * 255);

            progress = ((double)++curPix)/sumPix;
        }
    }
}

void
colorize_image_tile(cv::Mat img, int num, int x_start, int y_start, int pW, int pH)
{
    int baseline = 0;
    const int fontFace = CV_FONT_HERSHEY_SIMPLEX;
    const double fontScale = 1;
    const int thickness = 2;
    const string text{"thread "+to_string(num)};
    const cv::Scalar bgColors[] = {cv::Scalar{0, 0, 255},
                                   cv::Scalar{0, 255, 0},
                                   cv::Scalar{255, 0, 0},
                                   cv::Scalar{255, 255, 0},
                                   cv::Scalar{255, 0, 255},
                                   cv::Scalar{125, 125, 125},
                                   cv::Scalar{0, 125, 255},
                                   cv::Scalar{125, 255, 0},
                                   cv::Scalar{255, 125, 0},
                                   cv::Scalar{255, 255, 125},
                                   cv::Scalar{255, 125, 255}};
    // color each part differently
    cv::Mat tile{img(cv::Rect{x_start, y_start, pW, pH})};
    tile = bgColors[num]*0.2;
    const cv::Size textSize{cv::getTextSize(text, fontFace, fontScale, thickness, &baseline)};
    cv::putText(tile, text, cv::Point{pW / 2 - textSize.width/2, pH / 2}, fontFace, fontScale, cv::Scalar::all(255), thickness);
}


void
create_scene(vector<shared_ptr<Object>>& objects, vector<Light>& lights) {
    objects.push_back(make_shared<Sphere>(Vec3d{0, 0, -10}, 1, Material(Color::red(), 0.1, 0.2, 0.7)));
//    objects.push_back(make_shared<Sphere>(Vec{10,0,-20}, 5, scolor));
    objects.push_back(make_shared<Sphere>(Vec3d{1.75, -1.5, -9}, 0.3, Color{1, 1, 0}));
    objects.push_back(make_shared<Sphere>(Vec3d{-1, -1, -7}, 0.5, Color::blue()));
    objects.push_back(make_shared<Sphere>(Vec3d{80, -6, -150}, 5, Color{0, 0, 1}));
    objects.push_back(make_shared<Sphere>(Vec3d{-3, 2, -7}, 1, Color{1, 0, 1}));

    create_box(objects);
    const string mesh_root{"/home/chris/shared/github/chris/raytracer/data/3d_meshes/"};
    string bunny_res4_path{mesh_root+"bunny/reconstruction/bun_zipper_res4.ply"};
    string bunny_path{mesh_root+"bunny/reconstruction/bun_zipper.ply"};
    shared_ptr<Model> bunny{Model::load_ply(bunny_res4_path)};
    bunny->scale(15);
    bunny->translate(Vec3d{-2, -4, -7.5});
    objects.push_back(bunny);

    string buddha_res4_path{mesh_root+"happy_recon/happy_vrip_res4.ply"};
    string buddha_path{mesh_root+"happy_recon/happy_vrip.ply"};
    shared_ptr<Model> buddha{Model::load_ply(buddha_res4_path)};
    buddha->scale(15);
    buddha->translate(Vec3d{2, -4, -7.5});
    buddha->material.ks = 0.9;
    objects.push_back(buddha);


    // planes
    const int box_len{5};
    // back
    const Color wall_color{192.0 / 255, 155.0 / 255, 94.0 / 255};
    objects.push_back(make_shared<Plane>(0, 0, 1, box_len + 10, wall_color));
    // left
    objects.push_back(make_shared<Plane>(1, 0, 0, box_len, Color::red()));
    // right
    objects.push_back(make_shared<Plane>(-1, 0, 0, box_len, Color::green()));
    // bottom
    objects.push_back(make_shared<Plane>(0, 1, 0, box_len, wall_color));
    // top
    objects.push_back(make_shared<Plane>(0, -1, 0, box_len, wall_color));

//    s.radius = 10;
    lights.emplace_back(Light{Vec3d{0, 3, -7.5}, Color::white() * 20});
//    lights.emplace_back(Light{Vec3d{0, 8, -9}, Color::white()*0.5});
    lights.emplace_back(Light{Vec3d{-1, -1, -1}, Color::white() * 7});
//    lights.emplace_back(Light{Vec3d{5, -5, -2}, Color::white()*0.5});
//    lights.emplace_back(Light{Vec{-30,-20,1}});
}

int
main(int argc, char** argv) {
    string outFilename{"raytracer.png"};
    if(argc > 1)
    {
        if (string(argv[1]) == "-o") {
            outFilename = argv[2];
        }
    }

    cout << "... start ray tracer" << endl;
    cout << "... write to file: " << outFilename << endl;
//    check_op_overloading();

    // resolution has to be even
    constexpr unsigned int downScale{2};
    constexpr unsigned int H{600/downScale};
    constexpr unsigned int W{800/downScale};
    ImageType* img_ptr = new ImageType[W*H*3];
    memset(img_ptr, 0, sizeof(ImageType)*W*H*3);
    cv::Mat img{H, W, CV_8UC3, img_ptr};
    cv::namedWindow(winName, CV_WINDOW_AUTOSIZE);

//    constexpr unsigned int H = 250;
//    constexpr unsigned int W = 350;
    constexpr unsigned int MAX_VAL = 255;
    constexpr double ASPECT_RATIO = (double) W / H;
    constexpr double FOV = 60;

    Color background{0, 0.5, 0.5};

    vector<shared_ptr<Object>> objects;
    vector<Light> lights;
    create_scene(objects, lights);

    const Vec3d origin{0, 0, 0};  // center of projection

    // 8 threads (7 child threads + 1 main thread)
    const int num_threads{8};   // max: num available threads
    double progress[num_threads];
    thread threads[num_threads-1];

    // split image into tiles (2x4)
    const unsigned int nXTiles{2};
    const unsigned int nYTiles{4};
    const unsigned int pW{W/nXTiles};
    const unsigned int pH{H/nYTiles};

    // starting thread to show progress
    thread thread_show{preview, std::ref(img), progress, num_threads, 100};

    // start threads
    unsigned int x_start{0};
    unsigned int y_start{0};
    unsigned int ctr{0};
    for (unsigned int n=0; n<num_threads-1; ++n) {
        cout << "... starting thread " << n << endl;
        threads[n] = thread{render, img_ptr, x_start, y_start, pH, pW, H, W, ASPECT_RATIO, FOV, origin, objects,
         lights, background, std::ref(progress[ctr])};
        colorize_image_tile(img, ctr++, x_start, y_start, pW, pH);
        x_start += pW;
        if (x_start >= W) {
            x_start = 0;
            y_start += pH;
        }
    }

    // main thread does the rest
    colorize_image_tile(img, ctr, x_start, y_start, pW, pH);
    render(img_ptr, x_start, y_start, pH, pW, H, W, ASPECT_RATIO, FOV, origin, objects, lights, background, progress[ctr]);

    // wait for other threads to finish
    for (unsigned int n=0; n<num_threads-1; ++n) {
        cout << "... waiting for thread " << n << endl;
        if (threads[n].joinable())
            threads[n].join();
    }
    processing = false;
    if (thread_show.joinable())
        thread_show.join();

    cv::imshow(winName, img);
    cout << "... write image " << outFilename << endl;
    cv::imwrite(outFilename, img);
    cv::waitKey();
    delete[] img_ptr;
}
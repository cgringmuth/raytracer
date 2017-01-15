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
#include <mutex>
#include <exception>
#include <chrono>

#include <opencv2/opencv.hpp>


/** Some good resources ray tracing
 * Lecture about ray tracing: http://www.cs.uu.nl/docs/vakken/magr/2015-2016/index.html
 * url: https://www.scratchapixel.com/lessons/3d-basic-rendering/introduction-to-ray-tracing
 * operator overloading: http://stackoverflow.com/a/4421719/1959528
 * 3D meshes: https://graphics.stanford.edu/data/3Dscanrep/
 * Idea: Create the cornell box as test scene: https://en.wikipedia.org/wiki/Cornell_box
 */



/** TODOs
 * - todo: Add material with different reflection models (refraction etc.)
 * - todo: Loading scene from file (xml, YAML etc.) -> I would prefer yaml
 * - todo: Implement anti-aliasing
 * - todo: Soft-shadows
 * - todo: Area lights
 * - todo: depth of field
 * - todo: texture mapping
 * - todo: optimization: early pruning of objects which cannot be hit (kD-tree (spatial partitioning), BVH (object partitioning) etc.)
 * - todo: optimization: bounding box with fast intersection calculation around object (bounding box: sphere, box etc.)
 * - todo: optimization: do calculation on GPU
 * - todo: create scene to hold primitives, lights etc.
 * - todo: restructure project
 * - todo: motion blur
 * - todo: global illumination: https://en.wikipedia.org/wiki/Global_illumination
 */


/**
 * This enables Moeller-Trumbore algorithm for triangle intersection calculation. It is the fastest intersection
 * algorithm so far.
 */
#ifndef MT_TRIANGLE_INTERSECT
#define MT_TRIANGLE_INTERSECT   1
#endif

/**
 * Back-face culling basically leads some performance improvements. But it only works properly with solid objects.
 * Hence, shadows might not be rendered correctly when CULLING is turned on and you want to render hulls instead
 * of solid objects. This is why it is turned of per default.
 * Refer to https://www.scratchapixel.com/lessons/3d-basic-rendering/ray-tracing-rendering-a-triangle/single-vs-double-sided-triangle-backface-culling
 * to reader more.
 */
#ifndef CULLING
#define CULLING                 0
#endif

#ifndef USE_OPENMP
#define USE_OPENMP              0
#endif

using namespace std;





class Timer
{
public:
    Timer() : beg_(clock_::now()) {}
    void reset() { beg_ = clock_::now(); }
    double elapsed() const {
        return std::chrono::duration_cast<second_>
                (clock_::now() - beg_).count(); }

private:
    typedef std::chrono::high_resolution_clock clock_;
    typedef std::chrono::duration<double, std::ratio<1> > second_;
    std::chrono::time_point<clock_> beg_;
};

bool processing{true};
const string winName{"image"};
typedef unsigned char ImageType;
Timer timer;
constexpr double EPS{0.000001};
constexpr int MAX_DEPTH{10};
mutex RENDER_MUTEX;


/** Converts angle in degree into rad.
 *
 * @param ang angle in degree
 * @return angle in rad
 */
double deg2rad(double ang) {
    return ang * M_PI / 180;
}


struct Mat3d {
    double v[9];
    static const unsigned int width{3};
    static const unsigned int height{3};

    Mat3d(double v0, double v1, double v2,
          double v3, double v4, double v5,
          double v6, double v7, double v8) {
        v[0] = v0; v[1] = v1; v[2] = v2;
        v[3] = v3; v[4] = v4; v[5] = v5;
        v[6] = v6; v[7] = v7; v[8] = v8;
    }

    Mat3d(double* v) {
        memcpy(this->v, v, sizeof(double)*length());
    }

    Mat3d(const Mat3d& mat) {
        memcpy(this->v, mat.v, sizeof(double)*length());
    }

    double& at(unsigned int x, unsigned int y) { return v[y*width + x]; }
    double at(unsigned int x, unsigned int y) const { return v[y*width + x]; }

    double& operator[](size_t idx) {
        return v[idx];
    }

    double operator[](size_t idx) const {
        return v[idx];
    }

    size_t length() const { return width*height; }

    Mat3d& operator*=(const Mat3d& rhs) {
        const Mat3d tmp{*this};
        at(0,0) = tmp.at(0,0)*rhs.at(0,0) + tmp.at(1,0)*rhs.at(0,1) + tmp.at(2,0)*rhs.at(0,2);
        at(1,0) = tmp.at(0,0)*rhs.at(1,0) + tmp.at(1,0)*rhs.at(1,1) + tmp.at(2,0)*rhs.at(1,2);
        at(2,0) = tmp.at(0,0)*rhs.at(2,0) + tmp.at(1,0)*rhs.at(2,1) + tmp.at(2,0)*rhs.at(2,2);
        at(0,1) = tmp.at(0,1)*rhs.at(0,0) + tmp.at(1,1)*rhs.at(0,1) + tmp.at(2,1)*rhs.at(0,2);
        at(1,1) = tmp.at(0,1)*rhs.at(1,0) + tmp.at(1,1)*rhs.at(1,1) + tmp.at(2,1)*rhs.at(1,2);
        at(2,1) = tmp.at(0,1)*rhs.at(2,0) + tmp.at(1,1)*rhs.at(2,1) + tmp.at(2,1)*rhs.at(2,2);
        at(0,2) = tmp.at(0,2)*rhs.at(0,0) + tmp.at(1,2)*rhs.at(0,1) + tmp.at(2,2)*rhs.at(0,2);
        at(1,2) = tmp.at(0,2)*rhs.at(1,0) + tmp.at(1,2)*rhs.at(1,1) + tmp.at(2,2)*rhs.at(1,2);
        at(2,2) = tmp.at(0,2)*rhs.at(2,0) + tmp.at(1,2)*rhs.at(2,1) + tmp.at(2,2)*rhs.at(2,2);
        return *this;
    }

    static Mat3d rotation(double phiX, double phiY, double phiZ);

    static Mat3d rotationX(double phi) {
        double vmat[9];
        const double cosphi{cos(phi)};
        const double sinphi{sin(phi)};
        vmat[0] = 1; vmat[1] = 0;      vmat[2] = 0;
        vmat[3] = 0; vmat[4] = cosphi; vmat[5] = -sinphi;
        vmat[6] = 0; vmat[7] = sinphi; vmat[8] = cosphi;
        return Mat3d{vmat};
    }

    static Mat3d rotationY(double phi) {
        double vmat[9];
        const double cosphi{cos(phi)};
        const double sinphi{sin(phi)};
        vmat[0] = cosphi;  vmat[1] = 0; vmat[2] = sinphi;
        vmat[3] = 0;       vmat[4] = 1; vmat[5] = 0;
        vmat[6] = -sinphi; vmat[7] = 0; vmat[8] = cosphi;
        return Mat3d{vmat};
    }

    static Mat3d rotationZ(double phi) {
        double vmat[9];
        const double cosphi{cos(phi)};
        const double sinphi{sin(phi)};
        vmat[0] = cosphi;  vmat[1] = -sinphi; vmat[2] = 0;
        vmat[3] = sinphi;  vmat[4] = cosphi;  vmat[5] = 0;
        vmat[6] = 0;       vmat[7] = 0;       vmat[8] = 1;
        return Mat3d{vmat};
    }
};

Mat3d operator*(Mat3d lhs, const Mat3d& rhs) {
    return lhs *= rhs;
}

Mat3d Mat3d::rotation(double phiX, double phiY, double phiZ) {
    return Mat3d::rotationZ(phiZ) * Mat3d::rotationY(phiY) * Mat3d::rotationX(phiX);
}

ostream& operator<<(ostream& os, const Mat3d& mat) {
    for(int n=0; n<mat.length()-1; ++n) {
        cout << mat[n] << " ";
    }
    cout << mat[mat.length()-1];
    return os;
}

/** 3D vector in cartesian space.
 *
 */
struct Vec3d {
    double x, y, z;

    Vec3d() : x{0}, y{0}, z{0} {}

    Vec3d(double x, double y, double z) : x{x}, y{y}, z{z} {}

    Vec3d(const Vec3d& v) : x{v.x}, y{v.y}, z{v.z} {}

    Vec3d operator-() const {
        Vec3d v;
        v.x = -x;
        v.y = -y;
        v.z = -z;
        return v;
    }

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

    Vec3d& operator*=(const Mat3d& mat) {
        double tx{x*mat.at(0,0) + y*mat.at(1,0) + z*mat.at(2, 0)};
        double ty{x*mat.at(0,1) + y*mat.at(1,1) + z*mat.at(2, 1)};
        double tz{x*mat.at(0,2) + y*mat.at(1,2) + z*mat.at(2, 2)};
        x = tx;
        y = ty;
        z = tz;
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

Vec3d operator*(const double val, Vec3d lhs) {
    return lhs *= val;
}



double dotProduct(const Vec3d& v1, const Vec3d& v2) {
    return v1.dotProduct(v2);
}

Vec3d operator*(Vec3d lhs, const Mat3d& rhs) {
    return lhs *= rhs;
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
    explicit Color(const double v) : r{v}, g{v}, b{v} {};
    Color(double r, double g, double b) : r{r}, g{g}, b{b} {};
    Color(const Color& color) : r{color.r}, g{color.g}, b{color.b} {};

    Color& operator/=(double d) {
        r /= d;
        g /= d;
        b /= d;
        return *this;
    }

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

    static Color yellow() {
        return Color(1,1,0);
    }

    static Color glass() {
        return Color(0.788, 0.858, 0.862);
    }

};

Color operator/(Color lhs, const double d) {
    return lhs /= d;
}

Color operator+(Color lhs, const Color& rhs) {
    return lhs += rhs;
}

Color operator*(Color lhs, const double d) {
    return lhs *= d;
}

Color operator*(const double d, Color rhs) {
    return rhs *= d;
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

    bool reflective;
    double kr;  // reflectance coefficient

    bool refractive;
    /** The refraction coefficient. This is the coefficient for this material
     * src: https://graphics.stanford.edu/courses/cs148-10-summer/docs/2006--degreve--reflection_refraction.pdf
     *
     * Snell's law: n1*sin(phi1) = n2*sin(phi2)
     *
     * NOTE: It is allways assumed that ray go from medium (n1) to medium (n2).
     *
     * sin(phi1)   n1
     * --------- = -- = n12
     * sin(phi2)   n2
     *
     * Assuming air has n=1.0
     */
    double refractiveIdx;
    double kt;  // transimission coefficient (the amount of light which it can pass through object)

    Material(const Color& color, double ka=0.2, double kd=0.7, double ks=0.2, double kr=0, double kt=0, double specRefExp=8,
             double refractiveIdx=0, bool reflective=false, bool refractive=false)
            : color(color), ka(ka), kd(kd), ks(ks), specRefExp(specRefExp), kr(kr), refractiveIdx(refractiveIdx)
            , reflective(reflective), refractive(refractive), kt{kt} {}
    Material(double ka=0.2, double kd=0.7, double ks=0.2, double kr=0, double kt=0, double specRefExp=8, double refractiveIdx=0,
             bool reflective=false, bool refractive=false)
            : color{}, ka(ka), kd(kd), ks(ks), specRefExp(specRefExp), kr(kr), refractiveIdx(refractiveIdx),
              reflective(reflective), refractive(refractive), kt{kt} {}
};


/** Generic base class for all objects which shall be rendered
 *
 */
struct Primitive {
    Material material;

    Primitive(const Color& color) : material(color) {}
    Primitive(const Material& material) : material(material) {}
    Primitive() : material{} {}

    virtual bool intersect(const Ray& ray, double& dist, Vec3d& normal) const = 0;

    virtual Vec3d getNormal(const Vec3d& vec) const = 0;

};


struct Plane : public Primitive {
    double a, b, c, d;  // implicit description: ax + by + cz + d = 0

    Plane(double a, double b, double c, double d, const Color& color)
            : a(a), b(b), c(c), d(d), Primitive(color) {
        // normalize normal vector
        Vec3d pn{a, b, c};
        pn.normalize();
        this->a = pn[0];
        this->b = pn[1];
        this->c = pn[2];
    }

    Plane(double a, double b, double c, double d, const Material& material)
            : a(a), b(b), c(c), d(d), Primitive(material) {
        // normalize normal vector
        Vec3d pn{a, b, c};
        pn.normalize();
        this->a = pn[0];
        this->b = pn[1];
        this->c = pn[2];
    }


    Plane(Vec3d normal, double dist, const Color& color) : d{dist}, Primitive{color} {
        normal.normalize(); // make sure normal is normalized
        a = normal[0];
        b = normal[1];
        c = normal[2];
    }

    bool intersect(const Ray& ray, double& dist, Vec3d& normal) const override {
        //src: https://www.scratchapixel.com/lessons/3d-basic-rendering/ray-tracing-rendering-a-triangle/ray-triangle-intersection-geometric-solution
        normal = Vec3d{a, b, c};
        const double vd{normal.dotProduct(ray.direction)};

        if (abs(vd) < EPS)  // check if vd is 0 -> plane and ray parallel
            false;
        if (vd > 0)     // normal of plane is pointing away from camera (maybe handle differently)
            false;

        const double vo{normal.dotProduct(ray.origin) + d};
        dist = -vo / vd;

        return dist > EPS;
    }

    Vec3d getNormal(const Vec3d& vec) const override {
        return Vec3d{a, b, c};
    }
};

struct Triangle : public Primitive {
    Vec3d v0, v1, v2, n0, n1, n2;

    Triangle(const Vec3d& v0, const Vec3d& v1, const Vec3d& v2, const Color& color)
            : v0{v0}, v1{v1}, v2{v2}, Primitive{color}, n0{calcNormal()}, n1{calcNormal()}, n2{calcNormal()} { }
    Triangle(const Vec3d& v0, const Vec3d& v1, const Vec3d& v2, const Material material=Material{})
            : v0{v0}, v1{v1}, v2{v2}, Primitive{material}, n0{calcNormal()}, n1{calcNormal()}, n2{calcNormal()} { }
    Triangle(const Vec3d& v0, const Vec3d& v1, const Vec3d& v2,
             const Vec3d& n0, const Vec3d& n1, const Vec3d& n2,
             const Material material=Material{})
            : v0{v0}, v1{v1}, v2{v2}, Primitive{material}, n0{n0}, n1{n1}, n2{n2} { }

    virtual bool intersect(const Ray& ray, double& dist, Vec3d& normal) const override {
        normal = this->n1;
#if MT_TRIANGLE_INTERSECT==1
        // src: http://www.cs.virginia.edu/%7Egfx/Courses/2003/ImageSynthesis/papers/Acceleration/Fast%20MinimumStorage%20RayTriangle%20Intersection.pdf
        // Any point on the triangle can be described by (in the uv-space)
        // P = v0 + u(v1-v0) + v(v2-v0)
        // P can also be described by ray equation (when hit triangle)
        // P = o + td
        // Note: u and v can be used for texture mapping and normal interpolation
        const Vec3d v0v1{v1-v0};    // edge 1 from v0 to v1
        const Vec3d v0v2{v2-v0};    // edge 2 from v0 to v2
        const Vec3d pVec{ray.direction.cross_product(v0v2)};
        const double det{pVec.dotProduct(v0v1)};

        // if determinant is negative, triangle is backfacing
        // if determinant is close to 0, ray is missing triangle
#if CULLING == 1
        if (det < EPS)
            return false;
#else
        if (abs(det) < EPS)
            return false;
#endif
        const double invDet = 1 / det;

        // calc u
        const Vec3d tVec{ray.origin-v0};
        const double u{invDet * pVec.dotProduct(tVec)};
        if (u < 0 || u > 1)
            return false;

        // calc v
        const Vec3d qVec{tVec.cross_product(v0v1)};
        const double v{invDet * qVec.dotProduct(ray.direction)};
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

        const Vec3d hit{ray.getPoint(dist)}; // get point on plane

//        cout << "hit: " << hit << " hit length:" << hit.length() << endl;

        // do the "inside-outside" test
        // check if intersection point is on left side of each edge

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

#endif
        return dist > EPS;
    }

    virtual Vec3d getNormal(const Vec3d& vec) const override {
        return n0;  // todo: does not make sense anymore
    }

    Vec3d calcNormal() {
        // todo: with one normal for each vertex this is not correct anymore
        const Vec3d edge0{v1 - v0};
        const Vec3d edge1{v2 - v0};
        n0 = cross_product(edge0, edge1);
        n0.normalize();
        return n0;
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

    Triangle& operator*=(const Mat3d& mat) {
        v0 *= mat;
        v1 *= mat;
        v2 *= mat;
        n0 *= mat;
        n1 *= mat;
        n2 *= mat;
        return *this;
    }
};

Triangle operator+(Triangle lhs, const double t) {
    return lhs += t;
}

Triangle operator+(Triangle lhs, const Vec3d& v) {
    return lhs += v;
}


template <typename T>
ostream& operator<<(ostream& os, vector<T> vec) {
    for (int n=0; n<vec.size()-1; ++n)
        os << vec[n] << " ";
    os << vec[vec.size()-1];
    return os;
}

struct Model : Primitive {
    vector<Triangle> faces;

    Model(const Color& color, const vector<Triangle>& faces) : Primitive(color), faces(faces) {}
    Model(const Material& material, const vector<Triangle>& faces) : Primitive(material), faces(faces) {}
    Model(const Color& color) : Primitive(color) {}
    Model(const Material& material) : Primitive(material) {}

    Model() {}

    static shared_ptr<Model> load_ply(const string& fname, const Material& material, bool calcNormal=false) {
        cout << "... loading model: " << fname << endl;

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
            getline(ifs, line);
        }

        // assume coordinates x, y, z come first and ignore all other following values
        vector<Vec3d> vertices, normals;
        vector<vector<unsigned int>> face_idx;
        vector<Triangle> faces;

        // read vertices
        for (int i = 0; i < nvertices; ++i) {
            getline(ifs, line);
            stringstream ss{line};
            double x, y, z;
            ss >> x >> y >> z;
            vertices.emplace_back(Vec3d{x, y, z});
        }

        // read faces indices
        for (int i = 0; i < nfaces; ++i) {
            getline(ifs, line);
            stringstream ss{line};
            unsigned int num, iv0, iv1, iv2;
            ss >> num >> iv0 >> iv1 >> iv2;
            if (num != 3) {
                throw runtime_error{"Only triangles are supported"};
            }
            face_idx.emplace_back(vector<unsigned int>{iv0, iv1, iv2});
        }


        if (calcNormal) {
            // interpolate normals
            normals.resize(vertices.size());
            for (const auto& fidx : face_idx) {
                const Vec3d v0{vertices[fidx[0]]};
                const Vec3d v1{vertices[fidx[1]]};
                const Vec3d v2{vertices[fidx[2]]};
                normals[fidx[0]] += cross_product(v1-v0, v2-v0);
                normals[fidx[1]] += cross_product(v2-v1, v0-v1);
                normals[fidx[2]] += cross_product(v0-v2, v1-v2);
            }
            // normalize all normals
            for (auto& normal : normals) {
                normal.normalize();
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

        return make_shared<Model>(material, faces);
    }

    bool intersect(const Ray& ray, double& dist, Vec3d& normal) const override {
        // todo: through all faces and give closest distance which is not negative and return normal also (for all)
        double tmpdist;
        Vec3d tmpnormal;
        dist = std::numeric_limits<double>::max();
        bool hit{false};
        for (const auto& f : faces) {
            if (f.intersect(ray, tmpdist, tmpnormal) && tmpdist < dist) {
                dist = tmpdist;
                normal = tmpnormal;
                hit = true;
            }
        }
        return hit && dist > EPS;
    }

    Model& scale(double s) {
        for (auto& f : faces) {
            f.scale(s);
        }
        return *this;
    }

    Model& translate(const Vec3d& t) {
        return *this += t;
    }

    Vec3d getNormal(const Vec3d& vec) const override {
        return Vec3d{};     // fixme: currently broken
    }

    Model& operator*=(const Mat3d& mat) {
        for (auto& f : faces) {
            f *= mat;
        }
        return *this;
    }

    Model& operator+=(const Vec3d& rhs) {
        for (auto& f : faces) {
            f += rhs;
        }
        return *this;
    }

};

Model operator+(Model lhs, const Vec3d& rhs) {
    return lhs += rhs;
}

Model operator*(Model lhs, const Mat3d& rhs) {
    return lhs *= rhs;
}


/** The most common object to be rendered in a raytracer
 *
 */
struct Sphere : public Primitive {
    // define location + radius
    Vec3d center;
    double radius;

    Sphere(const Vec3d& c, double r, const Color& color) : center{c}, radius{r}, Primitive{color} {}
    Sphere(const Vec3d& c, double r, const Material& material) : center{c}, radius{r}, Primitive{material} {}

    // get intersection with ray: refer: https://en.wikipedia.org/wiki/Line%E2%80%93sphere_intersection
    // more details: https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-sphere-intersection
    virtual bool
    intersect(const Ray& ray, double& dist, Vec3d& normal) const override {
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

        return dist > EPS;      //  neg. dist are behind ray; eps is for not hitting itself
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


struct Camera {
    Vec3d eye;  // Camera position
    Vec3d up;   // up direction (usually [0,1,0])
    Vec3d right;
    Vec3d at;   // Look at direction
    double aspectRatio;
    double fov;     // field of view
    unsigned int imWidth;
    unsigned int imHeight;

    Camera(const Vec3d& eye, const Vec3d& up, const Vec3d& at, double aspectRatio, double fov, unsigned int imWidth,
           unsigned int imHeight) : eye(eye), up(up), at(at), right(cross_product(at, up)), aspectRatio(aspectRatio), fov(fov), imWidth(imWidth),
                                imHeight(imHeight) {}

    Camera(double aspectRatio, double fov, unsigned int imWidth, unsigned int imHeight) :
            eye(Vec3d{0,0,0}), up(Vec3d{0,1,0}), at(Vec3d{0,0,-1}), right(cross_product(at, up)), aspectRatio(aspectRatio), fov(fov), imWidth(imWidth),
                                  imHeight(imHeight) {}

    Ray castRay(unsigned int x, unsigned int y) const {
        const double px_ndc{(x + 0.5) / imWidth};
        const double py_ndc{(y + 0.5) / imHeight};
        const double cam_x{(2 * px_ndc - 1) * aspectRatio * tan(deg2rad(fov) / 2)};
        const double cam_y{(1 - 2 * py_ndc) * tan(deg2rad(fov) / 2)};
        Vec3d camDir{right * cam_x + up * cam_y + at};
        camDir.normalize();

        return Ray{eye, camDir};
    }

    Camera& rotate(double alpha, double beta, double gamma) {
        const Mat3d rot = Mat3d::rotation(alpha, beta, gamma);
        up *= rot; up.normalize();
        at *= rot; at.normalize();
        right = cross_product(at, up);
        return *this;
    }

    Camera& move(const Vec3d& trans) {
        eye += trans;
        return *this;
    }
};

class light_blue;

ostream&
operator<<(ostream& os, const Color& c) {
    os << c.r << " " << c.g << " " << c.b << " ";
    return os;
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
create_box(vector<shared_ptr<Primitive>>& objects) {
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
    const Vec3d v4{x_offset + x_width_half, y_offset - y_width_half, z_offset - z_width_half};
    const Vec3d v5{x_offset + x_width_half, y_offset + y_width_half, z_offset - z_width_half};
    const Vec3d v6{x_offset - x_width_half, y_offset + y_width_half, z_offset - z_width_half};
    const Vec3d v7{x_offset - x_width_half, y_offset - y_width_half, z_offset - z_width_half};

//    const Color color{192.0 / 255, 155.0 / 255, 94.0 / 255};
    const Color color = Color::light_gray();
    Material mat{color};
    mat.ks = 0.4;
    mat.ka = 0.2;
    mat.kd = 0.5;
    mat.kr = 0.1;
    vector<Triangle> triangles;


    // front
    // 0---3
    // | \ |
    // 1---2
    triangles.emplace_back(Triangle{v0, v1, v2, mat});
    triangles.emplace_back(Triangle{v0, v2, v3, mat});
//    objects.push_back(make_shared<Triangle>( v0, v1, v2, color));
//    objects.push_back(make_shared<Triangle>( v0, v2, v3, color));
    // right
    // 3---5
    // | \ |
    // 2---4
    triangles.emplace_back(Triangle{v3, v2, v4, mat});
    triangles.emplace_back(Triangle{v3, v4, v5, mat});
//    objects.push_back(make_shared<Triangle>( v3, v2, v4, color));
//    objects.push_back(make_shared<Triangle>( v3, v4, v5, color));
    // back
    // 5---6
    // | \ |
    // 4---7
    triangles.emplace_back(Triangle{v5, v4, v7, mat});
    triangles.emplace_back(Triangle{v5, v7, v6, mat});
//    objects.push_back(make_shared<Triangle>( v5, v4, v7, color));
//    objects.push_back(make_shared<Triangle>( v5, v7, v6, color));
    // left
    // 6---0
    // | \ |
    // 7---1
    triangles.emplace_back(Triangle{v6, v1, v0, mat});
    triangles.emplace_back(Triangle{v6, v7, v1, mat});
//    objects.push_back(make_shared<Triangle>( v6, v7, v1, color));
//    objects.push_back(make_shared<Triangle>( v6, v1, v0, color));
    // top
    // 6---5
    // | \ |
    // 0---3
    triangles.emplace_back(Triangle{v6, v0, v3, mat});
    triangles.emplace_back(Triangle{v6, v3, v5, mat});
//    objects.push_back(make_shared<Triangle>( v6, v0, v3, color));
//    objects.push_back(make_shared<Triangle>( v6, v3, v5, color));
    // bottom
    // 4---7
    // | \ |
    // 2---1
    triangles.emplace_back(Triangle{v4, v2, v1, mat});
    triangles.emplace_back(Triangle{v4, v1, v7, mat});
//    objects.push_back(make_shared<Triangle>( v7, v1, v2, color));
//    objects.push_back(make_shared<Triangle>( v7, v1, v2, color));
    shared_ptr<Model> model{make_shared<Model>(mat, triangles)};
    objects.push_back(model);
}


void
preview(cv::Mat& img, unsigned int& finPix, unsigned int sumPix, int delay = 1000)
{
    int key;
    double progress, curElapsed, lastElapsed;
    unsigned int curFinPix{0}, lastFinPix{0};
    bool printedProgress{false};
    const unsigned int progressInterval{5};
    while(processing)
    {
        cv::Mat tmpimg{img.clone()};
//        for (int n=0; n<numProg; ++n) {
//            curFinPix += finPixArr[n];
//            progress += (double)(finPixArr[n])/sumPixArr[n];
//        }
        progress = (float) finPix / sumPix * 100;

        curElapsed = timer.elapsed();
        unsigned int curFinPix{finPix};
        const double pixPerSec{((double)(curFinPix - lastFinPix))/(curElapsed - lastElapsed)};
        lastFinPix = curFinPix;
        lastElapsed = curElapsed;
        const string text{to_string((unsigned int)progress) + "%; t: " + to_string((unsigned int)curElapsed)
                          + " s; " + to_string((unsigned int) pixPerSec) + " pix/s"};
        const int fontFace{CV_FONT_HERSHEY_SIMPLEX};
        const double fontScale{0.8};
        const cv::Scalar color{0,255,0};
        const int thickness{1};
        int baseLine{0};
        const cv::Size textSize{cv::getTextSize(text, fontFace, fontScale, thickness+2, &baseLine)};
        const int x{10};
        const int y{10};
        const int border{12};
        const double alpha{0.2};

        if (!((unsigned int)progress % progressInterval) && !printedProgress){
            cout << "... " << text << endl;
            printedProgress = true;
        }

        printedProgress = !((unsigned int)progress % progressInterval);

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


double calcDist(const vector<shared_ptr<Primitive>>& objects,
                const Ray& ray,
                Vec3d& hitNormal
) {
    double dist{::std::numeric_limits<double>::max()}, tmpdist;
    Vec3d tmpnormal;
    // get closest intersection
    for (const auto& o : objects) {
        if (o->intersect(ray, tmpdist, tmpnormal) && tmpdist < dist) {
            dist = tmpdist;
            hitNormal = tmpnormal;
        }

    }
    return dist;
}

Color
 trace(const vector<shared_ptr<Primitive>>& objects,
      const vector<Light>& lights,
      const Color& background,
      const Ray& ray,
      int depth) {
    double dist{::std::numeric_limits<double>::max()}, tmpdist;
    const double bias{0.0001};   // bias origin of reflective/refractive/shadow ray a little into normal direction to adjust for precision problem
    Color color{background};
    Vec3d tmpnormal, hitNormal;
    Primitive* cur_obj{nullptr};

    // get closest intersection
    for (const auto& o : objects) {
        if (o->intersect(ray, tmpdist, tmpnormal)) {
            if (tmpdist < dist) {
                dist = tmpdist;
                hitNormal = tmpnormal;
                cur_obj = &(*o);
            }
        }
    }

    if (cur_obj != nullptr) {
        color = Color(0.0);
        const Material& curMaterial = cur_obj->material;
        const Vec3d hitPt{ray.getPoint(dist)};

        // shade pixel
        for (const auto& l : lights) {
            Vec3d lv{l.pos - hitPt};
            const double ldist{lv.length()};
            lv.normalize();
            bool inShadow{false};

            // cast shadow ray and check if other object is blocking light
            const Ray shadow_ray{hitPt + hitNormal * bias, lv};
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


            // diffuse shading
            const double cosPhi{
                    max(hitNormal.dotProduct(lv), 0.0)};    // todo: check why we have to clip negative values
            color += curMaterial.color * l.color * curMaterial.kd * (cosPhi / (ldist * ldist));    // todo: change color to diffuse color

            // specular shading
            const Vec3d reflectRay{hitNormal * 2 * dotProduct(hitNormal,lv) - lv};
            const double cosAlpha{
                    max(reflectRay.dotProduct(lv), 0.0)};    // todo: check why we have to clip negative values
            color += l.color * curMaterial.ks * (pow(cosAlpha, curMaterial.specRefExp) / (ldist * ldist));     // todo: add reflective color here
//            color.clamp(0, 1);
        }

        ++depth;


        constexpr double REFRACTIVE_INDEX_AIR{1.0};
        Vec3d refractNormal{hitNormal};
        double cosIncedent, cosTransmission, sinSqrPhit;
        double n1, n2, n;  // refractive index we come from (n1) and go to (n2)
        double rCoef;    // reflective/refractive coefficient based on angle

        // fresnel equation
        if (curMaterial.refractive && depth<=MAX_DEPTH) {
            cosIncedent = ray.direction.dotProduct(hitNormal);     // is <0 if ray hits outside object hull (ray pointing towards object)
            if (cosIncedent < 0.0) {
                // ray from air to object
                n1 = REFRACTIVE_INDEX_AIR;   // assuming air other medium
                n2 = curMaterial.refractiveIdx;
                cosIncedent = -cosIncedent;
            } else {
                // ray from object to air (inside)
                n1 = curMaterial.refractiveIdx;
                n2 = REFRACTIVE_INDEX_AIR;   // assuming air other medium
                refractNormal = -refractNormal;
            }
            n = n1/n2;
            sinSqrPhit = n * n * (1 - cosIncedent * cosIncedent);    // if sinSqrPhit is greater than 1 it is not valid
            if (sinSqrPhit <= 1) {
                cosTransmission = sqrt(1-sinSqrPhit);
                const double Rorth{(n1*cosIncedent - n2*cosTransmission)/(n1*cosIncedent + n2*cosTransmission)};
                const double Rpar{ (n2*cosIncedent - n1*cosTransmission)/(n2*cosIncedent + n1*cosTransmission)};
                rCoef = (Rorth*Rorth + Rpar*Rpar) / 2;
            } else {
                rCoef = 1;
            }

        }

        // reflective shading recursive tracing
        Color colorReflect{0};
        if (curMaterial.reflective && depth <= MAX_DEPTH) {
            const Vec3d reflectDir{-hitNormal * 2 * dotProduct(hitNormal,ray.direction) + ray.direction};
            colorReflect = curMaterial.kr * trace(objects, lights, background, Ray{hitPt + hitNormal * bias, reflectDir}, depth);
        }
        // refractive shading recursive tracing
        Color colorRefract{0};
        if (curMaterial.refractive && depth <= MAX_DEPTH && sinSqrPhit <= 1) {
            // if sinSqrPhit > 1.0 we cannot find a transmission vector -> we have 'total internal reflection' (TRI) or critical angle
                // todo: handle tri and critical angle
            const Vec3d refractDir{ray.direction * n + refractNormal * (n * cosIncedent - cosTransmission)};
            colorRefract = curMaterial.kt * trace(objects, lights, background, Ray{hitPt - refractNormal * bias, refractDir}, depth);
        }


        color += curMaterial.color * curMaterial.ka;
        if (curMaterial.refractive) {
            color += rCoef * colorReflect + (1 - rCoef) * colorRefract;
        } else {
            color += colorReflect;
        }

        color.clamp(0, 1);



//        if (depth == MAX_DEPTH)
//        {
//            cout << "max depth reached: color: " << color << " factor r: " << rCoef << endl
//                 << "\tcurMaterial.reflective: " << (curMaterial.reflective ? "true" : "false") << endl
//                 << "\tcurMaterial.refractive: " << (curMaterial.refractive ? "true" : "false") << endl
//                 << "\tcolorReflect: " << colorReflect << endl
//                 << "\tcolorRefract: " << colorRefract << endl;
//        }
    }

    return color;
}

void
render(ImageType* img, const unsigned int x_start, const unsigned int y_start, const unsigned int cH,
       const unsigned int cW, const Camera& camera, const vector<shared_ptr<Primitive>>& objects,
       const vector<Light>& lights, const Color& background, unsigned int& finPix
#if USE_OPENMP == 0
       ,unsigned int& thread_count
#endif
) {
    const unsigned int W{camera.imWidth};
    const unsigned int H{camera.imHeight};

#if USE_OPENMP == 1
    #pragma omp parallel for schedule(dynamic,10)       // OpenMP
#endif
    for (unsigned int y = y_start; y < cH+y_start; ++y) {
        ImageType* img_ptr{img + 3 * (y*W + x_start)};
        for (unsigned int x = x_start; x < cW+x_start; ++x) {
            const Ray ray = camera.castRay(x, y);

            // trace primary/camera ray
            Color px_color{trace(objects, lights, background, ray, 0)};
//            Color px_color{calcDist(objects, ray)};

            // Using Opencv bgr
            *(img_ptr++) = (ImageType) (px_color.b * 255);
            *(img_ptr++) = (ImageType) (px_color.g * 255);
            *(img_ptr++) = (ImageType) (px_color.r * 255);

            ++finPix;   // todo: make this thread-safe and efficient
        }
    }


#if USE_OPENMP == 0
    {
        lock_guard<mutex> guard(RENDER_MUTEX);
        --thread_count;
    }
#endif
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
                                   cv::Scalar{0,255,255},
                                   cv::Scalar{125, 125, 125},
                                   cv::Scalar{0, 125, 255},
                                   cv::Scalar{125,255,255},
                                   cv::Scalar{125, 255, 0},
                                   cv::Scalar{255, 125, 0},
                                   cv::Scalar{255, 255, 125},
                                   cv::Scalar{255, 125, 255}};
    // color each part differently
    cv::Mat tile{img(cv::Rect{x_start, y_start, pW, pH})};
    tile = bgColors[num%13]*0.3;
    const cv::Size textSize{cv::getTextSize(text, fontFace, fontScale, thickness, &baseline)};
//    cv::putText(tile, text, cv::Point{pW / 2 - textSize.width/2, pH / 2}, fontFace, fontScale, cv::Scalar::all(255), thickness);
}


void
create_scene(vector<shared_ptr<Primitive>>& objects, vector<Light>& lights) {
    lights.emplace_back(Light{Vec3d{0, 3, -7.5}, Color::white() * 3});
//    lights.emplace_back(Light{Vec3d{0, 8, -9}, Color::white()*0.5});
    lights.emplace_back(Light{Vec3d{0, 0, -1}, Color::white() * 10});
//    lights.emplace_back(Light{Vec3d{5, -5, -2}, Color::white()*0.5});
//    lights.emplace_back(Light{Vec{-30,-20,1}});
    const Material glass(Color::glass(), 0.1, 0.1, 0.2, 0.8, 0.8, 8, 1.5, true, true);
    const Material glass2(Color::glass(), 0, 0, 0.2, 0.8, 0.2, 8, 1.5, true, true);
    const Material porcelain(Color::white(), 0.2, 0.5, 0.7, 0.3, 0, 8, 0, true);
    const Material mirror(Color::white(), 0, 0, 0, 1, 1, 8, 0, true);
    const Material m1(Color::light_gray(), 0.1, 0.7, 0.4, 0.3, 0, 8, 0, true);
    const Material m2(Color::white(), 0.1, 0.7, 0.4);

    objects.push_back(make_shared<Sphere>(Vec3d{0, 0, -8}, 1, Material(Color::red(), 0, 0, 0, 1, 1, 8, 0, true)));
    objects.push_back(make_shared<Sphere>(Vec3d{2, 0.25, -8}, 0.75, Material{Color{1, 1, 0}, 0.2, 0.7, 0}));
//    objects.push_back(make_shared<Sphere>(Vec3d{0, 1, -3}, 0.5, glass));
    objects.push_back(make_shared<Sphere>(Vec3d{-2.5, 2, -5}, 1, Material{Color{1, 0, 1}, 0.2, 0.5, 0.7}));

    create_box(objects);
    const string mesh_root{"/home/chris/shared/github/chris/raytracer/data/3d_meshes/"};
    string bunny_res4_path{mesh_root+"bunny/reconstruction/bun_zipper_res4.ply"};
    string bunny_res2_path{mesh_root+"bunny/reconstruction/bun_zipper_res2.ply"};
    string bunny_path{mesh_root+"bunny/reconstruction/bun_zipper.ply"};
    shared_ptr<Model> bunny{Model::load_ply(bunny_path, porcelain, true)};     // glass bunny
//    shared_ptr<Model> bunny{Model::load_ply(bunny_path, Material(Color::white(), 0.2, 0.5, 0.8, 0.2))};
    *bunny *= Mat3d::rotation(M_PI/8, M_PI/6, 0);
//    *bunny *= Mat3d::rotationX(M_PI/6);
//    *bunny *= Mat3d::rotationZ(M_PI/2);
//    *bunny *= Mat3d::rotationY(M_PI/4);
    bunny->scale(20);
    *bunny += Vec3d{-1.5, -3, -6};
    objects.push_back(bunny);


//    string draon_res4_path{mesh_root+"dragon_recon/dragon_vrip_res4.ply"};
//    string draon_path{mesh_root+"dragon_recon/dragon_vrip.ply"};
//    shared_ptr<Model> dragon{Model::load_ply(draon_path)};
//    dragon->scale(15);
//    dragon->translate(Vec3d{2, -2, -7.5});
//    objects.push_back(dragon);


//    string buddha_res4_path{mesh_root+"happy_recon/happy_vrip_res4.ply"};
//    string buddha_path{mesh_root+"happy_recon/happy_vrip.ply"};
//    shared_ptr<Model> buddha{Model::load_ply(buddha_path)};
//    buddha->scale(15);
//    buddha->translate(Vec3d{2, -4, -7.5});
//    buddha->material.ks = 0.9;
//    objects.push_back(buddha);


    // planes
    const int box_len{4};
    // back
    const Color wall_color{192.0 / 255, 155.0 / 255, 94.0 / 255};
    objects.push_back(make_shared<Plane>(0, 0, 1, box_len + 8, wall_color));
    // left
    objects.push_back(make_shared<Plane>(1, 0, 0, box_len, Color::red()));
    // right
//    objects.push_back(make_shared<Plane>(-1, 0, 0, box_len, Material{Color::green(), 0,0,0,1,0,8, 0, true}));   // mirror
    objects.push_back(make_shared<Plane>(-1, 0, 0, box_len, Color::green()));
    // bottom
    objects.push_back(make_shared<Plane>(0, 1, 0, box_len, Color::blue()));
    // top
    objects.push_back(make_shared<Plane>(0, -1, 0, box_len, wall_color));
    // behind camera
    objects.push_back(make_shared<Plane>(0, 0, -1, box_len, Color(1,1,0)));

//    s.radius = 10;

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
#if MT_TRIANGLE_INTERSECT == 1
    cout << "... using Moeller-Trumbore algorithm for triangle intersection calculation" << endl;
#endif
    cout << "... write to file: " << outFilename << endl;

#if USE_OPENMP == 1
    cout << "... using OpenMP for parallelization" << endl;
#else
    cout << "... using c++ 11 threads for parallelization" << endl;
#endif
//    check_op_overloading();

    // resolution has to be even
    constexpr unsigned int downScale{1};
    constexpr unsigned int imHeight{600/downScale};
    constexpr unsigned int imWidth{800/downScale};
    ImageType* img_ptr = new ImageType[imWidth*imHeight*3];
    memset(img_ptr, 0, sizeof(ImageType)*imWidth*imHeight*3);
    cv::Mat img{imHeight, imWidth, CV_8UC3, img_ptr};
    cv::namedWindow(winName, CV_WINDOW_AUTOSIZE);

//    constexpr unsigned int H = 250;
//    constexpr unsigned int W = 350;
    constexpr unsigned int MAX_VAL = 255;
    constexpr double ASPECT_RATIO = (double) imWidth / imHeight;
    constexpr double FOV = 60;

    Color background{0, 0.5, 0.5};
    Camera camera{Vec3d{0,0,0}, Vec3d{0,1,0}, Vec3d{0,0,-1}, ASPECT_RATIO, FOV, imWidth, imHeight};
    camera.rotate(0,0,M_PI/16);
//    camera.rotate(0,0,0);
//    camera.move(Vec3d{-0.5,0,-0.25});

    vector<shared_ptr<Primitive>> objects;
    vector<Light> lights;
    create_scene(objects, lights);
    unsigned int finPix{0};

    // split image into tiles (2x4)
    const unsigned int nXTiles{20};
    const unsigned int nYTiles{20};
    const unsigned int tileWidth{imWidth/nXTiles};
    const unsigned int tileHeight{imHeight/nYTiles};

    timer.reset();
    // starting thread to show progress
    thread thread_show{preview, std::ref(img), std::ref(finPix), imWidth*imHeight, 200};

#if USE_OPENMP == 0
    // start threads
    const unsigned int max_threads{thread::hardware_concurrency()};   // max: num available threads
    cout << "... starting " << max_threads << " threads" << endl;
    vector<thread> threads;
    unsigned int x_start{0};
    unsigned int y_start{0};
    unsigned int ctr{0};
    unsigned int thread_count{0};
    while (true) {
        {
            lock_guard<mutex> guard(RENDER_MUTEX);
            threads.push_back(thread{render, img_ptr, x_start, y_start, tileHeight, tileWidth, std::ref(camera),
                                           std::ref(objects),
                                           lights, background,
                                           std::ref(finPix),
                                           std::ref(thread_count)});
            ++thread_count;
        }
//        colorize_image_tile(img, ctr++, x_start, y_start, tileWidth, tileHeight);
        x_start += tileWidth;
        if (x_start >= imWidth) {
            x_start = 0;
            y_start += tileHeight;
            if (y_start >= imHeight)
                break;
        }
        // wait until new threads can be started
        while (thread_count >= max_threads) {
            this_thread::sleep_for(chrono::milliseconds(100));
        }
    }

        // wait for other threads to finish
    cout << "... waiting for threads to finish " << endl;
    for (auto& t : threads) {
        if (t.joinable())
            t.join();
    }
#else
    render(img_ptr, 0, 0, imHeight, imWidth, camera, objects, lights, background, finPix);
#endif
    cout << "... finished rendering (" << timer.elapsed() << "s)" << endl;
    processing = false;
    if (thread_show.joinable())
        thread_show.join();

    cv::imshow(winName, img);
    cout << "... write image " << outFilename << endl;
    cv::imwrite(outFilename, img);
    cv::waitKey();
    delete[] img_ptr;
}
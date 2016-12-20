#include <iostream>
#include <math.h>
#include <fstream>
#include <vector>
#include <memory>
#include <limits>


// url: https://www.scratchapixel.com/lessons/3d-basic-rendering/introduction-to-ray-tracing

using namespace std;

/** 3D vector in cartesian space.
 *
 */
struct Vec {
    double x, y, z;
    Vec() : x{0}, y{0}, z{0} {}
    Vec(double x_, double y_, double z_) : x{x_}, y{y_}, z{z_} {}
    Vec(const Vec& v) : x{v.x}, y{v.y}, z{v.z} {}

    // const shows that the operator does not change the class
    Vec operator- (const Vec& rhs) const
    {
        Vec temp;
        temp.x = x - rhs.x;
        temp.y = y - rhs.y;
        temp.z = z - rhs.z;
        return temp;
    }

    Vec operator/ (const double v) const {
        return Vec{x/v, y/v, z/v};
    }

    Vec operator+ (const Vec& rhs) const {
        return Vec{x+rhs.x, y+rhs.y, z+rhs.z};
    }

    Vec operator* (double val) const {
        return Vec{x*val, y*val, z*val};
    }

    double dotProduct(const Vec& vec2) const
    {
        return (x * vec2.x) +
               (y * vec2.y) +
               (z * vec2.z);
    }

    double length() const
    {
        return sqrt(dotProduct(*this));
    }

    Vec& normalize()
    {
        double l = length();
        x /= l;
        y /= l;
        z /= l;
        return *this;
    }
};

/** Line which will be used for back raytracing
 *
 */
struct Ray {
    // origin
    Vec origin;
    // direction
    Vec direction;

    Ray(Vec o, Vec d): origin{o}, direction{d} {}

    Vec getPoint(double dist) const
    {
        return origin + direction*dist;
    }
};


/** Generic base class for all objects which shall be rendered
 *
 */
struct Object {
    virtual bool intersect(const Ray& ray, double& dist) const = 0;
    virtual Vec getNormal(const Vec& vec) const = 0;

};

/** The most common object to be rendered in a raytracer
 *
 */
struct Sphere : public Object {
    // define location + radius
    Vec centerPoint;
    double radius;

    Sphere(Vec c, double r): centerPoint{c}, radius{r} {}

    // get intersection with ray: refer: https://en.wikipedia.org/wiki/Line%E2%80%93sphere_intersection
    // more details: https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-sphere-intersection
    virtual bool
    intersect(const Ray& ray, double& dist) const override
    {
        // (l * (o - c))^2 - || o - c ||^2 + r^2
        double val1, val2, val3, dist1, dist2;
        const Vec temp = ray.origin-centerPoint;
        val1 = temp.dotProduct(ray.direction);
        val2 = temp.length();
        val3 = val1*val1 - val2*val2 + radius*radius;

        if(val3 < 0) {
            return false;
        }

        // compute distance
        dist1 = -temp.dotProduct(ray.direction) + sqrt(val3);
        dist2 = -temp.dotProduct(ray.direction) - sqrt(val3);

        if(dist1 < 0 || dist2 < 0) {
            dist = max(dist1,dist2);
        } else if (dist1 > 0 && dist2 > 0) {
            dist = min(dist1,dist2);
        }

        return dist > 0;
    }

    virtual Vec
    getNormal(const Vec& P) const override
    {
        // src: https://cs.colorado.edu/~mcbryan/5229.03/mail/110.htm
        Vec n{P - centerPoint};
        n = n / radius;
        return n;
    }
};


/** Clips the value to min and max.
 * If the input value lies in [min,max], it will not be changed. Otherwise it will be set to min if val < min or to
 * max if val > max.
 * @param min Min value
 * @param max Max value
 * @param val Input value
 * @return The clipped value
 */
template <typename T>
T clamp(T min, T max, T val) {
    val = val < min ? min : val;
    val = val > max ? max : val;
    return val;
}

/** Container to save pixel color information in rgb format.
 *
 */
struct Color {
    double r,g,b;
    Color(): r{0}, g{0}, b{0} {};
    Color(double r_, double g_, double b_): r{r_}, g{g_}, b{b_} {};

    Color operator*(double d) {
        return Color{r*d, g*d, b*d};
    }

    Color operator*(Color c) {
        return Color{r*c.r, g*c.g, b*c.b};
    }

    Color operator+(const Color& c) {
        return Color{r+c.r, g+c.g, b+c.b};
    }

    Color operator+= (const Color& rhs) {
        r += rhs.r;
        g += rhs.g;
        b += rhs.b;

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
        return Color(1,1,1);
    }
    static Color black() {
        return Color();
    }
    static Color red() {
        return Color(1,0,0);
    }
 };


struct Light {
    Color color;
    Vec pos;

    Light(Vec pos_, Color color_): pos{pos_}, color{color_} {}
    Light(Vec pos_): pos{pos_}, color{Color::white()} {}
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


int
main(int argc, char** argv)
{
    cout << "... start ray tracer" << endl;

    constexpr unsigned int H = 500;
    constexpr unsigned int W = 800;
    constexpr unsigned int MAX_VAL = 255;
    constexpr double ASPECT_RATIO = (double)W/H;
    constexpr double FOV = 100;

    ofstream ofs{"out2.ppm"};    // http://netpbm.sourceforge.net/doc/ppm.html
    ofs << "P3\n"
        << to_string(W) << " " << to_string(H) << "\n"
        << to_string(MAX_VAL) << "\n";

    vector<shared_ptr<Object>> objects;
    objects.push_back(make_shared<Sphere>(Vec{0,0,-20}, 5));
//    objects.push_back(make_shared<Sphere>(Vec{10,0,-20}, 5));
    objects.push_back(make_shared<Sphere>(Vec{2,1,-15}, 1));
    objects.push_back(make_shared<Sphere>(Vec{4,4,-22}, 2.5));
    objects.push_back(make_shared<Sphere>(Vec{80,-6,-150}, 5));
    objects.push_back(make_shared<Sphere>(Vec{-4,4,-5}, 2.5));
    objects.push_back(make_shared<Sphere>(Vec{0,4,-5}, 2.5));

//    s.radius = 10;
    Color background{0,0.5,0.5};
    Color scolor = Color::red();

    Color* img = new Color[W*H];
    double* zbuff = new double[W*H];
    Color* img_ptr = img;
    const Vec origin{0,0,0};  // center of projection

    vector<Light> lights;
    lights.emplace_back(Light{Vec{30,0,-2}, Color::white()});
//    lights.emplace_back(Light{Vec{-30,-20,1}});

    img_ptr = img;
    for (unsigned int y = 0; y<H; ++y) {
        for (unsigned int x = 0; x<W; ++x) {

            const double px_ndc = (x+0.5)/W;
            const double py_ndc = (y+0.5)/H;
            const double cam_x = (2*px_ndc  - 1) * ASPECT_RATIO * tan(deg2rad(FOV)/2);
            const double cam_y = (1 - 2*py_ndc) * tan(deg2rad(FOV)/2);

            Vec d{cam_x, cam_y, -1};
            d.normalize();
            const Ray ray{origin, d};

            double dist{std::numeric_limits<double>::max()}, tmpdist;
            Color px;
            bool intersect{false};
            Object* curo{nullptr};


            // check intersections
            for (const auto& o : objects) {
                if ( o->intersect(ray, tmpdist) ) {
                    if (tmpdist < dist) {
                        dist = tmpdist;
                        curo = &(*o);
                    }
                }
            }

            if (curo != nullptr) {
                Vec pintersect{ray.getPoint(dist)};
                const Vec n{curo->getNormal(pintersect)};

                for (const auto& l : lights) {
                    Vec lv{l.pos - pintersect};
//                    Vec lv{l.pos};
                    lv.normalize();
                    bool inShadow{false};

                    // check if object is blocking light
                    Ray shadow_ray{pintersect, lv};
                    for (const auto& o : objects) {
                        if (&(*o) == curo)
                            continue;
                        if ( o->intersect(shadow_ray, tmpdist) ) {
                            inShadow = true;
                            break;
                        }
                    }
                    if (inShadow) {
                        continue;
                    }

                    const double diff_factor{n.dotProduct(lv)};
                    px += scolor * l.color * diff_factor;
                }


                px.clamp(0, 1);
                px = px + scolor * 0.2;
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
    for (unsigned int i = 0; i<W*H; ++i) {
        Color c = *(img_ptr++) * MAX_VAL;
        c.round();
        ofs << c;
    }


    delete[] img;
    delete[] zbuff;
}
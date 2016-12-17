#include <iostream>
#include <math.h>
#include <fstream>
#include <vector>


// url: https://www.scratchapixel.com/lessons/3d-basic-rendering/introduction-to-ray-tracing

using namespace std;


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

struct Sphere {
    // define location + radius
    Vec centerPoint;
    double radius;

    Sphere(Vec c, double r): centerPoint{c}, radius{r} {}

    // get intersection with ray: refer: https://en.wikipedia.org/wiki/Line%E2%80%93sphere_intersection
    // more details: https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-sphere-intersection
    bool
    intersect(const Ray &ray, double &distance) const
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
            distance = max(dist1,dist2);
        } else if (dist1 > 0 && dist2 > 0) {
            distance = min(dist1,dist2);
        }

        return true;
    }

    Vec
    getNormal(const Vec& P) const
    {
        // src: https://cs.colorado.edu/~mcbryan/5229.03/mail/110.htm
        Vec n{P - centerPoint};
        n = n / radius;
        return n;
    }
};

template <typename T>
T clamp(T min, T max, T val) {
    val = val < min ? min : val;
    val = val > max ? max : val;
    return val;
}

struct Color {
    double r,g,b;
    Color(): r{0}, g{0}, b{0} {};
    Color(double r_, double g_, double b_): r{r_}, g{g_}, b{b_} {};

    Color operator*(double d) {
        return Color{r*d, g*d, b*d};
    }

    Color operator+(const Color& c) {
        return Color{r+c.r, g+c.g, b+c.b};
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

ostream&
operator<<(ostream& os, const Color& c) {
    os << c.r << " " << c.g << " " << c.b << " ";
    return os;
}


int
main(int argc, char** argv)
{
    cout << "... start ray tracer" << endl;

    constexpr unsigned int H = 500;
    constexpr unsigned int W = 800;
    constexpr unsigned int MAX_VAL = 255;
    constexpr double ASPECT_RATIO = (double)W/H;
    constexpr double FOV = 45;

    ofstream ofs{"out2.ppm"};    // http://netpbm.sourceforge.net/doc/ppm.html
    ofs << "P3\n"
        << to_string(W) << " " << to_string(H) << "\n"
        << to_string(MAX_VAL) << "\n";

    vector<Sphere> spheres;
    spheres.push_back(Sphere{Vec{0,0,-20}, 5});
    spheres.push_back(Sphere{Vec{4,4,-22}, 2.5});
    spheres.push_back(Sphere{Vec{-4,4,-5}, 2.5});

//    s.radius = 10;
    Color background{0,0.5,0.5};
    Color scolor = Color::red();

    Color* img = new Color[W*H];
    double* zbuff = new double[W*H];
    double* zbuff_ptr = zbuff;
    Color* img_ptr = img;
    const Vec origin{0,0,0};  // center of projection
    const Vec light{100,0,0};

    // initialize z buffer to inifinity
    for (unsigned int i = 0; i<W*H; ++i) {
        *(zbuff_ptr++) = 2000000;
    }

    zbuff_ptr = zbuff;
    for (unsigned int y = 0; y<H; ++y) {
        for (unsigned int x = 0; x<W; ++x) {

            const double px_ndc = (x+0.5)/W;
            const double py_ndc = (y+0.5)/H;
            const double cam_x = (2*px_ndc  - 1) * ASPECT_RATIO * tan(FOV/2);
            const double cam_y = (1 - 2*py_ndc) * tan(FOV/2);

            Vec d{cam_x, cam_y, -1};
            d.normalize();
            const Ray ray{origin, d};

            double dist;
            Color px = background;

            // check intersections
            for (const auto& s : spheres) {
                if ( s.intersect(ray, dist) ) {
                    if (*zbuff_ptr < dist) {
                        continue;
                    }
                    *zbuff_ptr = dist;
                    Vec pintersect = ray.getPoint(dist);
                    const Vec n = s.getNormal(pintersect);
                    Vec lv = light-pintersect;
                    lv.normalize();
                    const double diff_factor = n.dotProduct(lv);
                    px = scolor * diff_factor;
                    px.clamp(0,1);
                }
            }
            zbuff_ptr++;
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
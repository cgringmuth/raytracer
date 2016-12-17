#include <iostream>
#include <math.h>
#include <fstream>


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

    double dotProduct(const Vec& vec2) const
    {
        return (x * vec2.x) +
               (y * vec2.y) +
               (z * vec2.z);
    }

    double lenght() const
    {
        return sqrt(dotProduct(*this));
    }

    Vec& normalize()
    {
        double l = lenght();
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
};

struct Sphere {
    // define location + radius
    Vec centerPoint;
    double radius;

    // get normal (low prior)

    // get intersection with ray: refer: https://en.wikipedia.org/wiki/Line%E2%80%93sphere_intersection
    // more details: https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-sphere-intersection
    bool intersection(const Ray &ray, double &distance)
    {
        // (l * (o - c))^2 - || o - c ||^2 + r^2
        double val1, val2, val3, dist1, dist2;
        const Vec temp = ray.origin-centerPoint;
        val1 = temp.dotProduct(ray.direction);
        val2 = temp.lenght();
        val3 = val1*val1 - val2*val2 + radius*radius;
        if(val3 < 0) {
            return false;
        }
//
//        // compute distance
//        dist1 = -temp.dotProduct(ray.direction) + sqrt(val3);
//        dist2 = -temp.dotProduct(ray.direction) - sqrt(val3);
//
//        if(dist1 < 0 && dist2 > 0){
//            distance = dist2;
//        }else if(dist1 > 0 && dist2 < 0){
//            distance = dist1;
//        }else if (dist1 > 0 && dist2 > 0){
//            distance = min(dist1,dist2);
//        }else {
//            cout << "Unexpected distance values";
//            return false;
//        }

        return true;
    }
};


struct Color {
    double r,g,b;
    Color(): r{0}, g{0}, b{0} {};
    Color(double r_, double g_, double b_): r{r_}, g{g_}, b{b_} {};

    Color operator*(double d) {
        return Color{r*d, g*d, b*d};
    }

    Color& mult(double d) {
        r *= d;
        g *= d;
        b *= d;

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

int
main(int argc, char** argv)
{
    cout << "... start ray tracer" << endl;

    constexpr unsigned int H = 500;
    constexpr unsigned int W = 500;
    constexpr unsigned int MAX_VAL = 255;

    ofstream ofs{"out.ppm"};    // http://netpbm.sourceforge.net/doc/ppm.html
    ofs << "P3\n"
        << to_string(W) << " " << to_string(H) << "\n"
        << to_string(MAX_VAL) << "\n";
    Sphere s;

    s.centerPoint = Vec(0, 0, 50);
    s.radius = 49;
    Color background = Color::black();
    Color scolor = Color::red();

    Color img[W*H];
    Vec origin{0,0,0};

    for (unsigned int y = 0; y<H; ++y) {
        for (unsigned int x = 0; x<W; ++x) {

            Vec d{x-W/2.0, H/2.0-y, 1};
            d.normalize();
            Ray ray{origin, d};

            double dist;
            // check intersection
            if ( s.intersection(ray, dist) ) {   // intersect -> do shading (but const color for now)
                img[x*W + y] = scolor;
                cout << "x: " << x << " y: " << y << " ";
            }
            else {  // no intersection -> background color
                img[x*W + y] = background;
            }

        }
    }
    for (unsigned int i = 0; i<W*H; ++i) {
        Color c = img[i] * MAX_VAL;
        ofs << c.r << " " << c.g << " " << c.b << " ";
    }

}
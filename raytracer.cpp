#include <iostream>
#include <fstream>


// url: https://www.scratchapixel.com/lessons/3d-basic-rendering/introduction-to-ray-tracing

using namespace std;


struct Vec {
    double x, y, z;
    Vec() : x{0}, y{0}, z{0} {}
    Vec(double x_, double y_, double z_) : x{x_}, y{y_}, z{z_} {}
    Vec(const Vec& v) : x{v.x}, y{v.y}, z{v.z} {}
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

    // get normal (low prior)

    // get intersection with ray: refer: https://en.wikipedia.org/wiki/Line%E2%80%93sphere_intersection
    // more details: https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-sphere-intersection
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
    Color background = Color::red();

    Color img[W*H];

    for (unsigned int x = 0; x<W; ++x) {
        for (unsigned int y = 0; y<H; ++y) {

            Ray ray{Vec{}, Vec{x, y, 1}};

            // check intersection
            if ( false ) {   // intersect -> do shading (but const color for now)

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
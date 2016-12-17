#include <iostream>

using namespace std;


struct Vec {
    double x, y, z;
};

struct Ray {
    // origin
    Vec origin;
    // direction
    Vec direction;
};

struct Sphere {
    // define location + radius

    // get normal (low prior)

    // get intersection with ray: refer: https://en.wikipedia.org/wiki/Line%E2%80%93sphere_intersection
    // more details: https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-sphere-intersection
};

int
main(int argc, char** argv)
{
    cout << "hello world" << endl;

    constexpr unsigned int H = 500;
    constexpr unsigned int W = 500;

}
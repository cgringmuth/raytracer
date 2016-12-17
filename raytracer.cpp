#include <iostream>
#include <math.h>

using namespace std;


struct Vec {
    double x, y, z;

    // const shows that the operator does not change the class
    Vec operator- (const Vec& rhs) const
    {
        Vec temp;
        temp.x = x - rhs.x;
        temp.y = y - rhs.y;
        temp.z = z - rhs.z;
        return temp;
    }

    double dotProduct(const Vec& vec2)
    {
        return ((x * vec2.x) + (y * vec2.y) + (z * vec2.z));
    }
};

struct Ray {
    // origin
    Vec origin;
    // direction
    Vec direction;
};

struct Sphere {
    // define location + radius
    Vec centerPointSphere;
    double radiusSphere;

    //Ray to test
    Ray RayIntersect;

    // get normal (low prior)

    // get intersection with ray: refer: https://en.wikipedia.org/wiki/Line%E2%80%93sphere_intersection
    // more details: https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-sphere-intersection
    bool intersection(const Vec &centerPointSphere, const double &radiusSphere, const Ray &RayIntersect,
                      double &distance)
    {
        // (l * (o - c))^2 - || o - c ||^2 + r^2
        double val1, val2, val3, dist1, dist2;
        val1 = (RayIntersect.origin-centerPointSphere).dotProduct(RayIntersect.direction);
        val2 = (RayIntersect.origin-centerPointSphere).dotProduct((RayIntersect.origin-centerPointSphere));
        val3 = val1 - val2 + pow(radiusSphere, 2.0);

        if(val3 < 0) {
            return false;
        }

        // compute distance
        dist1 = -(RayIntersect.origin-centerPointSphere).dotProduct(RayIntersect.direction) + sqrt(val3);
        dist2 = -(RayIntersect.origin-centerPointSphere).dotProduct(RayIntersect.direction) - sqrt(val3);

        if(dist1 < 0 && dist2 > 0){
            distance = dist2;
            return true;
        }else if(dist1 > 0 && dist2 < 0){
            distance = dist1;
            return true;
        }else if (dist1 > 0 && dist2 > 0){
            distance = min(dist1,dist2);
            return true;
        }else {
            cout << "Unexpected distance values";
            return false;
        }
    }
};

int
main(int argc, char** argv)
{
    cout << "hello world" << endl;

    constexpr unsigned int H = 500;
    constexpr unsigned int W = 500;

}
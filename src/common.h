//
// Created by chris on 13.08.17.
//

#ifndef RAYTRACER_COMMON_H_H
#define RAYTRACER_COMMON_H_H

#include <iostream>

/**
 * This enables Moeller-Trumbore algorithm for triangle intersection calculation. It is the fastest intersection
 * algorithm so far.
 */
#ifndef MT_TRIANGLE_INTERSECT
#define MT_TRIANGLE_INTERSECT   1
#endif

/**
 * Back-face culling basically leads to some performance improvements. But it only works properly with solid objects.
 * Hence, shadows might not be rendered correctly when CULLING is turned on and you want to render hulls instead
 * of solid objects. This is why it is turned of per default.
 * Refer to https://www.scratchapixel.com/lessons/3d-basic-rendering/ray-tracing-rendering-a-triangle/single-vs-Float-sided-triangle-backface-culling
 * to reader more.
 */
#ifndef CULLING
#define CULLING                 0
#endif

#ifndef USE_OPENMP
#define USE_OPENMP              0
#endif


//typedef double Float;
typedef float Float;


constexpr Float EPS{0.000001};


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

#define TO_STREAM(stream, var) (stream) << #var": " << (var)
#define TO_COUT(var) TO_STREAM(std::cout, var)

#define OUTPUT(var) #var": " << (var)
#define OUTPUTS(var) #var": " << (var) << ", "

// FIXME: Not working properly
template <typename T>
void printVars(T t) {
    std::cout << OUTPUT(t) << "\n";
}


template <typename T, typename... Rest>
void printVars(T t, Rest... rest) {
    std::cout << OUTPUT(t) << ", ";
    printVars(rest...);
}

#endif //RAYTRACER_COMMON_H_H

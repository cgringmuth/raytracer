//
// Created by chris on 13.08.17.
//

#ifndef RAYTRACER_COMMON_H_H
#define RAYTRACER_COMMON_H_H


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

#endif //RAYTRACER_COMMON_H_H

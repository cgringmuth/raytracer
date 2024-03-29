//
// Created by chris on 19.08.17.
//

#ifndef RAYTRACER_MATERIAL_H
#define RAYTRACER_MATERIAL_H

#include <ostream>
#include "common.h"
#include "color.h"


struct Material {
    Color color;
    Float ka;  // ambient reflectance
    Float kd;  // diffuse reflectance

    Float ks;  // specular reflectance
    Float specRefExp; // specular-reflection exponent

    bool reflective;
    Float kr;  // reflectance coefficient

    bool refractive;    // TODO: is this switch really necessary?
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
    Float refractiveIdx;
    Float kt;  // transmission coefficient (the amount of light which it can pass through object)

    explicit Material(Float ka=0.2, Float kd=0.7, Float ks=0.2, Float kr=0, Float kt=0, Float specRefExp=8, Float refractiveIdx=0,
                      bool reflective=false, bool refractive=false)
            : Material(Color{}, ka, kd, ks, kr, kt, specRefExp, refractiveIdx, reflective, refractive) {}
    explicit Material(const Color& color, Float ka=0.2, Float kd=0.7, Float ks=0.2, Float kr=0, Float kt=0, Float specRefExp=8,
                      Float refractiveIdx=0, bool reflective=false, bool refractive=false)
            : color(color), ka(ka), kd(kd), ks(ks), specRefExp(specRefExp), reflective(reflective), kr(kr)
            , refractive(refractive), refractiveIdx(refractiveIdx), kt{kt} {}

    friend std::ostream &operator<<(std::ostream &os, const Material &material) {
        os << "color: " << material.color << " ka: " << material.ka << " kd: " << material.kd << " ks: " << material.ks
           << " specRefExp: " << material.specRefExp << " reflective: " << std::boolalpha << material.reflective << " kr: " << material.kr
           << " refractive: " << std::boolalpha << material.refractive << " refractiveIdx: " << material.refractiveIdx << " kt: "
           << material.kt;
        return os;
    }
};

#endif //RAYTRACER_MATERIAL_H

//
// Created by chris on 13.08.17.
//

#ifndef RAYTRACER_UTILS_H
#define RAYTRACER_UTILS_H

#pragma once

#include <stdlib.h>
#include <cstring>
#include <cmath>
#include <iostream>

#include "common.h"

struct Mat3d {
    Float v[9];
    static const unsigned int width{3};
    static const unsigned int height{3};

    Mat3d(Float v0, Float v1, Float v2,
          Float v3, Float v4, Float v5,
          Float v6, Float v7, Float v8) {
        v[0] = v0; v[1] = v1; v[2] = v2;
        v[3] = v3; v[4] = v4; v[5] = v5;
        v[6] = v6; v[7] = v7; v[8] = v8;
    }

    Mat3d(const Mat3d& mat) : Mat3d{mat.v} { }
    Mat3d(const Float* v) {
        memcpy(this->v, v, sizeof(Float)*length());
    }

    Float& at(unsigned int x, unsigned int y) { return v[y*width + x]; }
    Float at(unsigned int x, unsigned int y) const { return v[y*width + x]; }

    Float& operator[](size_t idx) {
        return v[idx];
    }

    Float operator[](size_t idx) const {
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

    static Mat3d rotation(Float phiX, Float phiY, Float phiZ);

    static Mat3d rotationX(Float phi) {
        Float vmat[9];
        const Float cosphi{cos(phi)};
        const Float sinphi{sin(phi)};
        vmat[0] = 1; vmat[1] = 0;      vmat[2] = 0;
        vmat[3] = 0; vmat[4] = cosphi; vmat[5] = -sinphi;
        vmat[6] = 0; vmat[7] = sinphi; vmat[8] = cosphi;
        return Mat3d{vmat};
    }

    static Mat3d rotationY(Float phi) {
        Float vmat[9];
        const Float cosphi{cos(phi)};
        const Float sinphi{sin(phi)};
        vmat[0] = cosphi;  vmat[1] = 0; vmat[2] = sinphi;
        vmat[3] = 0;       vmat[4] = 1; vmat[5] = 0;
        vmat[6] = -sinphi; vmat[7] = 0; vmat[8] = cosphi;
        return Mat3d{vmat};
    }

    static Mat3d rotationZ(Float phi) {
        Float vmat[9];
        const Float cosphi{cos(phi)};
        const Float sinphi{sin(phi)};
        vmat[0] = cosphi;  vmat[1] = -sinphi; vmat[2] = 0;
        vmat[3] = sinphi;  vmat[4] = cosphi;  vmat[5] = 0;
        vmat[6] = 0;       vmat[7] = 0;       vmat[8] = 1;
        return Mat3d{vmat};
    }
};

/** 3D vector in cartesian space.
 *
 */
template <typename T>
struct Vec3 {
    T x, y, z;

    Vec3() : Vec3<T>{0,0,0} {}
    Vec3(const Vec3<T>& v) : Vec3<T>{v.x, v.y, v.z} {}
    Vec3(Float x, Float y, Float z) : x{x}, y{y}, z{z} {}

    Vec3<T> operator-() const {
        Vec3<T> v;
        v.x = -x;
        v.y = -y;
        v.z = -z;
        return v;
    }

    Vec3<T>& operator-=(const Vec3<T>& rhs) {
        x -= rhs.x;
        y -= rhs.y;
        z -= rhs.z;
        return *this;
    }

    Vec3<T>& operator/=(const T v) {
        const Float invV{1.0/v};
        return *this*=invV;
    }

    Vec3<T>& operator+=(const Vec3<T>& rhs) {
        x += rhs.x;
        y += rhs.y;
        z += rhs.z;
        return *this;
    }

    Vec3<T>& operator+=(const T d) {
        x += d;
        y += d;
        z += d;
        return *this;
    }

    Vec3<T>& operator*=(T val) {
        x *= val;
        y *= val;
        z *= val;
        return *this;
    }

    Vec3& operator*=(const Mat3d& mat) {
        Float tx{x*mat.at(0,0) + y*mat.at(1,0) + z*mat.at(2, 0)};
        Float ty{x*mat.at(0,1) + y*mat.at(1,1) + z*mat.at(2, 1)};
        Float tz{x*mat.at(0,2) + y*mat.at(1,2) + z*mat.at(2, 2)};
        x = tx;
        y = ty;
        z = tz;
        return *this;
    }

    Float& operator[](size_t idx) {
        switch (idx) {
            case 0:
                return x;
            case 1:
                return y;
            case 2:
                return z;
            default:
                return x;  // todo: add error handling
        }
    }

    const Float& operator[](size_t idx) const {
        switch (idx) {
            case 0:
                return x;
            case 1:
                return y;
            case 2:
                return z;
            default:
                return x;  // todo: add error handling
        }
    }

    Float dotProduct(const Vec3& vec2) const {
        return (x * vec2.x) +
               (y * vec2.y) +
               (z * vec2.z);
    }

    Float length() const {
        return sqrt(dotProduct(*this));
    }

    Vec3<T>& normalize() {
        return *this *= 1/length();
    }

    Vec3<T> cross_product(const Vec3& v) const;
};

template <typename T>
Vec3<T> cross_product(const Vec3<T>& u, const Vec3<T>& v) {
    return Vec3<T>{
            u[1] * v[2] - u[2] * v[1],
            u[2] * v[0] - u[0] * v[2],
            u[0] * v[1] - u[1] * v[0]
    };
}

template <typename T>
Vec3<T> Vec3<T>::cross_product(const Vec3<T>& v) const {
    return ::cross_product(*this, v);
}

template <typename T>
Vec3<T> operator-(Vec3<T> lhs, const Vec3<T>& rhs) {
    return lhs -= rhs;
}

template <typename T>
Vec3<T> operator/(Vec3<T> lhs, const Float v) {
    return lhs /= v;
}

template <typename T>
Vec3<T> operator+(Vec3<T> lhs, const Vec3<T>& rhs) {
    return lhs += rhs;
}

template <typename T>
Vec3<T> operator+(Vec3<T> lhs, const Float d) {
    return lhs += d;
}

template <typename T>
Vec3<T> operator*(Vec3<T> lhs, const Float val) {
    return lhs *= val;
}

template <typename T>
Vec3<T> operator*(const Float val, Vec3<T> lhs) {
    return lhs *= val;
}


template <typename T>
Float dotProduct(const Vec3<T>& v1, const Vec3<T>& v2) {
    return v1.dotProduct(v2);
}

template <typename T>
Vec3<T> operator*(Vec3<T> lhs, const Mat3d& rhs) {
    return lhs *= rhs;
}

template <typename T>
std::ostream&
operator<<(std::ostream& os, const Vec3<T>& v) {
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


typedef Vec3<Float> Vec3f;


/** Line which will be used for back raytracing
 *
 */
struct Ray {
    // origin
    Vec3f origin;
    // direction
    Vec3f direction;

    Ray(Vec3f o, Vec3f d) : origin{o}, direction{d} {}

    Vec3f getPoint(Float dist) const {
        return origin + (direction * dist);
    }
};



#endif //RAYTRACER_UTILS_H

//
// Created by chris on 19.08.17.
//

#ifndef RAYTRACER_COLOR_H
#define RAYTRACER_COLOR_H

#include "common.h"
#include <cmath>
#include <ostream>


/** Container to save pixel color information in rgb format.
 *
 */
struct Color {

    Float r, g, b;

    Color() : Color{0} {};
    explicit Color(const Float v) : Color{v,v,v} {};
    Color(const Color& color) : Color{color.r, color.g, color.b} {};
    Color(Float r, Float g, Float b) : r{r}, g{g}, b{b} {};

    Color& operator/=(Float d) {
        const Float invD{1/d};
        return *this*=invD;
    }

    Color& operator*=(Float d) {
        r *= d;
        g *= d;
        b *= d;
        return *this;
    }

    Color& operator*=(Color c) {
        r *= c.r;
        g *= c.g;
        b *= c.b;
        return *this;
    }

    Color& operator+=(const Color& rhs) {
        r += rhs.r;
        g += rhs.g;
        b += rhs.b;
        return *this;
    }

    Color& clamp(Float min, Float max) {
        r = ::clamp(min, max, r);
        g = ::clamp(min, max, g);
        b = ::clamp(min, max, b);
        return *this;
    }

    Color& round() {
        r = std::round(r);
        g = std::round(g);
        b = std::round(b);
        return *this;
    }

    static Color white() {
        return Color(1, 1, 1);
    }

    static Color black() {
        return Color();
    }

    static Color red() {
        return Color(1, 0, 0);
    }

    static Color green() {
        return Color(0, 1, 0);
    }

    static Color blue() {
        return Color(0, 0, 1);
    }

    static Color gray() {
        return Color(0.5);
    }

    static Color light_gray() {
        return Color(0.75);
    }

    static Color yellow() {
        return Color(1,1,0);
    }

    static Color glass() {
        return Color(0.788, 0.858, 0.862);
    }

    friend Color operator/(Color lhs, const Float d) {
        return lhs /= d;
    }

    friend Color operator+(Color lhs, const Color& rhs) {
        return lhs += rhs;
    }

    friend Color operator*(Color lhs, const Float d) {
        return lhs *= d;
    }

    friend Color operator*(const Float d, Color rhs) {
        return rhs *= d;
    }

    friend Color operator*(Color lhs, const Color& rhs) {
        return lhs *= rhs;
    }

    friend std::ostream &operator<<(std::ostream &os, const Color &color) {
        os << "r: " << color.r << " g: " << color.g << " b: " << color.b;
        return os;
    }
};

#endif //RAYTRACER_COLOR_H

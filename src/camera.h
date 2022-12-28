//
// Created by chris on 13.08.17.
//


#pragma once

#include <utility>
#include <vector>
#include "container.h"
#include "helper.h"


class Camera {
    Vec3f eye;  // Camera position
    Vec3f up;   // up direction_ (usually [0,1,0])
    Vec3f right;
    Vec3f at;   // Look at direction_
    Float aspectRatio;
    Float fov;     // field of view
    unsigned int imWidth;
    unsigned int imHeight;

    std::vector<Point2D> antiAliasingPattern;

public:
    Camera(Float aspectRatio, Float fov, unsigned int imWidth, unsigned int imHeight) :
            Camera{Vec3f{0, 0, 0}, Vec3f{0, 1, 0}, Vec3f{0, 0, -1}, aspectRatio, fov, imWidth, imHeight} {}

    Camera(const Vec3f &eye, const Vec3f &up, const Vec3f &at, Float aspectRatio, Float fov, unsigned int imWidth,
           unsigned int imHeight, std::vector<Point2D> antiAliasingPattern=defaultAntiAliasingPattern()) :
            eye(eye), up(up), at(at), aspectRatio(aspectRatio), fov(fov), imWidth(imWidth),
                                    imHeight(imHeight), antiAliasingPattern(std::move(antiAliasingPattern)) {
        this->up.normalize();
        this->at.normalize();
        right = cross_product(at, up);
    }

    static
    std::vector<Point2D> defaultAntiAliasingPattern() {
        return {Point2D(0.5, 0.5),
                Point2D(0.25, 0.25),
                Point2D(0.75, 0.25),
                Point2D(0.75, 0.75),
                Point2D(0.25, 0.75),
                Point2D(1.0, 0),
                Point2D(1.0, 1.0),
                Point2D(0, 1.0)
        };
    }

    static
    std::vector<Point2D> noAntiAliasing() {
        return {};
    }

    Ray getCamRay(const Float x, const Float y) const {
        const Float px_ndc{x / imWidth};
        const Float py_ndc{y / imHeight};
        const Float cam_x{(2 * px_ndc - 1) * aspectRatio * tan(deg2rad(fov) * 0.5)};
        const Float cam_y{(1 - 2 * py_ndc) * tan(deg2rad(fov) * 0.5)};
        Vec3f camDir{right * cam_x + up * cam_y + at};
        camDir.normalize();

        return {eye, camDir};
    }

    Ray castRay(unsigned int x, unsigned int y) const {
        return getCamRay(x + 0.5, y + 0.5);
    }

    std::vector<Ray> castRays(unsigned int x, unsigned int y) const {
        std::vector<Ray> rays{getCamRay(x, y)};
        for (const auto &aap : antiAliasingPattern) {
            rays.push_back(getCamRay(x + aap.x(), y + aap.y()));
        }

        return rays;
    }

    Camera &rotate(Float alpha, Float beta, Float gamma) {
        const Mat3d rot{Mat3d::rotation(alpha, beta, gamma)};
        up *= rot;
        up.normalize();
        at *= rot;
        at.normalize();
        right = cross_product(at, up);
        return *this;
    }

    Camera &move(const Vec3f &trans) {
        eye += trans;
        return *this;
    }

    unsigned int getImWidth() const {
        return imWidth;
    }

    unsigned int getImHeight() const {
        return imHeight;
    }

};

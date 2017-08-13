//
// Created by chris on 13.08.17.
//

#include "container.h"



Mat3d operator*(Mat3d lhs, const Mat3d& rhs) {
    return lhs *= rhs;
}

Mat3d Mat3d::rotation(Float phiX, Float phiY, Float phiZ) {
    return Mat3d::rotationZ(phiZ) * Mat3d::rotationY(phiY) * Mat3d::rotationX(phiX);
}

std::ostream& operator<<(std::ostream& os, const Mat3d& mat) {
    for(int n=0; n<mat.length()-1; ++n) {
        std::cout << mat[n] << " ";
    }
    std::cout << mat[mat.length()-1];
    return os;
}

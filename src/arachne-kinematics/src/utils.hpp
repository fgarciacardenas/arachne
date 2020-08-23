// utils.hpp
#pragma once

#include <Eigen/Dense>
#include <cmath>

using namespace Eigen;

// Constants
const double pi = M_PI;

void q2rot(Matrix3d& rotm, const double& a, const double& b, const double& c) {
    Matrix3d rx, ry, rz;
    
    rx << 1,      0,       0,
          0, cos(a), -sin(a),
          0, sin(a),  cos(a);

    ry <<  cos(b), 0, sin(b),
                0, 1,      0,
          -sin(b), 0, cos(b);

    rz << cos(c), -sin(c), 0,
          sin(c),  cos(c), 0,
               0,       0, 1;

    rotm << rz*ry*rx;
}

void rot2q(Vector3d& values, const Matrix3d& rotm) {
    double a = atan2(rotm(2,1), rotm(2,2));
    double b = atan2(-rotm(2,0), sqrt(pow(rotm(2,1),2)) + (pow(rotm(2,2),2)));
    double c = atan2(rotm(1,0), rotm(0,0));
    values << a, b, c;
}

Matrix3d skew(Vector3d& dist) {
    // Computes a skew-symmetric matrix based on a distance parameter
    Matrix3d m_skew;
    m_skew <<        0, -dist(2),  dist(1),
               dist(2),        0, -dist(0),
              -dist(1),  dist(0),        0;
    return m_skew;
}
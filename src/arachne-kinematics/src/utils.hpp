// utils.h
#pragma once

void q2rot(Matrix3d& rotm, const double& a, const double& b, const double& c) {
    Matrix3d rx, ry, rz;
    
    rx << 1, 0, 0,
          0, cos(a), -sin(a),
          0, sin(a), cos(a);

    ry << cos(b), 0, sin(b),
          0, 1, 0,
          -sin(b), 0, cos(b);

    rz << cos(c), -sin(c), 0,
          sin(c), cos(c), 0,
          0, 0, 1;

    rotm << rz*ry*rx;
}
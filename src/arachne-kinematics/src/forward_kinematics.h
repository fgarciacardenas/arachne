// forward_kinematics.hpp
#pragma once

#include "utils.h"

void dh(Matrix4d& mat, const double& theta, const double& alpha, const double& r, const double& d) {
    // This function computes an homogeneous transformation matrix based on the Denavit-Hartenberg convention.
    mat << cos(theta), -sin(theta) * cos(alpha),  sin(theta) * sin(alpha), r * cos(theta),
           sin(theta),  cos(theta) * cos(alpha), -cos(theta) * sin(alpha), r * sin(theta),
                  0.0,               sin(alpha),               cos(alpha),              d,
                  0.0,                      0.0,                      0.0,            1.0;
}

void leg2base(Matrix4d& T0, const Vector3d& posb, const Vector3d& rotb, const int& leg) {
    Matrix3d rotm;
    Matrix4d Tbase, trans;

    // Computes the position of the leg frame with respect to the base
    switch(leg) {
      case 1:
        dh(Tbase,  3*pi/4, 0, 14.1421, 0);
        break;
      case 2:
        dh(Tbase, -3*pi/4, 0, 14.1421, 0);
        break;
      case 3:
        dh(Tbase,   -pi/4, 0, 14.1421, 0);
        break;
      case 4:
        dh(Tbase,    pi/4, 0, 14.1421, 0);
        break;
      default:
        break;
    }

    // Homogeneous Transformation Matrix - Reposition with respect to the position and orientation of the base
    q2rot(rotm, rotb(0), rotb(1), rotb(2));
    trans << rotm(0, 0), rotm(0, 1), rotm(0, 2), posb(0),
             rotm(1, 0), rotm(1, 1), rotm(1, 2), posb(1),
             rotm(2, 0), rotm(2, 1), rotm(2, 2), posb(2),
             0, 0, 0, 1;
    T0 = trans * Tbase;
}

void forward_kinematics(Vector3d& pos, const Vector3d& q, const Vector3d& posb, const Vector3d& rotb, const int& leg) {
    // This function computes the forward kinematics of each leg of the robot.
    // Returns the position of the end-effector with respect to the base frame.
    Matrix4d T0, T1, T2, T3, Tend;

    // Position of the leg frame with respect to the base
    leg2base(T0, posb, rotb, leg);

    // Denavit-Hartenberg matrices
    dh(T1,            q(0), pi/2,      5.1, -5.925);
    dh(T2, q(1) + 0.610865,   pi,  12.2066,      0);
    dh(T3, q(2) - 1.303761,    0, -20.4042,      0);

    // End-effector position (x, y, z)
    Tend << T1 * T2 * T3;
    pos << Tend(0, 3), Tend(1, 3), Tend(2, 3);
}
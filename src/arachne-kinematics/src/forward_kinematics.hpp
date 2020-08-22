// forward_kinematics.hpp
#pragma once

#include <cmath>
#include <Eigen/Dense>
using namespace Eigen;

// Constants
const double pi = M_PI;

void denavit_hartenberg(Matrix4d& mat, const double& theta, const double& alpha, const double& r, const double& d) {
    mat << cos(theta), -sin(theta) * cos(alpha),  sin(theta) * sin(alpha), r * cos(theta),
           sin(theta),  cos(theta) * cos(alpha), -cos(theta) * sin(alpha), r * sin(theta),
           0.0, sin(alpha), cos(alpha), d,
           0.0, 0.0, 0.0, 1.0;
}


void reposition_leg(Matrix4d& leg_pos, const double& ang, const double& dx, const double& dy) {
    leg_pos << cos(ang), -sin(ang), 0, dx,
               sin(ang),  cos(ang), 0, dy,
               0, 0, 1, 0,
               0, 0, 0, 1;
}


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

void forward_kinematics(Vector3d& pos, const Vector3d& q, const Vector3d& posb, const Vector3d& rotb, const int& leg) {
    // This function finds the forward kinematics of each leg of the robot.
    // Returns the position of the end - effector according to the position and orientation of the base.This is possible
    // by calculating the jacobian of each leg a locating it in a homogeneous transformation matrix.
    
    Matrix3d rotm;
    Matrix4d leg_end, m_link1, m_link2, m_link3, trans, repos_leg;

    // Denavit - Hartenberg matrices
    denavit_hartenberg(m_link1,            q(0), pi/2,      5.1, 5.925);
    denavit_hartenberg(m_link2, q(1) + 0.610865,   pi,  12.2066,     0);
    denavit_hartenberg(m_link3, q(2) - 1.303761,    0, -20.4042,     0);

    // Homogeneous Transformation Matrix - Reposition in respect to position and orientation of the base
    q2rot(rotm, rotb(0), rotb(1), rotb(2));

    trans << rotm(0, 0), rotm(0, 1), rotm(0, 2), posb(0),
             rotm(1, 0), rotm(1, 1), rotm(1, 2), posb(1),
             rotm(2, 0), rotm(2, 1), rotm(2, 2), posb(2),
             0, 0, 0, 1;

    // Position of the legs with respect to the base
    if (leg == 1)
        reposition_leg(repos_leg, -3*pi/4,  14.1421, -14.1421);
    else if (leg == 2)
        reposition_leg(repos_leg,   -pi/4,  14.1421,  14.1421);
    else if (leg == 3)
        reposition_leg(repos_leg,  3*pi/4, -14.1421, -14.1421);
    else
        reposition_leg(repos_leg,    pi/4, -14.1421,  14.1421);
    
    // End - effector position(x, y, z)
    leg_end << trans * repos_leg*m_link1*m_link2*m_link3;
    pos << leg_end(0, 3), leg_end(1, 3), leg_end(2, 3);
}
// com_control.hpp
#pragma once

void rot2q(Vector3d& values, const Matrix3d& rotm) {
    double a = atan2(rotm(2,1), rotm(2,2));
    double b = atan2(-rotm(2,0), sqrt(pow(rotm(2,1),2)) + (pow(rotm(2,2),2)));
    double c = atan2(rotm(1,0), rotm(0,0));
    values << a, b, c;
}

void full_kinematics(VectorXd& pos, const Vector3d& q, const Vector3d& posb, const Vector3d& rotb, const int& leg) {
    // This function finds the forward kinematics of each leg of the robot.
    // Returns the position of the end - effector according to the position and orientation of the base. This is possible
    // by calculating the jacobian of each leg a locating it in a homogeneous transformation matrix.

    Matrix3d rotm;
    Matrix4d leg_base, leg_str, leg_mid, leg_end, m_link1, m_link2, m_link3, trans, repos_leg;
    const double r1 = 5.5;     // Distance from servo 1 to 2
    const double r2 = 7.5;     // Distance from servo 2 to 3
    const double r3 = 22.5;    // Distance from servo 3 to effector
    const double r4 = 10.253;  // Distance from base to servo 1

    // Denavit - Hartenberg matrices
    denavit_hartenberg(m_link1, q(0), pi / 2, r1, 0);
    denavit_hartenberg(m_link2, q(1) + pi / 2, pi, -r2, 0);
    denavit_hartenberg(m_link3, q(2), pi, -r3, 0);

    // Homogeneous Transformation Matrix - Reposition in respect to position and orientation of the base
    q2rot(rotm, rotb(0), rotb(1), rotb(2));

    trans << rotm(0, 0), rotm(0, 1), rotm(0, 2), posb(0),
             rotm(1, 0), rotm(1, 1), rotm(1, 2), posb(1),
             rotm(2, 0), rotm(2, 1), rotm(2, 2), posb(2),
             0, 0, 0, 1;

    // Position of the legs with respect to the base
    if (leg == 1) {
        reposition_leg(repos_leg, -3 * pi / 4, r4, -r4);
    }
    else if (leg == 2) {
        reposition_leg(repos_leg, -pi / 4, r4, r4);
    }
    else if (leg == 3) {
        reposition_leg(repos_leg, 3 * pi / 4, -r4, -r4);
    }
    else {
        reposition_leg(repos_leg, pi / 4, -r4, r4);
    }

    // Returns the position of every link in the system
    leg_base = trans * repos_leg;
    leg_str = trans * repos_leg*m_link1;
    leg_mid = trans * repos_leg*m_link1*m_link2;
    leg_end = trans * repos_leg*m_link1*m_link2*m_link3;

    // Position vector[base, leg_start, leg_middle, leg_end]: (x, y, z)
    pos << leg_base(0, 3), leg_base(1, 3), leg_base(2, 3),
        leg_str(0, 3), leg_str(1, 3), leg_str(2, 3),
        leg_mid(0, 3), leg_mid(1, 3), leg_mid(2, 3),
        leg_end(0, 3), leg_end(1, 3), leg_end(2, 3);
}


void com_kinematics(VectorXd& position, const Vector3d& q, const int& leg, const Vector3d& posb, const Vector3d& rotb, const Vector4d& w) {
    // Variables
    const double r1 = 5.5;     // Distance from servo 1 to 2
    const double r2t = 3.75;   // Distance from servo 2 to com2
    const double r2 = 7.5;     // Distance from servo 2 to 3
    const double r3t = 11;     // Distance from servo 3 to com3
    const double r3 = 22.5;    // Distance from servo 3 to end - effector
    const double r4 = 10.253;  // Distance from base to servo 1
    Matrix3d rotm;
    Matrix4d m_1, m_2t, m_2, m_3t, m_3, trans, repos_leg;

    // Denavit - Hartenberg matrices
    denavit_hartenberg(m_1, q(0), pi / 2, r1, 0);
    denavit_hartenberg(m_2t, q(1) + pi / 2, pi, -r2t, 0);
    denavit_hartenberg(m_2 ,q(1) + pi / 2, pi, -r2, 0);
    denavit_hartenberg(m_3t, q(2), pi, -r3t, 0);
    denavit_hartenberg(m_3, q(2), pi, -r3, 0);

    // Homogeneous Transformation Matrix - Reposition in respect to position and orientation of the base
    q2rot(rotm, rotb(0), rotb(1), rotb(2));

    trans << rotm(0, 0), rotm(0, 1), rotm(0, 2), posb(0),
             rotm(1, 0), rotm(1, 1), rotm(1, 2), posb(1),
             rotm(2, 0), rotm(2, 1), rotm(2, 2), posb(2),
             0, 0, 0, 1;

    // Position of the legs with respect to the base
    if (leg == 1) {
        reposition_leg(repos_leg, -3 * pi / 4, r4, -r4);
    }
    else if (leg == 2) {
        reposition_leg(repos_leg, -pi / 4, r4, r4);
    }
    else if (leg == 3) {
        reposition_leg(repos_leg, 3 * pi / 4, -r4, -r4);
    }
    else {
        reposition_leg(repos_leg, pi / 4, -r4, r4);
    }
    
    // Location of center of mass
    // Position and weight of the center of mass
    Matrix4d m_com1 = (w(0) / w(3)) * trans*repos_leg;
    Matrix4d m_com2 = (w(1) / w(3)) * trans*repos_leg*m_1*m_2t;
    Matrix4d m_com3 = (w(2) / w(3)) * trans*repos_leg*m_1*m_2*m_3t;
    Matrix4d mf = trans*repos_leg*m_1*m_2*m_3;

    // Position Vector
    Vector3d p1, p2, p3, p4;
    p1 << m_com1(0,3), m_com1(1,3), m_com1(2,3);
    p2 << m_com2(0,3), m_com2(1,3), m_com2(2,3);
    p3 << m_com3(0,3), m_com3(1,3), m_com3(2,3);
    p4 << mf(0,3), mf(1,3), mf(2,3);
    
    position << p1, p2, p3, p4;
}

void com_pos(Vector3d& com_pos, const VectorXd& q) {
    // Weights
    double w_com1 = 0.075;
    double w_com2 = 0.15;
    double w_com3 = 0.2;
    double w_base = 0.7;
    double w_total = 4 * w_com1 + 4 * w_com2 + 4 * w_com3 + w_base;
    Vector4d w;
    VectorXd leg1_com(12), leg2_com(12), leg3_com(12), leg4_com(12);
    Vector3d posb, rotb;
    posb << q.segment(0, 3);
    rotb << q.segment(15, 3);

    w << w_com1, w_com2, w_com3, w_total;

    // Find the center of mass
    com_kinematics(leg1_com, q.segment(3, 3), 1, posb, rotb, w);
    com_kinematics(leg2_com, q.segment(6, 3), 2, posb, rotb, w);
    com_kinematics(leg3_com, q.segment(9, 3), 3, posb, rotb, w);
    com_kinematics(leg4_com, q.segment(12, 3), 4, posb, rotb, w);

    // COM of the base
    Vector3d base = (w_base / w_total) * posb;

    // COM position
    com_pos << leg1_com.segment(0, 3) + leg1_com.segment(3, 3) + leg1_com.segment(6, 3) + leg2_com.segment(0, 3) + leg2_com.segment(3, 3) 
        + leg2_com.segment(6, 3) + leg3_com.segment(0, 3) + leg3_com.segment(3, 3) + leg3_com.segment(6, 3) + leg4_com.segment(0, 3) 
        + leg4_com.segment(3, 3) + leg4_com.segment(6, 3) + base;
}


void com_jacobian(MatrixXd& com_jacobian, const VectorXd& q, const Vector3d& current_pos) {
    // Analytic jacobian for leg position. Returns a 3x3 matrix with input q = [q1, q2, q3]
    VectorXd dq;
    Vector3d pos_dq;
    double delta = 0.00001;

    for (int i = 0; i <= 17; i++) {
        // Copy initial articular configuration and use delta increment on index i
        dq = q;
        dq(i) += delta;

        // Homegeneous Transformation Matrix after increment
        com_pos(pos_dq, dq);

        // Finite difference
        com_jacobian(0, i) = (pos_dq(0) - current_pos(0)) / delta;
        com_jacobian(1, i) = (pos_dq(1) - current_pos(1)) / delta;
        com_jacobian(2, i) = (pos_dq(2) - current_pos(2)) / delta;
    }
}
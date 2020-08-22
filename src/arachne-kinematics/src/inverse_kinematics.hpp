// inverse_kinematics.hpp
#pragma once

#include <qpOASES.hpp>
#include "forward_kinematics.hpp"
#include "com_control.hpp"

// Declare structs
struct Cost_values
{
    MatrixXd h;
    VectorXd f;
};

class Parameters {
public:
    // Maximum number of iterations
    int max_iter;
    // Gain of quadratic function
    int lamb;
    // Time between signals
    double d_t;
    // Maximum tolerance for minimization
    double tol;
    // Weights: [posb, leg1, leg2, leg3, leg4, rotb]
    VectorXi w;

    Parameters() {
        // Maximum number of iterations
        this->max_iter = 100;
        // Gain of quadratic function
        this->lamb = -10;
        // Time between signals
        this->d_t = 0.01;
        // Maximum tolerance for minimization
        this->tol = 0.01;
        // Weights: [posb, leg1, leg2, leg3, leg4, rotb]
        this->w = VectorXi::Ones(7);
        this->w(0) = 1;
        this->w(5) = 1;
        this->w(6) = 0;
    }
};

void analytic_jacobian(Matrix3d& jacobian, const Vector3d& q, const Vector3d& current_pos, const Vector3d& posb, const Vector3d& rotb, const int& leg);

class Legs {
public:
    Vector3d leg1, leg2, leg3, leg4, posb, rotb, com;
    Matrix3d leg1_jacobian, leg2_jacobian, leg3_jacobian, leg4_jacobian;

    void update_position(const VectorXd& q) {
        posb = q.segment(0, 3);
        rotb = q.segment(15, 3);
        forward_kinematics(leg1, q.segment(3, 3), posb, rotb, 1);
        forward_kinematics(leg2, q.segment(6, 3), posb, rotb, 2);
        forward_kinematics(leg3, q.segment(9, 3), posb, rotb, 3);
        forward_kinematics(leg4, q.segment(12, 3), posb, rotb, 4);
    }

    void update_jacobian(const VectorXd& q) {
        analytic_jacobian(leg1_jacobian, q.segment(3, 3), leg1, posb, rotb, 1);
        analytic_jacobian(leg2_jacobian, q.segment(6, 3), leg2, posb, rotb, 2);
        analytic_jacobian(leg3_jacobian, q.segment(9, 3), leg3, posb, rotb, 3);
        analytic_jacobian(leg4_jacobian, q.segment(12, 3), leg4, posb, rotb, 4);
    }
};

void analytic_jacobian(Matrix3d& jacobian, const Vector3d& q, const Vector3d& current_pos, const Vector3d& posb, const Vector3d& rotb, const int& leg) {
    // Analytic jacobian for leg position. Returns a 3x3 matrix with input q = [q1, q2, q3]
    Vector3d dq;
    Vector3d pos_dq;
    double delta = 0.00001;

    for (int i = 0; i <= 2; i++) {
        // Copy initial articular configuration and use delta increment on index i
        dq = q;
        dq(i) += delta;
        
        // Homegeneous Transformation Matrix after increment
        if (leg == 1) {
            forward_kinematics(pos_dq, dq, posb, rotb, 1);
        }
        else if (leg == 2) {
            forward_kinematics(pos_dq, dq, posb, rotb, 2);
        }
        else if (leg == 3) {
            forward_kinematics(pos_dq, dq, posb, rotb, 3);
        }
        else {
            forward_kinematics(pos_dq, dq, posb, rotb, 4);
        }

        // Finite difference
        jacobian(0, i) = (pos_dq(0) - current_pos(0)) / delta;
        jacobian(1, i) = (pos_dq(1) - current_pos(1)) / delta;
        jacobian(2, i) = (pos_dq(2) - current_pos(2)) / delta;
    }
}


void leg_jacobian(MatrixXd& leg_jacobian, const Vector3d& q, const int& leg, Legs& pos) {
    Matrix3d jacobian, m_skew;
    
    if (leg == 1) {
        // Distance vector of the end - effector with respect to base
        Vector3d d = pos.posb - pos.leg1;

        // Skew - Symmetric matrix
        m_skew << 0, -d(2), d(1),
                  d(2), 0, -d(0),
                  -d(1), d(0), 0;

        leg_jacobian << Matrix3d::Identity(), pos.leg1_jacobian, Matrix<double, 3, 9>::Identity(), m_skew;
    }
    else if (leg == 2) {
        // Distance vector of the end - effector with respect to base
        Vector3d d = pos.posb - pos.leg2;

        // Skew - Symmetric matrix
        m_skew << 0, -d(2), d(1),
            d(2), 0, -d(0),
            -d(1), d(0), 0;

        leg_jacobian << Matrix3d::Identity(), Matrix3d::Zero(), pos.leg2_jacobian, Matrix<double, 3, 6>::Identity(), m_skew;
    }
    else if (leg == 3) {
        // Distance vector of the end - effector with respect to base
        Vector3d d = pos.posb - pos.leg3;

        // Skew - Symmetric matrix
        m_skew << 0, -d(2), d(1),
            d(2), 0, -d(0),
            -d(1), d(0), 0;

        leg_jacobian << Matrix3d::Identity(), Matrix<double, 3, 6>::Identity(), pos.leg3_jacobian, Matrix3d::Zero(), m_skew;
    }
    else {
        // Distance vector of the end - effector with respect to base
        Vector3d d = pos.posb - pos.leg4;

        // Skew - Symmetric matrix
        m_skew << 0, -d(2), d(1),
            d(2), 0, -d(0),
            -d(1), d(0), 0;

        leg_jacobian << Matrix3d::Identity(), Matrix<double, 3, 9>::Identity(), pos.leg4_jacobian, m_skew;
    }
}

void calc_err(double& err, const VectorXd& q, const VectorXd& xd, Legs& pos) {
    // Find the error of each leg and the base
    Vector3d err_leg1 = xd.segment(3, 3) - pos.leg1;
    Vector3d err_leg2 = xd.segment(6, 3) - pos.leg2;
    Vector3d err_leg3 = xd.segment(9, 3) - pos.leg3;
    Vector3d err_leg4 = xd.segment(12, 3) - pos.leg4;
    Vector3d err_posb = xd.segment(0, 3) - pos.posb;
    Vector3d err_rotb = xd.segment(15, 3) - pos.rotb;

    // Sum of the squared errors
    err = sqrt(pow(err_leg1(0), 2) + pow(err_leg1(1), 2) + pow(err_leg1(2), 2) + pow(err_leg2(0), 2) + pow(err_leg2(1), 2) + pow(err_leg2(2), 2) 
        + pow(err_leg3(0), 2) + pow(err_leg3(1), 2) + pow(err_leg3(2), 2) + pow(err_leg4(0), 2) + pow(err_leg4(1), 2) + pow(err_leg4(2), 2) 
        + pow(err_posb(0), 2) + pow(err_posb(1), 2) + pow(err_posb(2), 2) + pow(err_rotb(0), 2) + pow(err_rotb(1), 2) + pow(err_rotb(2), 2));
}


void costfunc(Cost_values& cost, const VectorXd& q, const VectorXd& xd, const int& lamb, const VectorXi& w, const VectorXd& com_xd, Legs& pos) {
    // This function finds the values of h and f in order to initialize the quadratic program.
    // The inputs are q : actuated and sub - actuated angles, xd : desired position vector, p : weights and lamb : gain.

    // Position and orientation of the base
    MatrixXd j1(3, 18), j2(3, 18), j3(3, 18), j4(3, 18), j5(3, 18), j6(3, 18), j_com(3, 18);
    
    // Jacobian of each leg 
    leg_jacobian(j1, q.segment(3, 3), 1, pos);
    leg_jacobian(j2, q.segment(6, 3), 2, pos);
    leg_jacobian(j3, q.segment(9, 3), 3, pos);
    leg_jacobian(j4, q.segment(12, 3), 4, pos);

    // Jacobians of the base position and orientation
    j5 << Matrix3d::Identity(), Matrix<double, 3, 15>::Zero();
    j6 << Matrix<double, 3, 15>::Zero(), Matrix3d::Identity();

    // Position and jacobian of center of mass
    com_pos(pos.com, q);
    //com_jacobian(j_com, q, pos.com);

    // Values of h and f(Hessian and vector of linear elements) :
    //MatrixXd h = w(0) * j1.transpose()*j1 + w(1) * j2.transpose()*j2 + w(2) * j3.transpose()*j3 + w(3) * j4.transpose()*j4 + w(4) * j5.transpose()*j5 + w(5) * j6.transpose()*j6 + w(6) * j_com.transpose()*j_com;
    MatrixXd h = w(0) * j1.transpose()*j1 + w(1) * j2.transpose()*j2 + w(2) * j3.transpose()*j3 + w(3) * j4.transpose()*j4 + w(4) * j5.transpose()*j5 + w(5) * j6.transpose()*j6;

    //VectorXd f = -2 * (w(1) * lamb * (pos.leg1 - xd.segment(3, 3)).transpose()*j1 + w(2) * lamb * (pos.leg2 - xd.segment(6, 3)).transpose()*j2 + w(3) * lamb * (pos.leg3 - xd.segment(9, 3)).transpose()*j3 + w(4) * lamb * (pos.leg4 - xd.segment(12, 3)).transpose()*j4 + w(0) * lamb * (pos.posb - xd.segment(0, 3)).transpose()*j5 + w(5) * lamb * (pos.rotb - xd.segment(15, 3)).transpose()*j6 + w(6) * lamb * (pos.com - com_xd).transpose()*j_com);
    VectorXd f = -2 * (w(1) * lamb * (pos.leg1 - xd.segment(3, 3)).transpose()*j1 + w(2) * lamb * (pos.leg2 - xd.segment(6, 3)).transpose()*j2 + w(3) * lamb * (pos.leg3 - xd.segment(9, 3)).transpose()*j3 + w(4) * lamb * (pos.leg4 - xd.segment(12, 3)).transpose()*j4 + w(0) * lamb * (pos.posb - xd.segment(0, 3)).transpose()*j5 + w(5) * lamb * (pos.rotb - xd.segment(15, 3)).transpose()*j6);

    cost = { h, f };
}


void quad_prog(MatrixXd& qf, VectorXd& q, const VectorXd& xd, const Parameters& parameters, const Vector3d& com_xd, Legs& pos) {
    // This function manages the minimization program and find the error of the desired function
    
    USING_NAMESPACE_QPOASES
    
    int itr = 0;
    int j = 0;
    double err;
    double arr_h[324];
    double arr_f[18];
    Vector3d rot_vec;
    VectorXd dq(18);
    Matrix3d rot_axis, skw, rgs, rot;
    Cost_values cost;

    // Setting up QProblemB object
    QProblemB qp_quad(18);

    Options options;
    options.initialStatusBounds = ST_INACTIVE;
    options.numRefinementSteps = 1;
    options.enableCholeskyRefactorisation = 1;
    options.printLevel = PL_NONE;
    qp_quad.setOptions(options);

    while (itr < parameters.max_iter) {
        costfunc(cost, q, xd, parameters.lamb, parameters.w, com_xd, pos);
        Map<MatrixXd>(&arr_h[0], 18, 18) = cost.h;
        Map<VectorXd>(&arr_f[0], 18, 1) = cost.f;

        // Setup data of first QP
        real_t H[18 * 18];
        for (int i = 0; i < 324; ++i) {
            H[i] = arr_h[i];
        }
        real_t g[18];
        for (int i = 0; i < 18; ++i) {
            g[i] = arr_f[i];
        }
        real_t lb[18];
        for (int i = 0; i < 18; ++i) {
            lb[i] = -1e10;
        }
        real_t ub[18];
        for (int i = 0; i < 18; ++i) {
            ub[i] = 1e10;
        }

        // Solve QP
        int_t nWSR = 10;
        qp_quad.init(H, g, lb, ub, nWSR, 0);
        real_t xOpt[18];

        qp_quad.getPrimalSolution(xOpt);
        for (int i = 0; i < 18; ++i) {
            dq[i] = xOpt[i];
        }

        // Update the position vector
        q.segment(0, 15) = q.segment(0, 15) + parameters.d_t * dq.segment(0, 15);

        // Update the orientation vector
        rot_axis << cos(q(16)) * cos(q(17)), -sin(q(17)), 0,
                    cos(q(16)) * sin(q(17)),  cos(q(17)), 0,
                                -sin(q(16)),           0, 1;

        rot_vec << rot_axis * dq.segment(15, 3);
        
        skw <<           0, -rot_vec(2),  rot_vec(1),
                rot_vec(2),           0, -rot_vec(0),
               -rot_vec(1),  rot_vec(0),           0;

        // Rodrigues rotation formula
        rgs = Matrix3d::Identity() + sin(parameters.d_t) * skw + (1 - cos(parameters.d_t)) * (skw*skw);
        q2rot(rot, q(15), q(16), q(17));
        rot *= rgs;
        rot2q(rot_vec, rot);
        q.segment(15, 3) = rot_vec;

        for (int i = 0; i < 18; ++i) {
            qf(j, i) = q(i);
        }

        calc_err(err, q, xd, pos);
        if (err <= parameters.tol) {
            itr++;
            break;
        }

        pos.update_position(q);
        pos.update_jacobian(q);
        itr++;
        j++;
    }
}
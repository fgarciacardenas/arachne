// inverse_kinematics.hpp
#pragma once

#include <qpOASES.hpp>
#include "forward_kinematics.hpp"
//#include "com_control.hpp"

// Declare structs
struct Cost_values {
    MatrixXd h;
    VectorXd f;
};

struct Parameters {
    int max_iterations, lambda;
    double dt, tolerance;
    VectorXi w;

    Parameters() {
        // Maximum number of iterations
        this->max_iterations = 100;
        // Gain of quadratic function
        this->lambda = -10;
        // Time between signals
        this->dt = 0.01;
        // Maximum tolerance for minimization
        this->tolerance = 0.01;
        // Weights: [posb, rotb, leg1, leg2, leg3, leg4]
        this->w = VectorXi::Ones(7);
        this->w(0) = 1; // Base position
        this->w(1) = 1; // Base orientation
        this->w(6) = 0;
    }
};

void analytic_jacobian(Matrix3d& jacobian, const Vector3d& q0, const Vector3d& current_pos, const VectorXd& base, const int& leg);

class Legs {
public:
    Vector3d leg1, leg2, leg3, leg4, com;
    VectorXd base;
    Matrix3d leg1_jacobian, leg2_jacobian, leg3_jacobian, leg4_jacobian;

    void update_position(const VectorXd& q) {
        base = q.segment(0, 6);
        forward_kinematics(leg1, q.segment( 6, 3), base, 1);
        forward_kinematics(leg2, q.segment( 9, 3), base, 2);
        forward_kinematics(leg3, q.segment(12, 3), base, 3);
        forward_kinematics(leg4, q.segment(15, 3), base, 4);
    }

    void update_jacobian(const VectorXd& q) {
        analytic_jacobian(leg1_jacobian, q.segment( 6, 3), leg1, base, 1);
        analytic_jacobian(leg2_jacobian, q.segment( 9, 3), leg2, base, 2);
        analytic_jacobian(leg3_jacobian, q.segment(12, 3), leg3, base, 3);
        analytic_jacobian(leg4_jacobian, q.segment(15, 3), leg4, base, 4);
    }
};

void analytic_jacobian(Matrix3d& jacobian, const Vector3d& q0, const Vector3d& current_pos, const VectorXd& base, const int& leg) {
    // Analytic jacobian of the leg system. Returns a 3x3 matrix with input q = [q1, q2, q3].
    Vector3d q, dq_pos;
    double dt = 0.00001;

    for (int i = 0; i <= 2; ++i) {
        // Copy initial joint configuration and use delta increment on index i
        q = q0;
        q(i) += dt;
        
        // Homegeneous Transformation Matrix after increment
        forward_kinematics(dq_pos, q, base, leg);

        // Finite difference
        jacobian(0, i) = (dq_pos(0) - current_pos(0)) / dt;
        jacobian(1, i) = (dq_pos(1) - current_pos(1)) / dt;
        jacobian(2, i) = (dq_pos(2) - current_pos(2)) / dt;
    }
}

double squared_error(const VectorXd& xd, Legs& pos) {
    // Compute the error of each leg and the base
    VectorXd base_err = xd.segment( 0, 6) - pos.base;
    Vector3d leg1_err = xd.segment( 6, 3) - pos.leg1;
    Vector3d leg2_err = xd.segment( 9, 3) - pos.leg2;
    Vector3d leg3_err = xd.segment(12, 3) - pos.leg3;
    Vector3d leg4_err = xd.segment(15, 3) - pos.leg4;

    // Sum of the squared errors
    return sqrt(pow(base_err(0), 2) + pow(base_err(1), 2) + pow(base_err(2), 2)
           +    pow(base_err(3), 2) + pow(base_err(4), 2) + pow(base_err(5), 2)
           +    pow(leg1_err(0), 2) + pow(leg1_err(1), 2) + pow(leg1_err(2), 2)
           +    pow(leg2_err(0), 2) + pow(leg2_err(1), 2) + pow(leg2_err(2), 2) 
           +    pow(leg3_err(0), 2) + pow(leg3_err(1), 2) + pow(leg3_err(2), 2) 
           +    pow(leg4_err(0), 2) + pow(leg4_err(1), 2) + pow(leg4_err(2), 2));
}


void cost_function(Cost_values& cost, const VectorXd& q, const VectorXd& xd, const int& lambda, const VectorXi& w, const VectorXd& com_xd, Legs& pos) {
    // This function computes the values of h and f to initialize the quadratic program.
    // The inputs are q : actuated and sub-actuated joints, xd : desired position vector, p : weights and lamb : gain.

    // Extended Jacobian representation to compute the floating base of the quadruped robot
    MatrixXd extended_leg1(3, 18), extended_leg2(3, 18), extended_leg3(3, 18), extended_leg4(3, 18);
    MatrixXd extended_posb(3, 18), extended_rotb(3, 18), extended_com(3, 18);
    
    // Extended Jacobian of each leg and the floating base
    Vector3d dist = pos.base.segment(0,3) - pos.leg1;
    extended_leg1 << Matrix3d::Identity(), pos.leg1_jacobian, Matrix<double, 3, 9>::Identity(), skew(dist);
    
    dist = pos.base.segment(0,3) - pos.leg2;
    extended_leg2 << Matrix3d::Identity(), Matrix3d::Zero(), pos.leg2_jacobian, Matrix<double, 3, 6>::Identity(), skew(dist);
    
    dist = pos.base.segment(0,3) - pos.leg3;
    extended_leg3 << Matrix3d::Identity(), Matrix<double, 3, 6>::Identity(), pos.leg3_jacobian, Matrix3d::Zero(), skew(dist);
    
    dist = pos.base.segment(0,3) - pos.leg4;
    extended_leg4 << Matrix3d::Identity(), Matrix<double, 3, 9>::Identity(), pos.leg4_jacobian, skew(dist);
    
    extended_posb << Matrix3d::Identity(), Matrix<double, 3, 15>::Zero();
    extended_rotb << Matrix<double, 3, 15>::Zero(), Matrix3d::Identity();

    // Position and jacobian of center of mass
    //com_pos(pos.com, q);
    //com_jacobian(j_com, q, pos.com);

    // Values of h and f (Hessian and vector of linear elements) :
    MatrixXd h = w(0) * extended_posb.transpose()*extended_posb + w(1) * extended_rotb.transpose()*extended_rotb
               + w(2) * extended_leg1.transpose()*extended_leg1 + w(3) * extended_leg2.transpose()*extended_leg2
               + w(4) * extended_leg3.transpose()*extended_leg3 + w(5) * extended_leg4.transpose()*extended_leg4;
            // + w(6) * j_com.transpose()*j_com

    VectorXd f = -2 * (w(0) * lambda * (pos.base.segment(0, 3) - xd.segment( 0, 3)).transpose()*extended_posb 
                     + w(1) * lambda * (pos.base.segment(3, 3) - xd.segment(15, 3)).transpose()*extended_rotb
                     + w(2) * lambda * (pos.leg1 - xd.segment( 3, 3)).transpose()*extended_leg1
                     + w(3) * lambda * (pos.leg2 - xd.segment( 6, 3)).transpose()*extended_leg2
                     + w(4) * lambda * (pos.leg3 - xd.segment( 9, 3)).transpose()*extended_leg3
                     + w(5) * lambda * (pos.leg4 - xd.segment(12, 3)).transpose()*extended_leg4);
                  // + w(6) * lamb * (pos.com - com_xd).transpose()*j_com)

    cost = { h, f };
}


void quadratic_program(MatrixXd& qf, VectorXd& q, const VectorXd& xd, const Parameters& parameters, const Vector3d& com_xd, Legs& pos) {
    // This function manages the quadratic minimization program and computes the error of the desired function
    USING_NAMESPACE_QPOASES
    
    int iteration = 0;
    double arr_h[324], arr_f[18];
    Vector3d rotation_vector;
    VectorXd dq(18);
    Matrix3d rotation_axis, skw, rodrigues_matrix, rotation_matrix;
    Cost_values cost;

    // Setting up QProblemB object
    QProblemB qp_quad(18);
    Options options;
    options.initialStatusBounds = ST_INACTIVE;
    options.numRefinementSteps = 1;
    options.enableCholeskyRefactorisation = 1;
    options.printLevel = PL_NONE;
    qp_quad.setOptions(options);

    while (iteration < parameters.max_iterations) {
        // Update state of the robot
        //pos.update_position(q);
        //pos.update_jacobian(q);

        // Compute the cost function of the Quadratic Program (QP)
        cost_function(cost, q, xd, parameters.lambda, parameters.w, com_xd, pos);
        
        // Convert from Eigen matrix to std::array
        Map<MatrixXd>(&arr_h[0], 18, 18) = cost.h;
        Map<VectorXd>(&arr_f[0], 18, 1) = cost.f;

        // Setup data of first QP: Hessian, linear, lower bound, upper bound
        real_t H[18 * 18], g[18], lb[18], ub[18]; 
        for (int i = 0; i < 324; ++i)
            H[i] = arr_h[i];
        
        for (int i = 0; i < 18; ++i)
            g[i] = arr_f[i];
        
        for (int i = 0; i < 18; ++i)
            lb[i] = -1e10;

        for (int i = 0; i < 18; ++i)
            ub[i] = 1e10;

        // Solve QP
        int_t nWSR = 10;
        qp_quad.init(H, g, lb, ub, nWSR, 0);
        real_t xOpt[18];

        // Get QP solution
        qp_quad.getPrimalSolution(xOpt);
        for (int i = 0; i < 18; ++i)
            dq[i] = xOpt[i];

        // Update the position vector
        q.segment(0, 3) = q.segment(0, 3) + parameters.dt * dq.segment(0, 3);
        q.segment(6, 12) = q.segment(6, 12) + parameters.dt * dq.segment(6, 12);

        // Update the orientation vector
        rotation_axis << cos(q(4)) * cos(q(5)), -sin(q(5)), 0,
                    cos(q(4)) * sin(q(5)),  cos(q(5)), 0,
                               -sin(q(4)),          0, 1;

        // Vectorization
        rotation_vector << rotation_axis * dq.segment(3, 3);
        skw = skew(rotation_vector);

        // Rodrigues rotation formula
        rodrigues_matrix = Matrix3d::Identity() + sin(parameters.dt) * skw + (1 - cos(parameters.dt)) * (skw*skw);
        q2rot(rotation_matrix, q(3), q(4), q(5));
        
        // Rotation increment
        rotation_matrix *= rodrigues_matrix;
        
        // Vectorization
        rot2q(rotation_vector, rotation_matrix);
        q.segment(3, 3) = rotation_vector;

        // Store each iteration of the quadratic program
        for (int i = 0; i < 18; ++i)
            qf(iteration, i) = q(i);

        //std::cout << "Desired pos: \n" << xd << std::endl;
        //std::cout << "Current pos: \n" << pos.leg1 << std::endl;
        //std::cout << "Error: \n" << squared_error(xd, pos) << std::endl;
        //std::cout << "Q-vector: \n" << q << std::endl;

        // Exit the program if the error is lower than the tolerance
        if (squared_error(xd, pos) <= parameters.tolerance) {
            ++iteration;
            break;
        }

        pos.update_position(q);
        pos.update_jacobian(q);
        ++iteration;
    }
}
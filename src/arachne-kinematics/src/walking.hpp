// walking.hpp
#pragma once

#include "forward_kinematics.hpp"

void walking(MatrixXd& com_jacobian, const VectorXd& q, const Vector3d& current_pos, const Legs& Leg) {
    MatrixXd leg1, leg2, leg3, leg4, cog1, cog2;
    
    // Ingresamos el paso
    double x_dist, y_dist, steps;
    std::cout << "\nIngrese el desplazamiento en x:\n";
    std::cin >> x_dist;
    std::cout << "\nIngrese el desplazamiento en y:\n";
    std::cin >> y_dist;
    std::cout << "\nIngrese la secuencia:\n";
    std::cin >> steps;

    VectorXd desired_pos(10);

    // Increment leg1 position
    desired_pos(0) = Leg.leg1(0) + x_dist;
    desired_pos(1) = Leg.leg1(1) + y_dist;

    // Update center of gravity
    desired_pos(8) = Leg.com(0) + x_dist / 2;
    desired_pos(9) = Leg.com(1) + y_dist / 2;
        
    // Increment leg2 position
    desired_pos(2) = Leg.leg2(0) + x_dist;
    desired_pos(3) = Leg.leg2(1) + y_dist;

    // Increment leg4 position
    desired_pos(6) = Leg.leg4(0) + x_dist;
    desired_pos(7) = Leg.leg4(1) + y_dist;

    // Update center of gravity
    desired_pos(8) = Leg.com(0) + x_dist / 2;
    desired_pos(9) = Leg.com(1) + y_dist / 2;

    // Increment leg3 position
    desired_pos(4) = Leg.leg3(0) + x_dist;
    desired_pos(5) = Leg.leg3(1) + y_dist;
}


void gait_step(double& modulo, double& residuo, double& num, double& distance) {
    if (distance >= 10) {
        modulo = 10;
        double r = fmod(distance, modulo);

        if (r >= 1) {
            residuo = r;
        }
        else {
            residuo = 0;
        }
        
        num = floor((distance - residuo)/modulo);
    }
    if (distance >= 1 && distance < 10) {
        modulo = 1;
        double r = fmod(distance, modulo);
        
        if (r >= 1) {
            residuo = r;
        }
        else {
            residuo = 0;
        }
    
        num = floor((distance-residuo)/modulo);
    }
    if (distance < 1) {
        modulo = 0;
        residuo = 0;
        num = 0;
    }
}


void walking(VectorXd& posd, Legs& pos, MatrixXd& final, int& ind) {
    // Initialize variables
    int quadrant;
    double angle, distance;
    double modulo, residuo, num, resix, resiy;
    VectorXd M(18);
    
    // Find the direction vector
    angle = atan2(posd(3) - posd(1), posd(2) - posd(0));
    distance = sqrt(pow(posd(2) - posd(0),2) + pow(posd(3) - posd(1),2));

    // Number of steps required
    gait_step(modulo, residuo, num, distance);
    
    // Desplazamiento en funcion del angulo hallado
    double parax = modulo * cos(angle);
    double paray = modulo * sin(angle);

    // Center legs
    Vector3d cl1, cl2, cl3, cl4;
    VectorXd qc(12);
    center(qc);

    forward_kinematics(cl1, qc.segment(0, 3), pos.posb, pos.rotb, 1);
    forward_kinematics(cl2, qc.segment(3, 3), pos.posb, pos.rotb, 2);
    forward_kinematics(cl3, qc.segment(6, 3), pos.posb, pos.rotb, 3);
    forward_kinematics(cl4, qc.segment(9, 3), pos.posb, pos.rotb, 4);

    M.segment(0, 3)  << pos.posb(0), pos.posb(1), pos.posb(2);
    M.segment(3, 3)  << cl1(0), cl1(1), cl1(2);
    M.segment(6, 3)  << cl2(0), cl2(1), cl2(2);
    M.segment(9, 3)  << cl3(0), cl3(1), cl3(2);
    M.segment(12, 3) << cl4(0), cl4(1), cl4(2);
    M.segment(15, 3) << pos.rotb(0), pos.rotb(1), pos.rotb(2);
    final.col(0) = M;
    
    // Select working quadrant
    if (angle >= 0 && angle < M_PI / 2)
        quadrant = 1;
    else if (angle >= M_PI / 2 && angle <= M_PI)
        quadrant = 2;
    else if (angle >= - M_PI && angle < - M_PI / 2)
        quadrant = 3;
    else if (angle >= - M_PI / 2 && angle < 0)
        quadrant = 4;

    // Se selecciona el cuadrante sobre el cual se trabaja
    switch(quadrant) {
        case 1:
            // Levanta pata 2 y la mueve a la posicion deseada
            M(8) = -20;
            final.col(1) = M;
            M(6) = M(6) + parax/2;
            M(7) = M(7) + paray/2;
            final.col(2) = M;
            M(8) = -22.5;
            final.col(3) = M;

            // MUEVE EL CENTRO DE MASAS
            M(0) = M(0) + parax/4;
            M(1) = M(1) + paray/4;
            final.col(4) = M;

            // LEVANTA Y MUEVE LA PATA 3
            M(11) = -20;
            final.col(5) = M;
            M(9) = M(9) + parax/2;
            M(10) = M(10) + paray/2;
            final.col(6) = M;
            M(11) = -22.5;
            final.col(7) = M;

            ind = 8; // Indice de la matriz IMPORTANTE
            
            for (int n=0; n < num; n++) {
                // LEVANTA Y MUEVE LA PATA 1
                M(5) = -20;
                final.col(ind) = M;
                M(3) = M(3) + parax;
                M(4) = M(4) + paray;
                final.col(ind+1) = M;
                M(5) = -22.5;
                final.col(ind+2) = M;
                
                // MUEVE EL CENTRO DE MASAS
                M(0) = M(0) + parax/2;
                M(1) = M(1) + paray/2;
                final.col(ind+3) = M;
                
                // LEVANTA Y MUEVE LA PATA 4
                M(14) = -20;
                final.col(ind+4) = M;
                M(12) = M(12) + parax;
                M(13) = M(13) + paray;
                final.col(ind+5) = M;
                M(14) = -22.5;
                final.col(ind+6) = M;
                
                // LEVANTA Y MUEVE LA PATA 2
                M(8) = -20;
                final.col(ind+7) = M;
                M(6) = M(6) + parax;
                M(7) = M(7) + paray;
                final.col(ind+8) = M;
                M(8) = -22.5;
                final.col(ind+9) = M;
                
                // MUEVE EL CENTRO DE MASAS
                M(0) = M(0) + parax/2;
                M(1) = M(1) + paray/2;
                final.col(ind+10) = M;

                // LEVANTA Y MUEVE LA PATA 3
                M(11) = -20;
                final.col(ind+11) = M;
                M(9) = M(9) + parax;
                M(10) = M(10) + paray;
                final.col(ind+12) = M;
                M(11) = -22.5;
                final.col(ind+13) = M;
                  
                ind += 14;
            }
            break;

        case 2:
            // Levanta pata 4 y la mueve a la posicion deseada
            M(14) = -20;
            final.col(1) = M;
            M(12) = M(12) + parax/2;
            M(13) = M(13) + paray/2;
            final.col(2) = M;
            M(14) = -22.5;
            final.col(3) = M;

            // MUEVE EL CENTRO DE MASAS
            M(0) = M(0) + parax/4;
            M(1) = M(1) + paray/4;
            final.col(4) = M;

            // LEVANTA Y MUEVE LA PATA 1
            M(5) = -20;
            final.col(5) = M;
            M(3) = M(3) + parax/2;
            M(4) = M(4) + paray/2;
            final.col(6) = M;
            M(5) = -22.5;
            final.col(7) = M;

            ind = 8; // Indice de la matriz IMPORTANTE
            
            for (int n=0; n < num; n++) {
                // LEVANTA Y MUEVE LA PATA 3
                M(11) = -20;
                final.col(ind) = M;
                M(9) = M(9) + parax;
                M(10) = M(10) + paray;
                final.col(ind+1) = M;
                M(11) = -22.5;
                final.col(ind+2) = M;
                
                // MUEVE EL CENTRO DE MASAS
                M(0) = M(0) + parax/2;
                M(1) = M(1) + paray/2;
                final.col(ind+3) = M;
                
                // LEVANTA Y MUEVE LA PATA 2
                M(8) = -20;
                final.col(ind+4) = M;
                M(6) = M(6) + parax;
                M(7) = M(7) + paray;
                final.col(ind+5) = M;
                M(8) = -22.5;
                final.col(ind+6) = M;
                
                // LEVANTA Y MUEVE LA PATA 4
                M(14) = -20;
                final.col(ind+7) = M;
                M(12) = M(12) + parax;
                M(13) = M(13) + paray;
                final.col(ind+8) = M;
                M(14) = -22.5;
                final.col(ind+9) = M;
                
                // MUEVE EL CENTRO DE MASAS
                M(0) = M(0) + parax/2;
                M(1) = M(1) + paray/2;
                final.col(ind+10) = M;

                // LEVANTA Y MUEVE LA PATA 1
                M(5) = -20;
                final.col(ind+11) = M;
                M(3) = M(3) + parax;
                M(4) = M(4) + paray;
                final.col(ind+12) = M;
                M(5) = -22.5;
                final.col(ind+13) = M;
                  
                ind += 14;
            }
            break;

        case 3:
            // Levanta pata 3 y la mueve a la posicion deseada
            M(11) = -20;
            final.col(1) = M;
            M(9) = M(9) + parax/2;
            M(10) = M(10) + paray/2;
            final.col(2) = M;
            M(11) = -22.5;
            final.col(3) = M;

            // MUEVE EL CENTRO DE MASAS
            M(0) = M(0) + parax/4;
            M(1) = M(1) + paray/4;
            final.col(4) = M;

            // LEVANTA Y MUEVE LA PATA 2
            M(8) = -20;
            final.col(5) = M;
            M(6) = M(6) + parax/2;
            M(7) = M(7) + paray/2;
            final.col(6) = M;
            M(8) = -22.5;
            final.col(7) = M;

            ind = 8; // Indice de la matriz IMPORTANTE
            
            for (int n=0; n < num; n++) {
                // LEVANTA Y MUEVE LA PATA 4
                M(14) = -20;
                final.col(ind) = M;
                M(12) = M(12) + parax;
                M(13) = M(13) + paray;
                final.col(ind+1) = M;
                M(14) = -22.5;
                final.col(ind+2) = M;
                
                // MUEVE EL CENTRO DE MASAS
                M(0) = M(0) + parax/2;
                M(1) = M(1) + paray/2;
                final.col(ind+3) = M;
                
                // LEVANTA Y MUEVE LA PATA 1
                M(5) = -20;
                final.col(ind+4) = M;
                M(3) = M(3) + parax;
                M(4) = M(4) + paray;
                final.col(ind+5) = M;
                M(5) = -22.5;
                final.col(ind+6) = M;
                
                // LEVANTA Y MUEVE LA PATA 3
                M(11) = -20;
                final.col(ind+7) = M;
                M(9) = M(9) + parax;
                M(10) = M(10) + paray;
                final.col(ind+8) = M;
                M(11) = -22.5;
                final.col(ind+9) = M;
                
                // MUEVE EL CENTRO DE MASAS
                M(0) = M(0) + parax/2;
                M(1) = M(1) + paray/2;
                final.col(ind+10) = M;

                // LEVANTA Y MUEVE LA PATA 2
                M(8) = -20;
                final.col(ind+11) = M;
                M(6) = M(6) + parax;
                M(7) = M(7) + paray;
                final.col(ind+12) = M;
                M(8) = -22.5;
                final.col(ind+13) = M;
                  
                ind += 14;
            }
            break;
             
        case 4:
            // Levanta pata 1 y la mueve a la posicion deseada
            M(5) = -20;
            final.col(1) = M;
            M(3) = M(3) + parax/2;
            M(4) = M(4) + paray/2;
            final.col(2) = M;
            M(5) = -22.5;
            final.col(3) = M;

            // MUEVE EL CENTRO DE MASAS
            M(0) = M(0) + parax/4;
            M(1) = M(1) + paray/4;
            final.col(4) = M;

            // LEVANTA Y MUEVE LA PATA 4
            M(14) = -20;
            final.col(5) = M;
            M(12) = M(12) + parax/2;
            M(13) = M(13) + paray/2;
            final.col(6) = M;
            M(14) = -22.5;
            final.col(7) = M;

            ind = 8; // Indice de la matriz IMPORTANTE
            
            for (int n=0; n < num; n++) {
                // LEVANTA Y MUEVE LA PATA 3
                M(11) = -20;
                final.col(ind) = M;
                M(9) = M(9) + parax;
                M(10) = M(10) + paray;
                final.col(ind+1) = M;
                M(11) = -22.5;
                final.col(ind+2) = M;
                
                // MUEVE EL CENTRO DE MASAS
                M(0) = M(0) + parax/2;
                M(1) = M(1) + paray/2;
                final.col(ind+3) = M;
                
                // LEVANTA Y MUEVE LA PATA 2
                M(8) = -20;
                final.col(ind+4) = M;
                M(6) = M(6) + parax;
                M(7) = M(7) + paray;
                final.col(ind+5) = M;
                M(8) = -22.5;
                final.col(ind+6) = M;
                
                // LEVANTA Y MUEVE LA PATA 1
                M(5) = -20;
                final.col(ind+7) = M;
                M(3) = M(3) + parax;
                M(4) = M(4) + paray;
                final.col(ind+8) = M;
                M(5) = -22.5;
                final.col(ind+9) = M;
                
                // MUEVE EL CENTRO DE MASAS
                M(0) = M(0) + parax/2;
                M(1) = M(1) + paray/2;
                final.col(ind+10) = M;

                // LEVANTA Y MUEVE LA PATA 4
                M(14) = -20;
                final.col(ind+11) = M;
                M(12) = M(12) + parax;
                M(13) = M(13) + paray;
                final.col(ind+12) = M;
                M(14) = -22.5;
                final.col(ind+13) = M;
                
                ind += 14;
            }
            break;
    }
}
// printConfig.hpp
#pragma once

#include <iostream>
#include <fstream>

void print_position(MatrixXd& qf, std::ofstream& myfile, const Parameters& parameters) {
    Legs posprint;
    int i = 0;
    while (qf(i,4) != 0) {
        posprint.update_position(qf.row(i));
        for (int j = 0; j < 3; j++) {
            myfile << posprint.posb(j) << ',';
        }
        for (int j = 0; j < 3; j++) {
            myfile << posprint.leg1(j) << ',';  
        }
        for (int j = 0; j < 3; j++) {
            myfile << posprint.leg2(j) << ',';
        }
        for (int j = 0; j < 3; j++) {
            myfile << posprint.leg3(j) << ',';
        }
        for (int j = 0; j < 3; j++) {
            myfile << posprint.leg4(j) << ',';
        }
        for (int j = 0; j < 3; j++) {
            myfile << posprint.rotb(j);
            if (j < 2) {
                myfile << ',';  
            } 
        }
        myfile << '\n';
        i++;
    }
}


void print_configuration(MatrixXd& qf, std::ofstream& myfile, const Parameters& parameters) {
    int i = 0;
    while (qf(i,4) != 0) {
        for (int j = 0; j < 18; j++) {
            myfile << qf(i,j);
            if (j < 17) {
                myfile << ',';  
            } 
        }
        myfile << '\n';
        i++;
    }
}


void compare(const VectorXd& xd, const Vector3d& com_xd, const Legs& pos) {
    IOFormat CommaInitFmt(StreamPrecision, DontAlignCols, ", ", ", ", "", "", " ", ";");
    std::cout << "Desired pos:" << xd.segment(3, 3).format(CommaInitFmt) << "\n" << "            " << xd.segment(6, 3).format(CommaInitFmt) << std::endl;
    std::cout << "            " << xd.segment(9, 3).format(CommaInitFmt) << "\n" << "            " << xd.segment(12, 3).format(CommaInitFmt) << std::endl;
    std::cout << "Final pos  :" << pos.leg1.format(CommaInitFmt) << "\n" << "            " << pos.leg2.format(CommaInitFmt) << std::endl;
    std::cout << "            " << pos.leg3.format(CommaInitFmt) << "\n" << "            " << pos.leg4.format(CommaInitFmt) << std::endl;
    std::cout << "Body pos   :" << xd.segment(0, 3).format(CommaInitFmt) << "\n" << "            " << pos.posb.format(CommaInitFmt) << std::endl;
    std::cout << "Body rot   :" << xd.segment(15, 3).format(CommaInitFmt) << "\n" << "            " << pos.rotb.format(CommaInitFmt) << std::endl;
    std::cout << "COM pos    :" << pos.com.format(CommaInitFmt) << "\n" << "            " << com_xd.format(CommaInitFmt) << std::endl << std::endl;
}
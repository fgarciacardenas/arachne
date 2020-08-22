// control.hpp
#pragma once

void initialize(VectorXd& q) {
    // q : Initial configuration vector[posb, leg1, leg2, leg3, leg4, rotb]
    q.segment( 0, 3) << 0, 0, 0;
    q.segment( 3, 3) << 90 * pi / 180, 90 * pi / 180, 90 * pi / 180;
    q.segment( 6, 3) << 90 * pi / 180, 90 * pi / 180, 90 * pi / 180;
    q.segment( 9, 3) << 90 * pi / 180, 90 * pi / 180, 90 * pi / 180;
    q.segment(12, 3) << 90 * pi / 180, 90 * pi / 180, 90 * pi / 180;
    q.segment(15, 3) << 0 * pi / 180, 0 * pi / 180, 0 * pi / 180;
}


void center(VectorXd& q) {
    q.segment( 3, 3) << 90 * pi / 180, 90 * pi / 180, 90 * pi / 180;
    q.segment( 6, 3) << 90 * pi / 180, 90 * pi / 180, 90 * pi / 180;
    q.segment( 9, 3) << 90 * pi / 180, 90 * pi / 180, 90 * pi / 180;
    q.segment(12, 3) << 90 * pi / 180, 90 * pi / 180, 90 * pi / 180;
}
#include <iostream>
#include "forward_kinematics.hpp"

using namespace std;

int main() {
    VectorXd q = VectorXd::Zero(18);
    Vector3d leg1, leg2, leg3, leg4, posb, rotb;

    forward_kinematics(leg1, q.segment( 6, 3), q.segment(0, 6), 1);
    forward_kinematics(leg2, q.segment( 9, 3), q.segment(0, 6), 2);
    forward_kinematics(leg3, q.segment(12, 3), q.segment(0, 6), 3);
    forward_kinematics(leg4, q.segment(15, 3), q.segment(0, 6), 4);

    cout << "Leg 1 pos:" << leg1(0) << ", " << leg1(1) << ", " << leg1(2) << endl;
    cout << "Leg 2 pos:" << leg2(0) << ", " << leg2(1) << ", " << leg2(2) << endl;
    cout << "Leg 3 pos:" << leg3(0) << ", " << leg3(1) << ", " << leg3(2) << endl;
    cout << "Leg 4 pos:" << leg4(0) << ", " << leg4(1) << ", " << leg4(2) << endl;

    VectorXd xd(18);
    xd.segment(0, 3)  << 0, 0, 0;
    xd.segment(3, 3)  << pos.leg1(0), pos.leg1(1), pos.leg1(2) + 2;
    xd.segment(6, 3)  << pos.leg2(0), pos.leg2(1), pos.leg2(2) + 2;
    xd.segment(9, 3)  << pos.leg3(0), pos.leg3(1), pos.leg3(2) + 2;
    xd.segment(12, 3) << pos.leg4(0), pos.leg4(1), pos.leg4(2) + 2;
    xd.segment(15, 3) << 0 * pi / 180, 0 * pi / 180, 0 * pi / 180;

    return 0;
}
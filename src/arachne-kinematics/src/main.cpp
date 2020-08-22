#include <iostream>
#include "forward_kinematics.h"

using namespace std;

int main() {
    VectorXd q = VectorXd::Zero(18);
    Vector3d leg1, leg2, leg3, leg4, posb, rotb;

    posb = q.segment(0, 3);
    rotb = q.segment(15, 3);
    forward_kinematics(leg1, q.segment(3, 3), posb, rotb, 1);
    forward_kinematics(leg2, q.segment(6, 3), posb, rotb, 2);
    forward_kinematics(leg3, q.segment(9, 3), posb, rotb, 3);
    forward_kinematics(leg4, q.segment(12, 3), posb, rotb, 4);

    cout << "Leg 1 pos:" << leg1(0) << ", " << leg1(1) << ", " << leg1(2) << endl;
    cout << "Leg 2 pos:" << leg2(0) << ", " << leg2(1) << ", " << leg2(2) << endl;
    cout << "Leg 3 pos:" << leg3(0) << ", " << leg3(1) << ", " << leg3(2) << endl;
    cout << "Leg 4 pos:" << leg4(0) << ", " << leg4(1) << ", " << leg4(2) << endl;

    return 0;
}


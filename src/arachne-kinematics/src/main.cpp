#define _USE_MATH_DEFINES
#include <bits/stdc++.h> 
#include <chrono>
#include <sensor_msgs/JointState.h>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"
#include "forward_kinematics.hpp"
#include "inverse_kinematics.hpp"
#include "control.hpp"
//#include "com_control.hpp"
//#include "printConfig.hpp"
//#include "walking.hpp"


int main(int argc, char **argv) {
    // Variable declaration
    VectorXd q = VectorXd::Zero(18);            // Joint configuration vector
    Vector3d com_xd;                            // Desired position of the center of mass
    Parameters parameters;                      // Parameters for the quadratic optimization
    MatrixXd qf(parameters.max_iterations, 18); // Intermediate-steps matrix
    Legs pos;                                   // Robot state class
    pos.update_position(q);
    pos.update_jacobian(q);

    // ROS node initialization
    ros::init(argc, argv, "arachneNode");
    ros::start();
    ros::NodeHandle n;
    ros::Rate loop_rate(3.5);

    // ROS publisher for the robot's joint position controller
    ros::Publisher joint_publisher[q.size()];
    for (int i = 0; i < q.size(); ++i)
        joint_publisher[i] = n.advertise<std_msgs::Float64>("/robot/joint" + std::to_string(i+1) + "_position_controller/command", 10);

    // ROS publisher for the robot's desired position
    ros::Publisher desiredPosition_publisher = n.advertise<std_msgs::Float64MultiArray>("/desired_pos", 10);
    
    // ROS publisher for the robot's visualization
    ros::Publisher rviz_publisher = n.advertise<sensor_msgs::JointState>("/joint_states", 10);

    // ROS messages declaration
    std_msgs::Float64 joints[q.size()];
    std_msgs::Float64MultiArray desiredPosition;
    sensor_msgs::JointState rviz_data;

    // Desired position [posb (base position), rotb (base orientation), leg1, leg2, leg3, leg4]:
    VectorXd xd(18);
    xd.segment( 0, 3) << 0, 0, 0;
    xd.segment( 3, 3) << 0 * pi/180, 0 * pi/180, 0 * pi/180;
    xd.segment( 6, 3) << pos.leg1(0), pos.leg1(1), pos.leg1(2) + 6 ;
    xd.segment( 9, 3) << pos.leg2(0), pos.leg2(1), pos.leg2(2);
    xd.segment(12, 3) << pos.leg3(0), pos.leg3(1), pos.leg3(2);
    xd.segment(15, 3) << pos.leg4(0), pos.leg4(1), pos.leg4(2);

    // Desired position of the COM
    com_xd << 0, 0, 0; //com_xd = cmass(q);
    
    // Joint names for the visualization
    rviz_data.name.clear();
    rviz_data.name.push_back("leg1_motor1");
    rviz_data.name.push_back("leg1_motor2");
    rviz_data.name.push_back("leg1_motor3");
    rviz_data.name.push_back("leg2_motor1");  
    rviz_data.name.push_back("leg2_motor2");
    rviz_data.name.push_back("leg2_motor3");
    rviz_data.name.push_back("leg3_motor1");
    rviz_data.name.push_back("leg3_motor2");
    rviz_data.name.push_back("leg3_motor3");
    rviz_data.name.push_back("leg4_motor1");
    rviz_data.name.push_back("leg4_motor2");
    rviz_data.name.push_back("leg4_motor3");

    // // Valores para la caminata
    // int ind = 0;
    // VectorXd posd(4);
    // std::cout << "Posicion actual: \n" << " xa: ";
    // std::cin >> posd(0);
    // std::cout << " ya: ";
    // std::cin >> posd(1);
    // std::cout << "Posicion deseada: \n" << " xd: ";
    // std::cin >> posd(2);
    // std::cout << " yd: ";
    // std::cin >> posd(3);
    // MatrixXd final(18,300);
    // walking(posd, pos, final, ind);

    // Se escriben los valores en un archivo
    //std::ofstream myfile;
    //myfile.open("charlyposgait.txt");

    int count = 0;
    while (ros::ok()) {  
        //std::cout << "Numero de paso" << endl;
        //std::cin >> count;
        
        // Quadratic program
        quadratic_program(qf, q, xd, parameters, com_xd, pos);
        //quad_prog(qf, q, final.col(count), parameters, com_xd, pos);
        
        // Compare desired and current positions(x, y, z)
        //compare(xd, com_xd, pos);
        //std::cout << final.col(count) << endl;
        //compare(final.col(count), com_xd, pos);
        //std::cout << endl << q << endl;

        for (int i = 0; i < q.size(); ++i)
            joints[i].data = q(i);
        
        // Se prepara la data de posicion deseada
        std::vector<double> rosxd;
        //rosxd.resize(final.col(count).size());
        //VectorXd::Map(&rosxd[0], final.col(count).size()) = final.col(count);

        // Se inserta la data en el array
        desiredPosition.data.clear();
        for (int i = 0; i < rosxd.size(); ++i)
            desiredPosition.data.push_back(rosxd[i]);

        // Se prepara la data de configuracion actual
        std::vector<double> rvizq;
        rvizq.resize(q.size());
        VectorXd::Map(&rvizq[0], q.size()) = q;

        // Se inserta la data en el array
        rviz_data.position.clear();
        rviz_data.header.stamp = ros::Time::now();
        for (int i = 6; i < 18; i++)
            rviz_data.position.push_back(rvizq[i]);

        // Se publican los valores del espacio articular
        for (int i = 0; i < q.size(); ++i)
            joint_publisher[i].publish(joints[i]);

        desiredPosition_publisher.publish(desiredPosition);
        rviz_publisher.publish(rviz_data);

        ros::spinOnce();
        loop_rate.sleep();
        
        // if (count == ind - 1) {
        //     count = 0;
        //     posd(0) = posd(2);
        //     posd(1) = posd(3);
        //     std::cout << "\nPosicion actual:\n" << " xa: " << posd(0) << "\n ya: " << posd(1);
        //     std::cout << "\nPosicion deseada:\n" << " xd: ";
        //     std::cin >> posd(2);
        //     std::cout << " yd: ";
        //     std::cin >> posd(3);
        //     walking(posd, pos, final, ind);
        // } else if (count < ind - 1) {
        //     // print_position(qf, myfile, parameters);
        //     count++;
        // }
    }
    //myfile.close();
}
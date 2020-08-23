#define _USE_MATH_DEFINES

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"
#include <sensor_msgs/JointState.h>
#include <bits/stdc++.h> 
#include <chrono>

#include "forward_kinematics.hpp"
#include "inverse_kinematics.hpp"
#include "control.hpp"
#include "com_control.hpp"
#include "printConfig.hpp"
#include "walking.hpp"


int main(int argc, char **argv) {
    // Se inician los nodos y topicos de ROS
    ros::init(argc, argv, "sendJointsGzNode");
    ros::start();
    ros::NodeHandle n;

    // Se publica el espacio articular actual del robot
    ros::Publisher pub_leg1m1 = n.advertise<std_msgs::Float64>("/robot/joint1_position_controller/command", 10);
    ros::Publisher pub_leg1m2 = n.advertise<std_msgs::Float64>("/robot/joint2_position_controller/command", 10);
    ros::Publisher pub_leg1m3 = n.advertise<std_msgs::Float64>("/robot/joint3_position_controller/command", 10);
    ros::Publisher pub_leg2m1 = n.advertise<std_msgs::Float64>("/robot/joint4_position_controller/command", 10);
    ros::Publisher pub_leg2m2 = n.advertise<std_msgs::Float64>("/robot/joint5_position_controller/command", 10);
    ros::Publisher pub_leg2m3 = n.advertise<std_msgs::Float64>("/robot/joint6_position_controller/command", 10);
    ros::Publisher pub_leg3m1 = n.advertise<std_msgs::Float64>("/robot/joint7_position_controller/command", 10);
    ros::Publisher pub_leg3m2 = n.advertise<std_msgs::Float64>("/robot/joint8_position_controller/command", 10);
    ros::Publisher pub_leg3m3 = n.advertise<std_msgs::Float64>("/robot/joint9_position_controller/command", 10);
    ros::Publisher pub_leg4m1 = n.advertise<std_msgs::Float64>("/robot/joint10_position_controller/command", 10);
    ros::Publisher pub_leg4m2 = n.advertise<std_msgs::Float64>("/robot/joint11_position_controller/command", 10);
    ros::Publisher pub_leg4m3 = n.advertise<std_msgs::Float64>("/robot/joint12_position_controller/command", 10);
    ros::Publisher pub_posbx = n.advertise<std_msgs::Float64>("/robot/joint13_position_controller/command", 10);
    ros::Publisher pub_posby = n.advertise<std_msgs::Float64>("/robot/joint14_position_controller/command", 10);
    ros::Publisher pub_posbz = n.advertise<std_msgs::Float64>("/robot/joint15_position_controller/command", 10);
    ros::Publisher pub_rotbx = n.advertise<std_msgs::Float64>("/robot/joint16_position_controller/command", 10);
    ros::Publisher pub_rotby = n.advertise<std_msgs::Float64>("/robot/joint17_position_controller/command", 10);
    ros::Publisher pub_rotbz = n.advertise<std_msgs::Float64>("/robot/joint18_position_controller/command", 10);

    // Se publica el espacio de configuracion deseado del robot
    ros::Publisher pub_desired = n.advertise<std_msgs::Float64MultiArray>("/desired_pos", 10);
    ros::Publisher pub_rviz = n.advertise<sensor_msgs::JointState>("/joint_states", 10);

    ros::Rate loop_rate(3.5);

    // Se declaran los mensajes de ROS
    std_msgs::Float64 j1;
    std_msgs::Float64 j2;
    std_msgs::Float64 j3;
    std_msgs::Float64 j4;
    std_msgs::Float64 j5;
    std_msgs::Float64 j6;
    std_msgs::Float64 j7;
    std_msgs::Float64 j8;
    std_msgs::Float64 j9;
    std_msgs::Float64 j10;
    std_msgs::Float64 j11;
    std_msgs::Float64 j12;
    std_msgs::Float64 j13;
    std_msgs::Float64 j14;
    std_msgs::Float64 j15;
    std_msgs::Float64 j16;
    std_msgs::Float64 j17;
    std_msgs::Float64 j18;
    std_msgs::Float64MultiArray jdesired;
    sensor_msgs::JointState rviz_val;

    // Se asignan las variables 
    VectorXd q = VectorXd::Zero(18);
    Vector3d com_xd;
    Parameters parameters;
    MatrixXd qf(parameters.max_iter, 18);
    Legs pos;
    
    pos.update_position(q);
    pos.update_jacobian(q);

    // Desired position[posb, leg1, leg2, leg3, leg4, rotb]:
    //xd.segment(0, 3)  << 3, 2, 0;
    //xd.segment(3, 3)  << 3 + pos.leg1(0), 2 + pos.leg1(1), 0 + pos.leg1(2);
    //xd.segment(6, 3)  << 1 + pos.leg2(0), 0 + pos.leg2(1), 0 + pos.leg2(2);
    //xd.segment(9, 3)  << 0 + pos.leg3(0), 1 + pos.leg3(1), 0 + pos.leg3(2);
    //xd.segment(12, 3) << 0 + pos.leg4(0), 0 + pos.leg4(1), 1 + pos.leg4(2);
    //xd.segment(15, 3) << 0 * pi / 180, 0 * pi / 180, 0 * pi / 180;

    // Desired position[posb, leg1, leg2, leg3, leg4, rotb]:
    VectorXd xd(18);
    xd.segment(0, 3)  << 0, 0, 0;
    xd.segment(3, 3)  << pos.leg1(0), pos.leg1(1), pos.leg1(2) + 2;
    xd.segment(6, 3)  << pos.leg2(0), pos.leg2(1), pos.leg2(2) + 2;
    xd.segment(9, 3)  << pos.leg3(0), pos.leg3(1), pos.leg3(2) + 2;
    xd.segment(12, 3) << pos.leg4(0), pos.leg4(1), pos.leg4(2) + 2;
    xd.segment(15, 3) << 0 * pi / 180, 0 * pi / 180, 0 * pi / 180;

    // Desired position of COM
    //com_xd = cmass(q);
    //com_xd << 2, 0.8, -3.8;
    com_xd << 0, 0, 0;
    
    // Se prepara la data de posiciones actuales
    std::vector<std::string> names;
    names.push_back("leg1_motor1");
    names.push_back("leg2_motor1");
    names.push_back("leg3_motor1");
    names.push_back("leg4_motor1");  
    names.push_back("leg1_motor2");
    names.push_back("leg2_motor2");
    names.push_back("leg3_motor2");
    names.push_back("leg4_motor2");
    names.push_back("leg1_motor3");
    names.push_back("leg2_motor3");
    names.push_back("leg3_motor3");
    names.push_back("leg4_motor3");
    
    // Se inserta la data en el array
    rviz_val.name.clear();
    for (int i = 0; i < names.size(); i++)
        rviz_val.name.push_back(names[i]);
    
    int count = 0;

    // Valores para la caminata
    int ind = 0;
    VectorXd posd(4);
    std::cout << "Posicion actual: \n" << " xa: ";
    std::cin >> posd(0);
    std::cout << " ya: ";
    std::cin >> posd(1);
    std::cout << "Posicion deseada: \n" << " xd: ";
    std::cin >> posd(2);
    std::cout << " yd: ";
    std::cin >> posd(3);
    MatrixXd final(18,300);
    walking(posd, pos, final, ind);

    // Se escriben los valores en un archivo
    std::ofstream myfile;
    myfile.open("charlyposgait.txt");

    while (ros::ok()) {  
        //std::cout << "Numero de paso" << endl;
        //std::cin >> count;
        //auto start = std::chrono::system_clock::now();
        // Quadratic program
        //quad_prog(qf, q, xd, parameters, com_xd, pos);
        quad_prog(qf, q, final.col(count), parameters, com_xd, pos);
        
        //auto end = std::chrono::system_clock::now();
        //std::chrono::duration<float,std::milli> duration = end - start;
        //std::cout << duration.count() << "s " << endl;
        
        // Compare desired and current positions(x, y, z)
        //compare(xd, com_xd, pos);
        //std::cout << final.col(count) << endl;
        //compare(final.col(count), com_xd, pos);
        //std::cout << endl << q << endl;

        j1.data = q(3);
        j2.data = q(4);
        j3.data = q(5);
        j4.data = q(6);
        j5.data = q(7);
        j6.data = q(8);
        j7.data = q(9);
        j8.data = q(10);
        j9.data = q(11);
        j10.data = q(12);
        j11.data = q(13);
        j12.data = q(14);
        j13.data = q(0);
        j14.data = q(1);
        j15.data = q(2);
        j16.data = q(15);
        j17.data = q(16);
        j18.data = q(17);
        
        // Se prepara la data de posicion deseada
        std::vector<double> rosxd;
        rosxd.resize(final.col(count).size());
        VectorXd::Map(&rosxd[0], final.col(count).size()) = final.col(count);

        // Se inserta la data en el array
        jdesired.data.clear();
        for (int i = 0; i < rosxd.size(); i++)
            jdesired.data.push_back(rosxd[i]);

        // Se prepara la data de configuracion actual
        std::vector<double> rvizq;
        rvizq.resize(q.size());
        VectorXd::Map(&rvizq[0], q.size()) = q;

        // Se inserta la data en el array
        rviz_val.position.clear();
        rviz_val.header.stamp = ros::Time::now();
        rviz_val.position.push_back(rvizq[3]);
        rviz_val.position.push_back(rvizq[6]);
        rviz_val.position.push_back(rvizq[9]);
        rviz_val.position.push_back(rvizq[12]);
        rviz_val.position.push_back(rvizq[4]);
        rviz_val.position.push_back(rvizq[7]);
        rviz_val.position.push_back(rvizq[10]);
        rviz_val.position.push_back(rvizq[13]);
        rviz_val.position.push_back(rvizq[5]);
        rviz_val.position.push_back(rvizq[8]);
        rviz_val.position.push_back(rvizq[11]);
        rviz_val.position.push_back(rvizq[14]);

        // Se publican los valores del espacio articular
        pub_leg1m1.publish(j1);
        pub_leg1m2.publish(j2);
        pub_leg1m3.publish(j3);
        pub_leg2m1.publish(j4);
        pub_leg2m2.publish(j5);
        pub_leg2m3.publish(j6);
        pub_leg3m1.publish(j7);
        pub_leg3m2.publish(j8);
        pub_leg3m3.publish(j9);
        pub_leg4m1.publish(j10);
        pub_leg4m2.publish(j11);
        pub_leg4m3.publish(j12);
        pub_posbx.publish(j13);
        pub_posby.publish(j14);
        pub_posbz.publish(j15);
        pub_rotbx.publish(j16);
        pub_rotby.publish(j17);
        pub_rotbz.publish(j18);
        pub_desired.publish(jdesired);
        pub_rviz.publish(rviz_val);

        ros::spinOnce();
        loop_rate.sleep();
        
        if (count == ind - 1) {
            count = 0;
            posd(0) = posd(2);
            posd(1) = posd(3);
            std::cout << "\nPosicion actual:\n" << " xa: " << posd(0) << "\n ya: " << posd(1);
            std::cout << "\nPosicion deseada:\n" << " xd: ";
            std::cin >> posd(2);
            std::cout << " yd: ";
            std::cin >> posd(3);
            walking(posd, pos, final, ind);
        } else if (count < ind - 1) {
            // print_position(qf, myfile, parameters);
            count++;
        }
    }
    myfile.close();
}
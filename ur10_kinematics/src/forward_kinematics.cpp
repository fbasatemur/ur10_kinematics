#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <Eigen/Dense> 
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32MultiArray.h>

using namespace Eigen;

std::vector<float> joint_angles{ 0., 0., 0., 0., 0., 0. };

Matrix4d ur10ForwardKinematics(std::vector<float>& q) {
    // End-effector point: E
    // The H matrix from point B to point A is denoted as A_H_B

    Matrix4d PB_H_P0, P0_H_P1, P1_H_P2, P2_H_P3, P3_H_P4, P4_H_P5, P5_H_P6, P6_H_P7, P7_H_E;

    PB_H_P0 <<  1,      0,      0,      0,
                0,      1,      0,      0,
                0,      0,      1,      2,
                0,      0,      0,      1;

    P0_H_P1 <<  cos(q[0]), -sin(q[0]),  0,   0,
                sin(q[0]),  cos(q[0]),  0,   0,
                0,          0,          1,   0.1273,
                0,          0,          0,   1;

    P1_H_P2 <<  cos(q[1]),      0,  sin(q[1]),          0,
                0,              1,         0,    0.220941,
                -sin(q[1]),     0,  cos(q[1]),          0,
                0,              0,         0,           1;

    P2_H_P3 <<  cos(q[2]),      0,  sin(q[2]),      0.612,
                0,              1,         0,           0,
                -sin(q[2]),     0,  cos(q[2]),          0,
                0,              0,         0,           1;

    P3_H_P4 << 1,          0,         0,        0,
                0,         1,         0,      -0.1719,
                0,         0,         1,       0,
                0,         0,         0,       1;

    P4_H_P5 << cos(q[3]),       0,  sin(q[3]),     0.5723,
                0,              1,         0,           0,
                -sin(q[3]),     0,  cos(q[3]),          0,
                0,              0,         0,           1;

    
    P5_H_P6 <<  cos(q[4]), -sin(q[4]),  0,          0,
                sin(q[4]),  cos(q[4]),  0,       0.1149,
                0,           0,         1,         0,
                0,           0,         0,         1;

    
    P6_H_P7 << cos(q[5]),       0,   sin(q[5]),       0,
                0,              1,      0,            0,
                -sin(q[5]),     0,   cos(q[5]),     -0.1157,
                0,              0,         0,         1;

    P7_H_E <<   1,      0,      0,      0,
                0,      1,      0,      0.1716059,
                0,      0,      1,      0,
                0,      0,      0,      1;

    // The transformation matrix is obtained
    Matrix4d T = PB_H_P0 * P0_H_P1 * P1_H_P2 * P2_H_P3 * P3_H_P4 * P4_H_P5 * P5_H_P6 * P6_H_P7 * P7_H_E;

    return T;
}

Eigen::Quaterniond rotationMatrixToQuaterniond(const Eigen::Matrix3d& rotation_matrix) {
    return Eigen::Quaterniond {rotation_matrix};
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg_p) {
    
    std::cout <<"[ odom_msg ]\n";
    std::cout <<"pose \n" << odom_msg_p->pose.pose.position << std::endl;
    std::cout <<"orientation \n" << odom_msg_p->pose.pose.orientation << std::endl;

    // The forward kinematic calculation is performed
    Matrix4d endEffectorPose = ur10ForwardKinematics(joint_angles);

    // The end-effector position and orientation are obtained
    Vector3d endEffectorPosition = endEffectorPose.block<3,1>(0,3);
    Matrix3d endEffectorOrientation = endEffectorPose.block<3,3>(0,0);


    std::cout <<"[ calculated ] \n";
    std::cout <<"pose \n"<< endEffectorPosition.transpose() << std::endl;
    std::cout <<"orientation \n"<< rotationMatrixToQuaterniond(endEffectorOrientation).coeffs().transpose() << std::endl;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "forward_kinematics_node");
    ros::NodeHandle nh;

    ros::Publisher acilar_pub = nh.advertise<std_msgs::Float32MultiArray>("/ur10_arm/acilar", 1);
    ros::Subscriber sub = nh.subscribe("/ur10_arm/odometri", 1, odomCallback);
    std_msgs::Float32MultiArray joint_msg;
    ros::Rate loop_rate(1);

    while (ros::ok()) {
        joint_angles = { 0.5, -0.2, 0.6, -0.6, -0.4, 0.5 };
        joint_msg.data = joint_angles;

        acilar_pub.publish(joint_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

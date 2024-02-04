#include <ros/ros.h>
#include <eigen3/Eigen/Core>

#include "std_msgs/String.h"
#include <std_msgs/Float32.h>

#include <std_msgs/Float32MultiArray.h>
#include <sstream>



/*
//Float32MultiArray tipinde bir mesaj oluþturmak icin gerekli adýmlar

vector<double> vec1 = { 1.1, 2., 3.1};
std_msgs::Float32MultiArray msg;

// set up dimensions
msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
msg.layout.dim[0].size = vec1.size();
msg.layout.dim[0].stride = 1;
msg.layout.dim[0].label = "x";

// copy in the data
msg.data.clear();
msg.data.insert(msg.data.end(), vec1.begin(), vec1.end());
*/



/*
 * ornek matris carpimi
 *
 Eigen::Matrix4d mat1, mat2;
 mat1 << 1,2,3,4,1,2,3,4,2,3,4,5,2,3,4,5;
 mat2 << 2,2,3,4,1,2,3,4,2,3,4,5,2,3,4,cos(M_PI);
 std::cout<<(mat1*mat2)(0,0)<<std::endl;
 *
 */



int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  ros::Publisher sag_pub = n.advertise<std_msgs::Float32>("/diff_drive_robot/sag_teker",1);
  ros::Publisher sol_pub = n.advertise<std_msgs::Float32>("/diff_drive_robot/sol_teker",1);

  ros::Rate loop_rate(10);


  std_msgs::Float32 sag_cmd, sol_cmd;
  sag_cmd.data = -0.5;
  sol_cmd.data = 0.5;


  int count=0;

  while (ros::ok() && count<=100)
  {
    ros::Time current_Time = ros::Time::now();


    sag_pub.publish(sag_cmd);
    sol_pub.publish(sol_cmd);

    ros::spinOnce();

    count++;
    loop_rate.sleep();

  }
  sag_cmd.data = 0;
  sol_cmd.data = 0;
  sag_pub.publish(sag_cmd);
  sol_pub.publish(sol_cmd);
   ros::spin();


  return 0;
}

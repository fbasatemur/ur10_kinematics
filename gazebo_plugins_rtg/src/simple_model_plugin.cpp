#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <stdio.h>
#include <thread>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <ros/advertise_options.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>

namespace gazebo
{
class ModelPluginRGT : public ModelPlugin
{
public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        if (!ros::isInitialized())
        {
            ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin.");
            return;
        }
        // Store the pointer to the model
        last_time =ros::Time::now();
        this->model = _parent;

        gazebo::common::PID PID = gazebo::common::PID(0.8,0.5,0.1);

        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                    boost::bind(&ModelPluginRGT::OnUpdate, this, _1));

        std::vector<physics::LinkPtr> linkler= this->model->GetLinks();

        std::vector<physics::JointPtr> jointler= this->model->GetJoints();

        std::cout<<linkler.size()<<std::endl;


        for (int i=0;i<linkler.size();i++){
            std::cout<<linkler[i]->GetName()<<std::endl;
        }

        std::cout<<jointler.size()<<std::endl;


        for (int i=0;i<jointler.size();i++){

            this->model->GetJointController()->SetVelocityPID(jointler[i]->GetScopedName() ,PID);
            std::cout<<jointler[i]->GetScopedName()<<std::endl;
        }

        this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

        ros::SubscribeOptions so =
                ros::SubscribeOptions::create<std_msgs::Float32>(
                    "/" + this->model->GetName() + "/sol_teker",
                    1,
                    boost::bind(&ModelPluginRGT::OnRosMsg, this, _1),
                    ros::VoidPtr(), &this->rosQueue);
        this->rosSub = this->rosNode->subscribe(so);
        this->rosQueueThread = std::thread(std::bind(&ModelPluginRGT::QueueThread, this));

        ros::SubscribeOptions so1 =
                ros::SubscribeOptions::create<std_msgs::Float32>(
                    "/" + this->model->GetName() + "/sag_teker",
                    1,
                    boost::bind(&ModelPluginRGT::OnRosMsg1, this, _1),
                    ros::VoidPtr(), &this->rosQueue1);
        this->rosSub1 = this->rosNode->subscribe(so1);
        this->rosQueueThread1 = std::thread(std::bind(&ModelPluginRGT::QueueThread1, this));

        ros::SubscribeOptions so2 =
                ros::SubscribeOptions::create<geometry_msgs::Twist>(
                    "/" + this->model->GetName() + "/cmd_vel",
                    1,
                    boost::bind(&ModelPluginRGT::OnRosMsg2, this, _1),
                    ros::VoidPtr(), &this->rosQueue2);
        this->rosSub2 = this->rosNode->subscribe(so2);
        this->rosQueueThread2 = std::thread(std::bind(&ModelPluginRGT::QueueThread2, this));


        this->sol_hiz =0;
        this->sag_hiz =0;
        this->robot_cmd.angular.x = 0;
        this->robot_cmd.angular.y = 0;
        this->robot_cmd.angular.z = 0;
        this->robot_cmd.linear.x = 0;
        this->robot_cmd.linear.y = 0;
        this->robot_cmd.linear.z = 0;


        this->rosPub = this->rosNode->advertise<nav_msgs::Odometry>("/" + this->model->GetName() + "/odometri",1);

    }

public: void OnRosMsg(const std_msgs::Float32ConstPtr &_msg)
    {
        this->sol_hiz=_msg->data;
    }

public: void OnRosMsg1(const std_msgs::Float32ConstPtr &_msg)
    {
        this->sag_hiz=_msg->data;
    }

public: void OnRosMsg2(const geometry_msgs::TwistConstPtr &_msg)
    {
        double L_distance = 0.85;
        double R_distance = 0.3;
        this->robot_cmd.angular.x = _msg->angular.x;
        this->robot_cmd.angular.y = _msg->angular.y;
        this->robot_cmd.angular.z = _msg->angular.z;
        this->robot_cmd.linear.x = _msg->linear.x;
        this->robot_cmd.linear.y = _msg->linear.y;
        this->robot_cmd.linear.z = _msg->linear.z;
        this->sag_hiz = -(this->robot_cmd.angular.z * L_distance + this->robot_cmd.linear.x * 2) / (R_distance * 2) ;
        this->sol_hiz = (-this->robot_cmd.angular.z * L_distance + this->robot_cmd.linear.x * 2) / (R_distance * 2) ;
        //std::cout<< this->robot_cmd.linear.x << std::endl;
    }

    /// \brief ROS helper function that processes messages
private: void QueueThread()
    {
        static const double timeout = 0.01;
        while (this->rosNode->ok())
        {
            this->rosQueue.callAvailable(ros::WallDuration(timeout));
        }
    }

private: void QueueThread1()
    {
        static const double timeout = 0.01;
        while (this->rosNode->ok())
        {
            this->rosQueue1.callAvailable(ros::WallDuration(timeout));
        }
    }

private: void QueueThread2()
    {
        static const double timeout = 0.01;
        while (this->rosNode->ok())
        {
            this->rosQueue2.callAvailable(ros::WallDuration(timeout));
        }
    }

    // Called by the world update start event
public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
        this->model->GetJointController()->SetVelocityTarget("diff_drive_robot::DiffDrive::govde_sagteker",this->sag_hiz);
        this->model->GetJointController()->SetVelocityTarget("diff_drive_robot::DiffDrive::govde_solteker",this->sol_hiz);

        publishOdometry();

    }

public: void publishOdometry()
    {
        ros::Time current_time = ros::Time::now();
        if((current_time-last_time).toSec()>0.1)
        {
            std::string odom_frame = "world";
            std::string base_footprint_frame = "base_link";
            tf::Quaternion qt;
            tf::Vector3 vt;
            ignition::math::Pose3d pose = this->model->WorldPose();
            qt = tf::Quaternion ( pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W());
            vt = tf::Vector3 ( pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z() );

            this->odom_.pose.pose.position.x = vt.x();
            this->odom_.pose.pose.position.y = vt.y();
            this->odom_.pose.pose.position.z = vt.z();

            this->odom_.pose.pose.orientation.x = qt.x();
            this->odom_.pose.pose.orientation.y = qt.y();
            this->odom_.pose.pose.orientation.z = qt.z();
            this->odom_.pose.pose.orientation.w = qt.w();

            // get velocity in /odom frame
            ignition::math::Vector3d linear;
            linear = this->model->WorldLinearVel();
            this->odom_.twist.twist.angular.z = this->model->WorldAngularVel().Z();

            // convert velocity to child_frame_id (aka base_footprint)
            float yaw = pose.Rot().Yaw();
            this->odom_.twist.twist.linear.x = cos ( yaw ) * linear.X() + sin ( yaw ) * linear.Y();
            this->odom_.twist.twist.linear.y = cos ( yaw ) * linear.Y() - sin ( yaw ) * linear.X();

            this->odom_.header.stamp = current_time;
            this->odom_.header.frame_id = odom_frame;
            this->odom_.child_frame_id = base_footprint_frame;

            //std::cout <<this->odom_.twist.twist.linear.x <<std::endl;
            rosPub.publish(this->odom_);
            last_time = current_time;
        }

    }

    // Pointer to the model
private: physics::ModelPtr model;

    // Pointer to the update event connection
private: event::ConnectionPtr updateConnection;

    /// \brief A node use for ROS transport
private: std::unique_ptr<ros::NodeHandle> rosNode;

    /// \brief A ROS subscriber
private: ros::Subscriber rosSub;
private: ros::Subscriber rosSub1;
private: ros::Subscriber rosSub2;


    /// \brief A ROS publisher
private: ros::Publisher rosPub;

    /// \brief A ROS callbackqueue that helps process messages
private: ros::CallbackQueue rosQueue;
private: ros::CallbackQueue rosQueue1;
private: ros::CallbackQueue rosQueue2;

    /// \brief A thread the keeps running the rosQueue
private: std::thread rosQueueThread;
private: std::thread rosQueueThread1;
private: std::thread rosQueueThread2;

private: double sol_hiz,sag_hiz;
private: geometry_msgs::Twist robot_cmd;
private: nav_msgs::Odometry odom_;
private: ros::Time last_time;

};

GZ_REGISTER_MODEL_PLUGIN(ModelPluginRGT)
}

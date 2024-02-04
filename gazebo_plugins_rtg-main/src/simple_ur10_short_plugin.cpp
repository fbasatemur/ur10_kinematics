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

        gazebo::common::PID PID = gazebo::common::PID(900,0.9,50);
        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                    boost::bind(&ModelPluginRGT::OnUpdate, this, _1));

        std::vector<physics::LinkPtr> linkler= this->model->GetLinks();

        std::vector<physics::JointPtr> jointler= this->model->GetJoints();

        std::cout<<linkler.size()<<std::endl;

        for (int i=0;i<linkler.size();i++){
            std::cout<<linkler[i]->GetName()<<std::endl;
            //math::Pose pose = this->model->GetLink(linkler[i]->GetName())->GetWorldPose();
            //std::cout<<"x:"<<pose.pos.x<<"\ty:"<<pose.pos.y<<"\tz:"<<pose.pos.z<<std::endl;
        }

        std::cout<<jointler.size()<<std::endl;


        for (int i=0;i<jointler.size();i++){
            this->model->GetJointController()->SetPositionPID(jointler[i]->GetScopedName() ,PID);
            std::cout<<jointler[i]->GetScopedName()<<std::endl;
            //math::Pose pose =this->model->GetJoint(jointler[i]->GetScopedName())->GetWorldPose();
            //std::cout<<"x:"<<pose.pos.x<<"\ty:"<<pose.pos.y<<"\tz:"<<pose.pos.z<<std::endl;
        }

        this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

        ros::SubscribeOptions so =
                ros::SubscribeOptions::create<std_msgs::Float32MultiArray>(
                    "/" + this->model->GetName() + "/acilar",
                    1,
                    boost::bind(&ModelPluginRGT::OnRosMsg, this, _1),
                    ros::VoidPtr(), &this->rosQueue);
        this->rosSub = this->rosNode->subscribe(so);
        this->rosQueueThread = std::thread(std::bind(&ModelPluginRGT::QueueThread, this));


        for (int i=0;i<3;i++){
            this->acilar[i]=0;
        }




        this->rosPub = this->rosNode->advertise<nav_msgs::Odometry>("/" + this->model->GetName() + "/odometri",1);

    }

public: void OnRosMsg(const std_msgs::Float32MultiArrayConstPtr &_msg)
    {
        for(int i=0;i<3;i++)
        {
            this->acilar[i]=_msg->data[i];
            std::cout << this->acilar[i] << " " ;
        }
        std::cout << std::endl;

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


    // Called by the world update start event
public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {

        this->model->GetJointController()->SetPositionTarget("ur10_arm::ur10_short::shoulder_pan",this->acilar[0]);
        this->model->GetJointController()->SetPositionTarget("ur10_arm::ur10_short::shoulder_lift",this->acilar[1]);
        this->model->GetJointController()->SetPositionTarget("ur10_arm::ur10_short::elbow",this->acilar[2]);

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
            ignition::math::Pose3d pose = this->model->GetLink("ur10_short::tip")->WorldPose();
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


    /// \brief A ROS publisher
private: ros::Publisher rosPub;

    /// \brief A ROS callbackqueue that helps process messages
private: ros::CallbackQueue rosQueue;

    /// \brief A thread the keeps running the rosQueue
private: std::thread rosQueueThread;

private: double acilar[3];
private: nav_msgs::Odometry odom_;
private: ros::Time last_time;

};

GZ_REGISTER_MODEL_PLUGIN(ModelPluginRGT)
}

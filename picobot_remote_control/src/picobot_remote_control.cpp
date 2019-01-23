#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include "teleop_twist_joy/teleop_twist_joy.h"
#include <iostream>
#include <string.h>
#include <unistd.h>

using namespace std;

class PicoRemoteControl
{
public:
  PicoRemoteControl();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

 
  ros::NodeHandle nh_;

  int lidar_status;
  int linear_, angular_;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;
};


PicoRemoteControl::PicoRemoteControl():
  linear_(1),//naprej,nazaj 1
  angular_(0),//levo desno 0
  a_scale_(1.4),
  l_scale_(0.4)
{

  //nh_.param("axis_linear", linear_, linear_);
  //nh_.param("axis_angular", angular_, angular_);
  //nh_.param("scale_angular", a_scale_, a_scale_);
  //nh_.param("scale_linear", l_scale_, l_scale_);

  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("pico/cmd_vel", 1);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &PicoRemoteControl::joyCallback, this);

}

void PicoRemoteControl::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{

  if(joy->buttons[3] > 0){//Y
    system("cd && rosservice call /start_motor");
    ROS_INFO("Vklaplam motor");
    }
else if(joy->buttons[2] > 0){//B
    system("cd && rosservice call /stop_motor");
    ROS_INFO("Izklapljam motor");
    }

if (joy->buttons[4] > 0){//LB
if (joy->buttons[0] > 0) {//X
l_scale_+=0.5;
a_scale_ +=0.7;
if(l_scale_ >= 1){
l_scale_ = 0.9;
}
if(a_scale_ >= 2.1){
a_scale_ = 2.1;
}
}
if (joy->buttons[1] > 0) {//A
l_scale_-=0.5;
a_scale_ -=0.7;
if(l_scale_ <= 0){
l_scale_=0.4;
}
if(a_scale_ <=0.7){
a_scale_ = 1.4;
}
}
}


  geometry_msgs::Twist twist;

  if(joy->buttons[5] > 0){//je RB
  twist.angular.z = a_scale_*joy->axes[angular_];
  twist.linear.x = l_scale_*joy->axes[linear_];
  vel_pub_.publish(twist);
  }else{
  twist.angular.z = 0;
  twist.linear.x = 0;
  vel_pub_.publish(twist); 
  }
}


int main(int argc, char** argv)
{
  //  system("cd && roslaunch /home/pi/catkin_ws/src/Picobot/picobot_imu/launch/picobot_imu.launch");
  ros::init(argc, argv, "picobot_remote_control");
  PicoRemoteControl pico_remote_control;
  ros::spin();
}


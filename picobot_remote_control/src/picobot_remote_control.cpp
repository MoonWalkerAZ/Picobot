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

  ros::NodeHandle n;

  int autoNavigation;
  int linear, angular;
  double linScale, angScale;
  ros::Publisher velPub;
  ros::Subscriber joySub;
};


PicoRemoteControl::PicoRemoteControl():

  linear(1),//levo,desno 1
  angular(0),//naprej,nazaj 0
  angScale(1.4),
  linScale(0.4)

{
  velPub = n.advertise<geometry_msgs::Twist>("pico/cmd_vel", 1);
  joySub = n.subscribe<sensor_msgs::Joy>("joy", 10, &PicoRemoteControl::joyCallback, this);
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


  //spremembe hitrosti
  if (joy->buttons[4] > 0){//LB
    if (joy->buttons[0] > 0) {//X
      linScale+=0.5;
      angScale +=0.7;
      if(linScale >= 1){
        linScale = 0.9;
      }
      if(angScale >= 2.1){
        angScale = 2.1;
      }
    }
    if (joy->buttons[1] > 0) {//A
      linScale-=0.5;
      angScale -=0.7;
      if(linScale <= 0){
        linScale=0.4;
      }
      if(angScale <=0.7){
        angScale = 1.4;
      }
    }
  }

  //zagon auto navigation
  if (joy->buttons[7] > 0){//RT
    system("cd && rosservice call /start_motor");
    n.setParam("/autoNavigation",1);
  }
  //izklop navigation
  if (joy->buttons[6] > 0){//LT
    n.setParam("/autoNavigation",0);
    system("cd && rosservice call /stop_motor");
  }

  geometry_msgs::Twist twist;

  if(joy->buttons[5] > 0){//RB varovalo
    twist.angular.z = angScale*joy->axes[angular];
    twist.linear.x = linScale*joy->axes[linear];
    velPub.publish(twist);
  }else{
    twist.angular.z = 0;
    twist.linear.x = 0;
    velPub.publish(twist);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "picobot_remote_control");
  PicoRemoteControl pico_remote_control;
  ros::spin();
}


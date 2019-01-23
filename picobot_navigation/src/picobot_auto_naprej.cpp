#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "sensor_msgs/LaserScan.h"
#include <vector>
#include <math.h>
#include <iostream>
#include <unistd.h>
#include <limits>
#include <stdlib.h>

using namespace std;

bool zataknil = false;
int stevec = 0;
float globKot;

class PicobotAuto{

public:
  PicobotAuto();

  struct Tocka{
    float x;
    float y;
    float kot;
    float kotA;
    float kotB;
    float razdaljaMedTockama;
  };

private:
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
  int stopinjaTan(int stopinja);
  ros::Subscriber sub;
  ros::Publisher pub;
  ros::NodeHandle n;
  int gyroYaw;
};

PicobotAuto::PicobotAuto(){

  sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, &PicobotAuto::scanCallback,this);
  pub = n.advertise<geometry_msgs::Twist>("pico/cmd_vel", 1);
}

void PicobotAuto::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){

  //Zaznavanje na min 15cm.
  //sirina: 30cm, dolžina: 36cm robota.

  int autoNavigation;
  n.getParam("/autoNavigation",autoNavigation);

  if(autoNavigation == 1){

    geometry_msgs::Twist twist;

    vector<float>sinx;
    vector<float>cosx;
    vector<float> razdalje;
    vector<int> koti;

    double inf = std::numeric_limits<double>::infinity();

    for(int i=0;i<scan->ranges.size();i++){

      if(scan->ranges[i] <= 0.75){
        razdalje.push_back(scan->ranges[i]);
        koti.push_back(i);
      }
    }

    //preverjamo razdalje na kotih od 225-180 in 180-135

    //n.getParam("/gyroYaw",gyroYaw);

    float razdalja = 0.33;
    bool zavijDesno, zavijLevo;

    float minLeva = inf;
    for(int i=0;i<45;i++){
      if(minLeva > scan->ranges[i] && scan->ranges[i] != inf){
        minLeva = scan->ranges[i];
      }
    }
    if (minLeva < razdalja || scan->ranges[60] < 0.24){
      zavijLevo = false;
    }else{
      zavijLevo = true;
    }

    float minDesna = inf;
    for(int i = 314;i<360;i++){
      if(minDesna > scan->ranges[i] && scan->ranges[i] != inf){
        minDesna = scan->ranges[i];
      }
    }
    if (minDesna < razdalja || scan->ranges[299] < 0.24){
      zavijDesno = false;
    }else{
      zavijDesno = true;
    }
    if (zataknil == false){
      if (zavijDesno && zavijLevo) {
        //ROS_INFO("gremo naravnost !!!");
        twist.linear.x = 0.1;  //optimalna hitrost
        twist.angular.z = 0;
        pub.publish(twist);
      }else{
        if(minDesna > minLeva){
          //ROS_INFO("minDesna: %f zavijDesno %i ",minDesna, zavijDesno);
          twist.linear.x = 0;
          twist.angular.z = -0.35;
          pub.publish(twist);
        }else{
          //ROS_INFO("minLeva: %f zavijLevo %i ",minLeva, zavijLevo);
          twist.linear.x = 0;
          twist.angular.z = 0.35;
          pub.publish(twist);
        }
      }
    }
    if (!zavijDesno && !zavijLevo){//zataknil
      zataknil = true;
      ROS_INFO("zataknil");
      stevec++;
    }else if(zavijDesno && zavijLevo){
      ROS_INFO("konec zatika");
      stevec = 0;
      zataknil = false;
    }

    if(zataknil){

      for(int i=0;i<koti.size();i++){

        sinx.push_back(sin(koti[i] * M_PI / 180));//stopinje v radiane
        cosx.push_back(cos (koti[i] * M_PI / 180));
      }

      //izracun x,y
      vector<Tocka> tocke;
      for(int i=0;i<razdalje.size();i++){
        Tocka tmp;
        tmp.x = razdalje[i] * cosx[i];
        tmp.y = razdalje[i] * sinx[i];
        tmp.kot = koti[i];
        tocke.push_back(tmp);
      }

      vector<float> razdaljeMedTockami;
      vector<Tocka>moznaRazpolovisca;

      //izracun oddaljenosti posameznih tock med seboj
      for(int i=0;i<tocke.size()-1;i++){
        float tmp = sqrt( pow((tocke[i+1].x - tocke[i].x ),2) + pow((tocke[i+1].y - tocke[i].y),2) );
        razdaljeMedTockami.push_back(tmp);
      }

      //preverimo ce je razdalja dovolj velika za robota
      for(int i=0;i<razdaljeMedTockami.size();i++){

        if(razdaljeMedTockami[i] > 0.4){

          Tocka S;//razpoloviscna tocka
          S.x = ((tocke[i].x + tocke[i+1].x)/2);
          S.y = ((tocke[i].y + tocke[i+1].y)/2);
          //kot
          S.kotA = tocke[i].kot;
          S.kotB = tocke[i+1].kot;
          S.kot = (tocke[i].kot + tocke[i+1].kot)/2;//atan(S.x/S.y) * (180.0/M_PI);
          S.razdaljaMedTockama = razdaljeMedTockami[i];
          moznaRazpolovisca.push_back(S);
        }
      }


      float max = 0.0;
      Tocka T;
      if (moznaRazpolovisca.size() > 0){
        //for(int i=0;i<moznaRazpolovisca.size();i++){
        // ROS_INFO("kotA: %f  (kot): %f  kotB: %f",moznaRazpolovisca[i].kotA,moznaRazpolovisca[i].kot,moznaRazpolovisca[i].kotB);
        //}
        for(int i=0;i<moznaRazpolovisca.size();i++){

          if(moznaRazpolovisca[i].razdaljaMedTockama > max){
            max = moznaRazpolovisca[i].razdaljaMedTockama;
            T = moznaRazpolovisca[i];
          }
        }

        if(stevec == 1){
          globKot = T.kot;
        }

        if (globKot <= 359 && globKot >= 180){
          ROS_INFO("obracam v desno, globKot %f", globKot);
          twist.angular.z = -0.35;
        }else {
          ROS_INFO("obracam v levo, globKot %f", globKot);
          twist.angular.z = 0.35;
        }
        twist.linear.x = 0;
        pub.publish(twist);

      }else{
        ROS_INFO("Ni moznih poti");
      }
      moznaRazpolovisca.clear();
      razdaljeMedTockami.clear();
      tocke.clear();
      koti.clear();
      razdalje.clear();
      sinx.clear();
      cosx.clear();
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "picobot_auto_naprej");
  PicobotAuto picobot_auto;
  ros::spin();
  return 0;
}

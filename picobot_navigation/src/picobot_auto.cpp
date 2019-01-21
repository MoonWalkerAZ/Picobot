#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "sensor_msgs/LaserScan.h"
#include <vector>
#include <math.h>
#include <iostream>
#include <unistd.h>
#include <limits>

using namespace std;

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
};

PicobotAuto::PicobotAuto(){

  sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, &PicobotAuto::scanCallback,this);
  pub = n.advertise<geometry_msgs::Twist>("pico/cmd_vel", 1);
}

void PicobotAuto::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){

  //Zaznavanje na min 15cm.
  //sirina: 30cm, dolžina: 36cm robota.

  geometry_msgs::Twist twist;

  vector<float>sinx;
  vector<float>cosx;
  vector<float> razdalje;
  vector<int> koti;

  for(int i=0;i<scan->ranges.size();i++){

    if(scan->ranges[i] <= 0.75){
      razdalje.push_back(scan->ranges[i]);
      koti.push_back(i);
    }
  }

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

  razdaljeMedTockami.push_back(sqrt( pow((tocke[tocke.size()-1].x - tocke[0].x ),2) + pow((tocke[tocke.size()-1].y - tocke[0].y),2) ));

  if(razdaljeMedTockami[0] > 0.40){

    Tocka S;//razpoloviscna tocka
    S.x = ((tocke[0].x + tocke[tocke.size()-1].x)/2);
    S.y = ((tocke[0].y + tocke[tocke.size()-1].y)/2);
    //kot
    S.kotA = tocke[0].kot;
    S.kotB = tocke[tocke.size()-1].kot;
    int tmp = ((S.kotA + S.kotB)/2)-180;
    if (tmp < 0 ) tmp+= 359;

    S.kot = tmp;
    S.razdaljaMedTockama = razdaljeMedTockami[razdaljeMedTockami.size()-1];
    moznaRazpolovisca.push_back(S);
  }

    //izracun oddaljenosti posameznih tock med seboj
    for(int i=0;i<tocke.size()-1;i++){
      float tmp = sqrt( pow((tocke[i+1].x - tocke[i].x ),2) + pow((tocke[i+1].y - tocke[i].y),2) );
      razdaljeMedTockami.push_back(tmp);
    }


    //preverimo ce je razdalja dovolj velika za robota
    for(int i=0;i<razdaljeMedTockami.size();i++){

      if(razdaljeMedTockami[i] > 0.45){

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
      for(int i=0;i<moznaRazpolovisca.size();i++){
       ROS_INFO("kotA: %f  (kot): %f  kotB: %f",moznaRazpolovisca[i].kotA,moznaRazpolovisca[i].kot,moznaRazpolovisca[i].kotB);
      }
      for(int i=0;i<moznaRazpolovisca.size();i++){

        if(moznaRazpolovisca[i].razdaljaMedTockama > max){
          max = moznaRazpolovisca[i].razdaljaMedTockama;
          T = moznaRazpolovisca[i];
        }
      }


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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "picobot_auto");
  PicobotAuto picobot_auto;
  ros::spin();
  return 0;
}

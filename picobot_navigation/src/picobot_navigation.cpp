#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "sensor_msgs/LaserScan.h"
#include <vector>
#include <math.h>
#include <iostream>
#include <unistd.h>
#include <limits>

using namespace std;

bool aliJeKajPredNami;

class PicobotNavigation{

public:
  PicobotNavigation();

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
  float gyroYaw;
};

PicobotNavigation::PicobotNavigation(){

  sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, &PicobotNavigation::scanCallback,this);
  pub = n.advertise<geometry_msgs::Twist>("pico/cmd_vel", 1);
}

int PicobotNavigation::stopinjaTan(int stopinja){

  if (stopinja >= 0 && stopinja <= 90) {
    return 90-stopinja;
  }else if (stopinja > 90 && stopinja <= 180){
    return 90-stopinja;
  }else if (stopinja > 180 && stopinja <= 270){
    return 270-stopinja;
  }else if (stopinja > 270 && stopinja <= 359){
    return 270-stopinja;
  }
  return 0;
}

void PicobotNavigation::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){

  //Zaznavanje na min 15cm.
  //Širina: 30cm, dolžina: 36cm robota.

  double inf = std::numeric_limits<double>::infinity();

  geometry_msgs::Twist twist;
  n.getParam("/gyroYaw",gyroYaw);

  vector<float>sinx;
  vector<float>cosx;

  vector<float> razdalje;
  vector<int> koti;
  //shranimo razdalje od 30cm-50cm
  for(int i=0;i<scan->ranges.size();i++){

    if(scan->ranges[i] <= 0.8){// && scan->ranges[i] != inf){
      razdalje.push_back(scan->ranges[i]);
      // koti.push_back(stopinjaTan(i));
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

  //kot pred robotom
  razdaljeMedTockami.push_back(sqrt( pow((tocke[tocke.size()-1].x - tocke[0].x ),2) + pow((tocke[tocke.size()-1].y - tocke[0].y),2) ));
  // ROS_INFO("razdaljaMedTockama (velikost): %i",razdaljeMedTockami.size());
  ROS_INFO("razdaljaMedTockama: %f",razdaljeMedTockami[razdaljeMedTockami.size()-1]);


  if(razdaljeMedTockami[razdaljeMedTockami.size()-1] > 0.40 ){

    Tocka S;//razpoloviscna tocka
    S.x = ((tocke[0].x + tocke[tocke.size()-1].x)/2);
    S.y = ((tocke[0].y + tocke[tocke.size()-1].y)/2);
    //kot
    S.kotA = tocke[0].kot;
    S.kotB = tocke[tocke.size()-1].kot;
    int tmp = ((S.kotA +S.kotB)/2)-180;
    if (tmp < 0 ) tmp+= 359;

    S.kot = tmp; //atan(S.x/S.y) * (180.0/M_PI);
    S.razdaljaMedTockama = razdaljeMedTockami[razdaljeMedTockami.size()-1];
    moznaRazpolovisca.push_back(S);

    aliJeKajPredNami = false;
  }else{
    aliJeKajPredNami = true;
   // ROS_INFO("razdaljaMedTockama: %f",razdaljeMedTockami[razdaljeMedTockami.size()-1]);
  }

  // ROS_INFO("razdaljaMedTockama: %f",razdaljeMedTockami[razdaljeMedTockami.size()-1]);

  if (aliJeKajPredNami == false){
    //gremo naprej
    twist.linear.x = 0.3;
    twist.angular.z = 0;
    pub.publish(twist);
  }

  if (aliJeKajPredNami){//gledamo ostale kote

    //izracun oddaljenosti posameznih tock med seboj
    for(int i=0;i<tocke.size()-1;i++){
      float tmp = sqrt( pow((tocke[i+1].x - tocke[i].x ),2) + pow((tocke[i+1].y - tocke[i].y),2) );
      razdaljeMedTockami.push_back(tmp);
    }


    //preverimo ce je razdalja dovolj velika za robota
    for(int i=0;i<razdaljeMedTockami.size();i++){

      if(razdaljeMedTockami[i] > 0.40){

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

    int vmesniKot = (int)T.kot;
    int skupaj = vmesniKot+gyroYaw;
    if (skupaj > 359){
      skupaj-=359;
    }

    while(aliJeKajPredNami){//ce je kaj pred robotom se obrne v drugo smer
      int gyr;
      n.getParam("/gyroYaw",gyr);
      ROS_INFO("kot %i skupaj: %i gyro: %i",vmesniKot,skupaj,gyr);
      if (vmesniKot <= 359 && vmesniKot >= 180){
        twist.angular.z = -0.6;
      }else{
        twist.angular.z = 0.6;
      }
      twist.linear.x = 0;
      pub.publish(twist);
      if (skupaj == gyr || skupaj+1 == gyr || skupaj+2 == gyr || skupaj-1 == gyr || skupaj-2 == gyr){
        ROS_INFO("Prava smer");
        twist.linear.x = 0;
        twist.angular.z = 0; 
        pub.publish(twist);
        break;
      }
    }
    //ROS_INFO("kotA: %f  (kot): %i  kotB: %f gyro: %f",T.kotA,vmesniKot,T.kotB,gyroYaw);
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
  ros::init(argc, argv, "picobot_navigation");
  PicobotNavigation picobot_navigation;
  ros::spin();
  return 0;
}

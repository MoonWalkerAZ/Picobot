#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <vector>
#include <math.h>
#include <limits>

using namespace std;

vector<float>sinx;
vector<float>cosx;

struct Tocka{
  float x;
  float y;
  float kot;
  float kotA;
  float kotB;
};

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    //Zaznavanje na min 15cm.
    //Širina: 30cm, dolžina: 36cm robota.
double inf = std::numeric_limits<double>::infinity();

//ROS_INFO(": [kot 270, razdalja %f]", scan->ranges[270]);


  vector<float> razdalje;
  vector<int> koti;
  //shranimo razdalje od 30cm-50cm
  for(int i=0;i<scan->ranges.size();i++){

    if(scan->ranges[i] <= 0.5){// && scan->ranges[i] != inf){
      razdalje.push_back(scan->ranges[i]);
      koti.push_back(i);
    }
  }
//ROS_INFO("velikost: %i",razdalje.size());

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
// ROS_INFO("velikost: %i",tocke.size());

  vector<float> razdaljeMedTockami;
  //izracun oddaljenosti posameznih tock med seboj
  for(int i=0;i<tocke.size()-1;i++){
    float tmp = sqrt( pow((tocke[i+1].x - tocke[i].x ),2) + pow((tocke[i+1].y - tocke[i].y),2) );
    razdaljeMedTockami.push_back(tmp);
  }
//ROS_INFO("velikost: %i",razdaljeMedTockami.size());
  //preverimo ce je razdalja dovolj velika za robota
  vector<Tocka>moznaRazpolovisca;
  for(int i=0;i<razdaljeMedTockami.size();i++){

    if(razdaljeMedTockami[i] > 0.40){
      Tocka S;//razpoloviscna tocka
      S.x = ((tocke[i].x + tocke[i+1].x)/2);
      S.y = ((tocke[i].y + tocke[i+1].y)/2);
      //kot
      S.kotA = tocke[i].kot;
      S.kotB = tocke[i+1].kot;
      S.kot = (tocke[i].kot + tocke[i+1].kot) /2;//atan(S.x/S.y) * (180.0/M_PI);
      moznaRazpolovisca.push_back(S);
    }
  }

//ROS_INFO("velikost: %i",moznaRazpolovisca.size());
if (moznaRazpolovisca.size() > 0){
 ROS_INFO("START"); 
 for(int i=0;i<moznaRazpolovisca.size();i++){
    ROS_INFO("kotA: %f  (kot): %f  kotB: %f",moznaRazpolovisca[i].kotA,moznaRazpolovisca[i].kot,moznaRazpolovisca[i].kotB);
  }
 ROS_INFO("STOP");
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
    ros::NodeHandle n;

 /*   for(int i=0;i<360;i++){

      sinx.push_back(sin(i * M_PI / 180));//stopinje v radiane
      cosx.push_back(cos (i * M_PI / 180));
    }*/

    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);

    ros::spin();

    return 0;
}

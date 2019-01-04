#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <vector>
#include <math.h>

using namespace std;

vector<float>sinx;
vector<float>cosx;

struct Tocka{
  float x;
  float y;
  float kot;
};

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    //Zaznavanje na min 15cm.
    //Širina: 30cm, dolžina: 36cm robota.

  vector<float> razdalje;
  vector<int> koti;
  //shranimo razdalje od 30cm-50cm
  for(int i=0;i<scan->ranges.size();i++){

    if(scan->ranges[i] > 30 && scan->ranges[i] < 50){
      razdalje.push_back(scan->ranges[i]);
      koti.push_back(i);
    }
  }

  for(int i=0;i<koti.size();i++){

    sinx.push_back(sin(koti[i] * M_PI / 180));//stopinje v radiane
    cosx.push_back(cos (koti[i] * M_PI / 180));
  }

  //izračun x,y
  vector<Tocka> tocke;
  for(int i=0;i<razdalje.size();i++){
    tocke[i].x = razdalje[i] * cosx[i];
    tocke[i].y = razdalje[i] * sinx[i];
    tocke[i].kot = koti[i];
  }

  vector<float> razdaljeMedTockami;
  //izracun oddaljenosti posameznih tock med seboj
  for(int i=0;i<tocke.size()-1;i++){
    razdaljeMedTockami[i] = sqrt( pow((tocke[i+1].x - tocke[i].x ),2) + pow((tocke[i+1].y - tocke[i].y),2) );
  }

  //preverimo ce je razdalja dovolj velika za robota
  vector<Tocka>moznaRazpolovisca;
  for(int i=0;i<razdaljeMedTockami.size();i++){

    if(razdaljeMedTockami[i] > 30){
      Tocka S;//razpoloviscna tocka
      S.x = ((tocke[i].x + tocke[i+1].x)/2);
      S.y = ((tocke[i].y + tocke[i+1].y)/2);
      //kot
      S.kot = (tocke[i].kot + tocke[i+1].kot)/2;
      moznaRazpolovisca.push_back(S);
    }
  }

  for(int i=0;i<moznaRazpolovisca.size();i++){
    ROS_INFO("mozne poti (kot): %f",moznaRazpolovisca[i].kot);
  }

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

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#define RAD2DEG(x) ((x)*180./M_PI)

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    //zaznavanje na min 15cm
    //scan->ranges.size(); 360 velikost
    //ROS_INFO(": [kot %f, razdalja %f]", RAD2DEG(scan->angle_min + scan->angle_increment * 90), scan->ranges[90]);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "picobot_navigation");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);

    ros::spin();

    return 0;
}

/*
 * Copyright (c) 2014, RoboPeak
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 *    this list of conditions and the following disclaimer in the documentation 
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR 
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/*
 *  RoboPeak LIDAR System
 *  RPlidar ROS Node client test app
 *
 *  Copyright 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 * 
 */


#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#define RAD2DEG(x) ((x)*180./M_PI)

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    //scan->ranges.size(); 360 velikost
    //ROS_INFO("Smo na 0 stopinjah: %f ",scan->ranges[0]);
    //ROS_INFO(": [kot %f, razdalja %f]", RAD2DEG(scan->angle_min + scan->angle_increment * 90), scan->ranges[90]);

    //desno spredaj 270-359 zaznavanje na 0.15 min
    for(int i = 270;i < 360;i++){
      //int degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
      //ROS_INFO("Zaznan predmet desno spredaj na razdalji %f pod kotom %f",scan->ranges[i],degree);
      if (scan->ranges[i] < 0.20){
        ROS_INFO("Dejanska blizina (manj od 20cm): %f",scan->ranges[i]);
      }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rplidar_node_client");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);

    ros::spin();

    return 0;
}

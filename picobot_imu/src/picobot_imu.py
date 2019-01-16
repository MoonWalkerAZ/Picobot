#!/usr/bin/env python
import rospy
from sense_hat import SenseHat, ACTION_PRESSED, ACTION_HELD, ACTION_RELEASED
import time
from random import randint
from std_msgs.msg import *

sense = SenseHat()

def pushed_up(event):
 sense.low_light = True
 g =(0,153,153)
 bl = (0,153,0)
 b = (0,0,0)
 picobot_pixels = [
 g,g,b,g,b,g,g,b,
 g,g,b,g,b,g,b,b,
 g,b,b,g,b,g,g,b,
 b,b,b,b,b,b,b,b,
 g,g,g,b,bl,b,bl,b,
 g,b,g,b,b,bl,b,b,
 g,g,g,b,bl,bl,bl,b,
 b,b,b,b,b,b,b,b
 ]
 sense.set_pixels(picobot_pixels)

def pushed_down(event):
 sense.clear((0, 0, 0))

def randomColor():
 red_random = randint(0,255)
 blue_random = randint(0,255)
 green_random = randint(0,255)
 return (red_random,green_random,blue_random)


def gyroData():
    pub = rospy.Publisher('gyro', Int32, queue_size=10)
    
    rospy.init_node('picobot_imu')
    rate = rospy.Rate(10) # 10hz

    sense.set_imu_config(False, True, False)
    
    while not rospy.is_shutdown():
        sense.stick.direction_up = pushed_up
        sense.stick.direction_down = pushed_down

        o = sense.get_orientation()
        #pitch = o["pitch"]
        #roll = o["roll"]
        yaw = (o["yaw"]-359)*-1
        rospy.set_param('gyroYaw', yaw)
        #rospy.loginfo(yaw)
        pub.publish(int(yaw))
        
        rate.sleep()

def main():
    
    sense.clear((0, 0, 0))
    sense.low_light = True
    g =(0,153,153)
    bl = (0,153,0)
    b = (0,0,0)
    picobot_pixels = [
    g,g,b,g,b,g,g,b,
    g,g,b,g,b,g,b,b,
    g,b,b,g,b,g,g,b,
    b,b,b,b,b,b,b,b,
    g,g,g,b,bl,b,bl,b,
    g,b,g,b,b,bl,b,b,
    g,g,g,b,bl,bl,bl,b,
    b,b,b,b,b,b,b,b
    ]
    sense.set_pixels(picobot_pixels)
    gyroData()
if __name__ == '__main__':
     main()


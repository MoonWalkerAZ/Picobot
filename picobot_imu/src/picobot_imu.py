#!/usr/bin/env python
import rospy
from sense_hat import SenseHat
import time
from random import randint
from std_msgs.msg import Float32

sense = SenseHat()

def randomColor():
 red_random = randint(0,255)
 blue_random = randint(0,255)
 green_random = randint(0,255)
 return (red_random,green_random,blue_random)


def gyroData():
    #pub = rospy.Publisher('gyro', Float32, queue_size=10)
    
    rospy.init_node('picobot_imu')
    rate = rospy.Rate(10) # 10hz

    sense.set_imu_config(False, True, False)
    
    while not rospy.is_shutdown():
        o = sense.get_orientation()
        #pitch = o["pitch"]
        #roll = o["roll"]
        yaw = o["yaw"]
        rospy.set_param('gyroYaw', o["yaw"])
        #rospy.loginfo(yaw)
        #pub.publish(yaw)
        
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


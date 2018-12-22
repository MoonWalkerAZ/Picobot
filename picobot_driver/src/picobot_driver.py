#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import RPi.GPIO as GPIO  # Import the GPIO Library
import time

# Set the GPIO modes
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
#test sprememb
_FREQUENCY = 20 # konstanta, za notranjo uporabo

def _porezi(value,min,max):
    """Zagotovimo da je vrednost med 0(min) in 100(max) """
    if value < min:
       return min
    elif value > max:
       return max
    return value

class Motor:

   def __init__(self, forward_pin, backward_pin, R_EN, L_EN):
      # self._forward_pin = forward_pin
       #self._backward_pin = backward_pin

       GPIO.setup(forward_pin, GPIO.OUT)
       GPIO.setup(backward_pin, GPIO.OUT)
       GPIO.setup(R_EN, GPIO.OUT)
       GPIO.setup(L_EN, GPIO.OUT)
       GPIO.output(R_EN, True)
       GPIO.output(L_EN, True)

       self._forward_pwm = GPIO.PWM(forward_pin, _FREQUENCY)
       self._backward_pwm = GPIO.PWM(backward_pin, _FREQUENCY)

   def move(self, speed_percent):

        speed = _porezi(abs(speed_percent),0,100)

        #Pozitivno stevilo premakne kolesa naprej, negativno pa nazaj
        if speed_percent < 0:
           self._backward_pwm.start(speed)
           self._forward_pwm.start(0)
        else:
           self._forward_pwm.start(speed)
           self._backward_pwm.start(0)

class Driver:

  def __init__(self):

    rospy.init_node('picobot_driver')
    self._last_received = rospy.get_time()
    self._timeout = rospy.get_param('~timeout', 5)
    self._max_speed = rospy.get_param('~max_speed', 0.1)#potrebujemo za nadzor angularne hitrosti
    self._wheel_base = rospy.get_param('~wheel_base', 0.091)

    # Nastavimo pine za motorje
    self.R_EN_levi = 5
    self.L_EN_levi = 6
    self._left_motor = Motor(13, 19, self.R_EN_levi, self.L_EN_levi)
    
    self.R_EN_desni = 9
    self.L_EN_desni = 11
    self._right_motor = Motor(10, 7, self.R_EN_desni, self.L_EN_desni)
   
    self._left_speed_percent = 0
    self._right_speed_percent = 0

    # Subscriber za twist sporocila
    rospy.Subscriber('/pico/cmd_vel',Twist, self.velocity_received_callback)


  def velocity_received_callback(self, message):
      """Obdelamo ukaze za hitrost"""
      self._last_received = rospy.get_time()

       # Dobimo linearne in angularne hotrosti
      linear = message.linear.x
      angular = message.angular.z

       # Izracunamo vrtenje koles v m/s
      left_speed = (angular/self._max_speed) * self._wheel_base/2 - linear
      right_speed = (angular/self._max_speed)* self._wheel_base/2 + linear
      self._left_speed_percent = (100 * left_speed)
      self._right_speed_percent = (100 * right_speed)

  def run(self):
     """ Loop driver-ja """

     rate = rospy.Rate(10)#10 Hz

     while not rospy.is_shutdown():
     # Ce dolgo ne dobim ukazov potem smo izgubili povezavo
          delay = rospy.get_time() - self._last_received
          if delay < self._timeout:
             self._left_motor.move(self._left_speed_percent)
             self._right_motor.move(self._right_speed_percent)
          else:
             self._left_motor.move(0)
             self._right_motor.move(0)
          rate.sleep() 


def main():
    driver = Driver()
    driver.run()

if __name__ == '__main__':
     main()

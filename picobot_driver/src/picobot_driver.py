#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import RPi.GPIO as GPIO
import time

#GPIO pini (BCM)
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

frekvenca = 120 # konstanta, za notranjo uporabo

class Motor:

   def __init__(self, naprejPin, nazajPin, R_EN, L_EN):

       GPIO.setup(naprejPin, GPIO.OUT)
       GPIO.setup(nazajPin, GPIO.OUT)
       GPIO.setup(R_EN, GPIO.OUT)
       GPIO.setup(L_EN, GPIO.OUT)
       GPIO.output(R_EN, True)
       GPIO.output(L_EN, True)

       self.naprejPwm = GPIO.PWM(naprejPin, frekvenca)
       self.nazajPwm = GPIO.PWM(nazajPin, frekvenca)

   def porezi(val,min,max):
    """Zagotovimo da je vrednost med 0(min) in 100(max) """
    if val < min:
       return min
    elif val > max:
       return max
    return val

   def premik(self, hitrostOdst):

        hitrost = porezi(abs(hitrostOdst),0,100)

        #Pozitivno stevilo premakne kolesa naprej, negativno pa nazaj
        if hitrostOdst < 0:
           self.nazajPwm.start(hitrost)
           self.naprejPwm.start(0)
        else:
           self.naprejPwm.start(hitrost)
           self.nazajPwm.start(0)

class Driver:

  def __init__(self):

    rospy.init_node('picobot_driver')
    self.nazadnjeSprejel = rospy.get_time()
    self.premor =  10
    self.maxHitorst =  0.1#potrebujemo za nadzor angularne hitrosti
    self.osKolesa =  0.091

    # Nastavimo pine za motorje
    self.R_EN_levi = 5
    self.L_EN_levi = 6
    self.leviMotor = Motor(13, 19, self.R_EN_levi, self.L_EN_levi)
    
    self.R_EN_desni = 9
    self.L_EN_desni = 11
    self.desniMotor = Motor(10, 7, self.R_EN_desni, self.L_EN_desni)
   
    self.levaHitrostProcent = 0
    self.desnaHitrostProcent = 0

    # Subscriber za twist sporocila
    rospy.Subscriber('/pico/cmd_vel',Twist, self.velocity_received_callback)


  def velocity_received_callback(self, message):
      """Obdelamo ukaze za hitrost"""
      self.nazadnjeSprejel = rospy.get_time()

       # Dobimo linearne in angularne hotrosti
      linear = message.linear.x
      angular = message.angular.z

       # Izracunamo vrtenje koles v m/s
      levaHitrost = (angular/self.maxHitorst) * self.osKolesa/2 - linear
      desnaHitrost = (angular/self.maxHitorst)* self.osKolesa/2 + linear
      self.levaHitrostProcent = (100 * levaHitrost)
      self.desnaHitrostProcent = (100 * desnaHitrost)

  def run(self):
     """ Loop driver-ja """

     rate = rospy.Rate(10)#10 Hz

     while not rospy.is_shutdown():
     # Ce dolgo ne dobim ukazov potem smo izgubili povezavo to je varovalo
          delay = rospy.get_time() - self.nazadnjeSprejel
          if delay < self.premor:
             self.leviMotor.premik(self.levaHitrostProcent)
             self.desniMotor.premik(self.desnaHitrostProcent)
          else:
             self.leviMotor.premik(0)
             self.desniMotor.premik(0)
          rate.sleep() 

def main():
    driver = Driver()
    driver.run()

if __name__ == '__main__':
     main()

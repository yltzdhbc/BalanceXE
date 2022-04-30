# coding=utf-8

#  BROBOT STEP CONSTANTS
#  STEPSMETER = 11428; STEPS/METERS      default: 11428
#  STEPSTURN = 4720; STEPS/TURN (360 deg) default:4720

# turn left 90  BROBOT MOVE 50 -1180 1180
# turn right 45 BROBOT MOVE 50 590 -590
# turn right 90 BROBOT MOVE 50 1180 -1180
# turn right 360 BROBOT MOVE 50 4720 -4720

# move forward 100cm BROBOT MOVE 50 11428 11428

import time
import thread
from robot import ROBOT

NormalMode = 0
ProMode = 1
LEFT = 0
RIGHT = 1
FRONT = 0
BACK = 1

myRobot1 = ROBOT()
myRobot1.UDP_IP = "192.168.1.201" #设置机器人的ip地址
myRobot1.mode(NormalMode)


angle = 0

while(angle <= 90):
   angle +=1
   myRobot1.Circular_Arc(0.01, LEFT, angle, 0.2)
   print "angle:", angle
   time.sleep(0.005)

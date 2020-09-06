#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
#robot measurements:
#width-
#height-
#length-
#attachment 1 measurments-
#...
#_______Variables_______
#variables to move forward and backward

#medium motor varibles

#counters
x=0
#____________Functions_____________
#basic functions
    def oneFootBlackLine():
        while x < 10:
            robot.DriveBase(100)
            x += 1 
    def turnright():


#button functions
    def rightButton():
        oneFootBlackLine()
        rightturn()
    def leftButton():


#main function
def main():
    while (True):
        if rightbutton.pressed:
            rightButton()


main()
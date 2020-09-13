#!/usr/bin/env pybricks-micropython

"""
Example LEGO® MINDSTORMS® EV3 Robot Educator Color Sensor Down Program
----------------------------------------------------------------------

This program requires LEGO® EV3 MicroPython v2.0.
Download: https://education.lego.com/en-us/support/mindstorms-ev3/python-for-ev3

Building instructions can be found at:
https://education.lego.com/en-us/support/mindstorms-ev3/building-instructions#robot
"""

from pybricks.ev3devices import Motor, ColorSensor, GyroSensor
from pybricks.parameters import Port , Direction
from pybricks.tools import wait
from pybricks.robotics import DriveBase
import Dimensions
# Initialize the motors.
left_motor = Motor(Port.B , positive_direction=Direction.COUNTERCLOCKWISE, gears=[40,8])
right_motor = Motor(Port.C , positive_direction=Direction.COUNTERCLOCKWISE, gears=[40,8])
gyro_sensor = GyroSensor(Port.S3)
# Initialize the color sensor.
#line_sensor = ColorSensor(Port.S3)
left_motor.reset_angle(0)
right_motor.reset_angle(0)
gyro_sensor.reset_angle(0)

fudge=1
speed=100
angle=0


robot = DriveBase(left_motor, right_motor, wheel_diameter=30, axle_track=135)
initial_distance = 0
robot.reset()
while ((robot.distance() - initial_distance) < 350) :
    robot.drive(speed,angle)
    drift = gyro_sensor.angle()
    angle = (drift * fudge) * -1      
wait(3) 
i=0
robot.stop()
speed=50
while (i<2 ):
    fruit = gyro_sensor.angle()
    fruit = fruit * (-1)
    robot.drive(speed,fruit)
    wait(3)
    robot.drive((speed* (-1)),fruit) 
    i = i + 1

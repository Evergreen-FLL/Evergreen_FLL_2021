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
#medium_motor = Motor(Port.D ,positive_direction=Direction.CLOCKWISE )
gyro_sensor = GyroSensor(Port.S3)
#medium_motor.run_angle(90)
# Initialize the color sensor.
line_sensor = ColorSensor(Port.S2)

left_motor.reset_angle(0)
right_motor.reset_angle(0)
gyro_sensor.reset_angle(0)


fudge=1
speed=100
angle=0


robot = DriveBase(left_motor, right_motor, wheel_diameter=30, axle_track=135)
left_motor.dc(90)
right_motor.dc(90)
""" initial_distance = 0
robot.reset()
while ((robot.distance() - initial_distance) < 350) :
     drift = gyro_sensor.angle()
     print(gyro_sensor.angle())
     angle = (drift * fudge) * -1      
wait(10) 
i=0
speed=200
 """





# Calculate the light threshold. Choose values based on your measurements.
BLACK = 8
WHITE = 85
threshold = 47

# Set the drive speed at 100 millimeters per second.
DRIVE_SPEED = 100

# Set the gain of the proportional line controller. This means that for every
# percentage point of light deviating from the threshold, we set the turn
# rate of the drivebase to 1.2 degrees per second.

# For example, if the light value deviates from the threshold by 10, the robot
# steers at 10*1.2 = 12 degrees per second.
PROPORTIONAL_GAIN = 3.5
drive_motor_power_left = 87 *0.4
drive_motor_power_right =87 *0.4

i=0
# Start following the line endlessly.
while (i<10000):
    # Calculate the deviation from the threshold.
    deviation = line_sensor.reflection() - threshold
        # Calculate the turn rate.
    turn_rate = (PROPORTIONAL_GAIN * deviation)
   
    drive_motor_power_left = (87 + turn_rate) *0.4
    drive_motor_power_right = (87 - turn_rate) *0.4
    left_motor.dc(drive_motor_power_left)
    right_motor.dc(drive_motor_power_right)
    print ("color", line_sensor.reflection(),"Turn", turn_rate, "Lpower", drive_motor_power_left,"Rpower", drive_motor_power_right)
    # Set the drive base speed and turn rate.
    #robot.drive(DRIVE_SPEED, turn_rate)

    i=i+1
    # You can wait for a short time or do other things in this loop.
   


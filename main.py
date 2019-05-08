#!/usr/bin/env pybricks-micropython

from pybricks import ev3brick as brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import (Port, Stop, Direction, Button, Color,
                                 SoundFile, ImageFile, Align)
from pybricks.tools import print, wait, StopWatch
from pybricks.robotics import DriveBase

motor_grabber = Motor(Port.D)
motor_l = Motor(Port.B)
motor_r = Motor(Port.C)
robot = DriveBase(motor_l, motor_r, 56, 114) # wheel diameter (mm) | axle track (mm)
color_sensor = ColorSensor(Port.S1)
#ultrasonic = UltrasonicSensor(Port.S4)

collision_threshhold = 400  # mm
drive_speed = 100           # mm/s
rotation_speed = 45         # deg/s
calibration_surface = -1
myColor = -1                # color to collect

#don't crash into obstacles
def collision_avoidance():
    dist = ultrasonic.distance()
    if dist < collision_threshhold:
        robot.drive_time(0, 45, 500)
        collision_avoidance() # check if collision free

#grab cubes
def close_grabber():
    motor_grabber.run_target(5, 110, Stop.HOLD, False)
    returnToHome()

#navigate home
def returnToHome():
    robo.drive_time(drive_speed,180,1000)

#check color of cube and start grabbing or move away
def check_color():
    if color_sensor.color() != calibration_surface:
        robot.stop()
        if color_sensor.color() == myColor:
            close_grabber()
        else:
            #move away from object
            robot.drive_time((-1*drive_speed), 0, 1000)

myColor = color_sensor.color # set my color to collect
#TODO Save homeloc and route home
while True:

    collision_avoidance()
    check_ball()
    robot.drive_time(drive_speed, 90, 1000)
    brick.display.clear()
    robot.drive(drive_speed, 0)
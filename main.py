#!/usr/bin/env pybricks-micropython

from pybricks import ev3brick as brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import (Port, Stop, Direction, Button, Color,
                                 SoundFile, ImageFile, Align)
from pybricks.tools import print, wait, StopWatch
from pybricks.robotics import DriveBase

import random

motor_grabber = Motor(Port.D)
motor_l = Motor(Port.B)
motor_r = Motor(Port.C)
robot = DriveBase(motor_l, motor_r, 56, 114) # wheel diameter (mm) | axle track (mm)
color_sensor = ColorSensor(Port.S1)
ultrasonic = UltrasonicSensor(Port.S4)

collision_threshhold = 100  # mm
drive_speed = 80            # mm/s
rotation_speed = 45         # deg/s
myColor = -1                # color to collect
dropZoneColor = -1          # color of surface to drop cubes
calibration_surface = -1

# avoid objects and other mindstorms
def collision_avoidance():
    dist = ultrasonic.distance()
    if dist < collision_threshhold:

        robot.stop()
        # turn left or right
        rand = random.randint(0, 1)
        if rand == 0:
            robot.drive_time(40, -90, 1400)
        else:
            robot.drive_time(40, 90, 1400)

        # check again after turning
        collision_avoidance()

        # drive away from object
        robot.drive_time(drive_speed, 0, 1000)

# close the grabber to collect an item
def close_grabber():
    motor_grabber.run_until_stalled(100, Stop.HOLD)

# open the grabber to drop an item
def open_grabber():
    motor_grabber.run_until_stalled(-100, Stop.HOLD)

def make_turn():
    turn = random.randint(0, 50)
    if turn == 0:
        # decide left or right turn
        rand = random.randint(0,1)
        if rand == 0:
            robot.drive_time(drive_speed, 90, random.randint(400,1500))
        else:
            robot.drive_time(drive_speed, -90, random.randint(400,1500))

# initialize color sensor
myColor = Color.BLACK       # color of zone to collect items
dropZoneColor = Color.RED   # color of zone to drop items

# initialize state of grabber
loaded = False
open_grabber()

# main routine
while True:

    # search for pick-up zone and grab an item
    while loaded == False:
        # drive around and look for pick-up zone
        while True:
            collision_avoidance()
            if color_sensor.color() != calibration_surface:
                robot.stop()
                    # if zone is detacted stop driving around
                    if color_sensor.color() == myColor:
                        break

            # start/continue driving ..
            robot.drive(drive_speed, 0)
            wait(100)

            # .. eventually make a turn
            make_turn()

        # stop and grab an item
        robot.stop()
        close_grabber()

        # move away from pick-up zone
        robot.drive_time((-1*drive_speed), 0, 4000)
        robot.drive_time(50, -90, 2000)
        loaded = True
    
    # search for drop zone and drop the item
    while loaded == True:
        # drive around and look for drop zone
        while True:
            collision_avoidance()
            if color_sensor.color() != calibration_surface:
                robot.stop()
                # if zone is detacted stop driving around
                    if color_sensor.color() == dropZoneColor:
                        break

            # start/continue driving ..
            robot.drive(drive_speed, 0)
            wait(100)

            # .. eventually make a turn
            make_turn()
        
        # stop and drop an item
        robot.stop()
        open_grabber()

        # move away from drop zone
        robot.drive_time((-1*drive_speed), 0, 4000)
        robot.drive_time(50, -90, 2000)
        loaded = False
        
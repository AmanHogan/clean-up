#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, ColorSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Color
from pybricks.media.ev3dev import SoundFile, ImageFile, Image
from robotics import Robot, Navigator
from pybricks.iodevices import I2CDevice
from globals import *
from logger import log

notInGoal = True

# Initialize the EV3 Brick.
ev3 = EV3Brick()

# Define Sensors
rMotor = Motor(Port.D, positive_direction=Direction.COUNTERCLOCKWISE)
lMotor = Motor(Port.A, positive_direction=Direction.COUNTERCLOCKWISE)
colorSesor = ColorSensor(Port.S4)
ultrasonicSensor = UltrasonicSensor(Port.S1)
infraredSensor = I2CDevice(Port.S3, 0x01)
gyroSensor = GyroSensor(Port.S2, Direction.COUNTERCLOCKWISE)
nav = Navigator(gyroSensor)
robot = Robot(lMotor, rMotor, nav, colorSesor , ultrasonicSensor, infraredSensor, ev3)

log("================================== Starting Robot ==================================")

while robot.notInGoal:
	robot.update_sensors()
	robot.update_queue()
	robot.process_behavior()





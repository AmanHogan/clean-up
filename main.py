#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor, UltrasonicSensor)
from pybricks.parameters import Port, Stop, Direction, Color
from pybricks.media.ev3dev import SoundFile, ImageFile
from robotics import Robot, Navigator
from pybricks.iodevices import I2CDevice
from globals import *

notInGoal = True

# Initialize the EV3 Brick.
ev3 = EV3Brick()
ev3.speaker.set_speech_options(voice='f3')


# Define Sensors
rMotor = Motor(Port.D, positive_direction=Direction.COUNTERCLOCKWISE)
lMotor = Motor(Port.A, positive_direction=Direction.COUNTERCLOCKWISE)
nav = Navigator()
colorSesor = ColorSensor(Port.S4)
ultrasonicSensor = UltrasonicSensor(Port.S2)
infraredSensor = I2CDevice(Port.S2, 0x01)

robot = Robot(lMotor, rMotor, nav, colorSesor , ultrasonicSensor, infraredSensor)

while notInGoal:
	robot.update_sensors()
	robot.update_queue()
	robot.process_behavior()

while True:
		
	direction = int.from_bytes(irSensor.read(0x42, length=1), "little")

	strengths = []
	for i in range(5):
		strengths.append(int.from_bytes(irSensor.read(0x43+i, length=1), "little"))

	print(direction)
	print(strengths)


# Initialize sensors and motors
# rMotor = Motor(Port.D, positive_direction=Direction.COUNTERCLOCKWISE)
# lMotor = Motor(Port.A, positive_direction=Direction.COUNTERCLOCKWISE)
# nav = Navigator()
# fSensor = TouchSensor(Port.S3)
# lSensor = TouchSensor(Port.S1)
# cSensor = ColorSensor(Port.S4)
# uSensor = UltrasonicSensor(Port.S2)

# robot = Robot(lMotor,rMotor, nav, fSensor, lSensor, cSensor ,uSensor)




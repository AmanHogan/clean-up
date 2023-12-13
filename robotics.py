"""Acts as a replacement for pybricks robotics module. Module is
responsible for moving the robot, angle tracking, and robot behavior proccesing"""

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor, UltrasonicSensor)
from pybricks.parameters import Port, Stop, Direction, Color, ImageFile
from globals import *
from logger import log
import heapq
from behaviors import (RobotBehavior, FindBall, HasBall)
import math

#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from globals import *
from logger import log

class Navigator:
    """Class responsible for keeping track of the robot's logical orientation
    """

    def __init__(self):
        self.orientation = 0 # orientation [deg]
        log("INFO: Current Robot Orientation: " + str(self.orientation) + " degrees...")

    def update_nav(self, angle) -> None:
        """
        Updates the logical orientation of the robot and keeps track of previous positions\n
        Args: angle (int): angle that the robot needs to be be turned by
        """
        self.orientation = (self.orientation + angle) % 360
        

class Robot:
    """
    Custom defined Robot class for the lego ev3 robot. Responsible for moving, turning, and 
    updating the priority queue of the robot.
    """
    def __init__(self, left_motor: Motor, right_motor: Motor, navigator: Navigator, 
                 color: ColorSensor, sonic: UltrasonicSensor, infrared, ev3):
        
        self.left_motor = left_motor # controls left tire
        self.right_motor = right_motor # controls right tire
        self.navigator = navigator # object to keep track of robot's orientation
        self.csensor = color # color sensor
        self.usensor = sonic # ultrasonic sensor
        self.isensor = infrared # IR sensor

        self.opponentImage = ImageFile.DECLINE
        self.teamImage = ImageFile.ACCEPT


        self.tireRPM = TIRE_RPM

        self.queue = [] # priority queue
        heapq.heapify(self.queue)

        self.distanceToWall = sonic.distance() # distance to wall [mm]
        self.current_color = color.color() # Color detected by Color Sensor
        self.hasBall = False # True if robot has ball
        self.isFindingBall = False # True if Robot is finding Ball
        self.notInGoal = True # True if Ball not in our goal

        # max signal from the strengths IR array
        self.mxSignal = max([int.from_bytes(self.isensor.read(0x43+i, length=1), "little") for i in range(5)])
        
        # list of strengths from IR sensor of zones 1,3,5,7 and 9
        self.strengths = [int.from_bytes(self.isensor.read(0x43+i, length=1), "little") for i in range(5)]

        # Boolean vars for knowing which side the robot is on
        self.inTeamGoal = True
        self.onTeamSide = False
        self.onOpponentSide = False
        self.inOpponentGoal = False

        self.ev3 = ev3
        
    def move(self, distance) -> None:
        """
        Moves robot a given distance in [mm]\n
        Args: distance (float): Distance to be traveled [mm]
        """

        # tire rotation angle
        angle =  (((distance)/TIRE_CIRC)*FULL_ROTATION)*DISTANCE_ERROR 
        self.left_motor. run_angle(TIRE_RPM, angle, wait=False)
        self.right_motor.run_angle(TIRE_RPM, angle)
        log("Robot Moved: " + str(distance) + " milimeters...")

    def turn(self, angle) -> None:
        """
        Turns robot a given angle [deg]\n
        Args: angle (float): angle to turn robot [deg]
        """

        # Calculate steering angle
        steering_angle =  (((2*M_PI*ROBOT_RADIUS*(angle/360))/TIRE_CIRC)*FULL_ROTATION)*TURN_ERROR
        log("INFO: Robot supposed to turn: " + str(angle) + " degrees...")

    
        # Turn Tires
        self.left_motor. run_angle(TIRE_RPM, -(steering_angle), wait=False)
        self.right_motor.run_angle(TIRE_RPM, (steering_angle))
        self.navigator.update_nav(steering_angle)
            
        log("INFO: Current Robot Orientation: " + str(self.navigator.orientation) + " degrees...")
        self.update_sensors()
        
    def run(self) -> None:
        """
        The motor accelerates to TIRE_RPM 
        and keeps running at this speed 
        until you give a new command.
        """
        self.left_motor.run(TIRE_RPM)
        self.right_motor.run(TIRE_RPM)

    def run_time(self, mseconds) -> None:
        """
        The motor accelerates to TIRE_RPM 
        and keeps running at this speed 
        until you give a new command.
        """
        self.left_motor.run_time(TIRE_RPM,mseconds)
        self.right_motor.run_time(TIRE_RPM, mseconds)

    def stop(self) -> None:
        """
        Stops the motor and lets it spin freely.
        The motor gradually stops due to friction.
        """
        self.left_motor.brake()
        self.right_motor.brake()

    def process_behavior(self) -> None:
        """
        Proccess a behavior from the priority queue. Pops the highest priority 
        behavior from queue then processes the behavior in RobotBehavior classes.
        """

        behavior = heapq.heappop(self.queue)
        
        if behavior.priority == HAS_BALL:
            log("BEHAVIOR STARTED: Has Ball")
            behavior.run(self)
            log("BEHAVIOR ENDED: Has Ball")

        if behavior.priority == FIND_BALL:
            log("BEHAVIOR STARTED: Find Ball")
            behavior.run(self)
            log("BEHAVIOR ENDED: Find Ball")

    def update_sensors(self) -> None:
        """Function updates all of the robot's sensor values and stores these values in the robot object
        """
        self.distanceToWall = self.usensor.distance()
        self.current_color = self.csensor.color()
        self.mxSignal = max([int.from_bytes(self.isensor.read(0x43+i, length=1), "little") for i in range(5)])
        self.strengths = [int.from_bytes(self.isensor.read(0x43+i, length=1), "little") for i in range(5)]

        log("INFO: Signal Strengths {}".format(str(self.strengths)), True)
        
    def update_queue(self) -> None:
        """
        Updates the priority queue using the robot's sensor values. Defaults to FindBall if
        priority queue is empty.
        """
        if not self.queue:
            if not any(isinstance(behavior, FindBall) for behavior in self.queue):
                self.queue.append(FindBall())

        # If signal is greater than the threshold, start ball finding
        if self.mxSignal > SIGNAL_THRESHOLD:
           if not any(isinstance(behavior, HasBall) for behavior in self.queue):
                self.queue.append(HasBall())
    
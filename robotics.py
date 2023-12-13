"""Acts as a replacement for pybricks robotics module. Module is responsible for moving the robot, angle tracking, and robot behavior proccesing"""

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, ColorSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Color
from globals import *
from logger import log
import heapq
from behaviors import (RobotBehavior, FindBall, HasBall)
from pybricks.media.ev3dev import SoundFile, ImageFile, Image
import math

class Navigator:
    """Class responsible for keeping track of the robot's orientation"""

    def __init__(self, gyroSensor: GyroSensor):
        self.gyro = gyroSensor # gyro sensor
        self.gyro.reset_angle(INIT_ORIENTATION) # setting initial angle of gyro
        self.orientation = self.normalize_angle(self.gyro.angle()) # orientation of robot [deg]
        log("INFO: Current Robot Orientation: " + str(self.orientation) + " degrees...")

    def normalize_angle(self, angle):
        """
        Normalizes the angle given into a range of [-180, 180) degrees
        Args: angle (int): orientation given by gyroscope
        Returns: int : robot orientation in range [-180, 180) degrees
        """

        normalized_angle = angle % 360  # Wrap the angle to [0, 360)
        if normalized_angle > 180:
            normalized_angle -= 360  # Convert to the range of [-180, 180)
        return normalized_angle

class Robot:
    """
    Custom defined Robot class for the lego ev3 robot. Responsible for moving, turning, and 
    updating the priority queue of the robot.
    """
    def __init__(self, left_motor: Motor, right_motor: Motor, navigator: Navigator, color: ColorSensor, sonic: UltrasonicSensor, infrared, ev3):
        
        self.left_motor = left_motor # controls left tire
        self.right_motor = right_motor # controls right tire
        self.navigator = navigator # object to keep track of robot's orientation
        self.csensor = color # color sensor
        self.usensor = sonic # ultrasonic sensor
        self.isensor = infrared # IR sensor

        # Images displayed when finding ball or when it has found a ball
        self.hasBallImage = Image(ImageFile.ANGRY)
        self.findingBallImage = Image(ImageFile.PINCHED_MIDDLE)

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

        self.ev3 = ev3
        
    def move(self, distance) -> None:
        """
        Moves robot a given distance in [mm]\n
        Args: distance (float): Distance to be traveled [mm]
        """
        angle =  (((distance)/TIRE_CIRC)*FULL_ROTATION)*DISTANCE_ERROR 
        self.left_motor. run_angle(self.tireRPM, angle, wait=False)
        self.right_motor.run_angle(self.tireRPM, angle)
        log("Robot Moved: " + str(distance) + " milimeters...")

    def turn(self, angle) -> None:
        """
        Turns robot a given angle [deg]\n
        Args: angle (float): angle to turn robot [deg]
        """

        # Calculate steering angle
        steering_angle =  (((2*M_PI*ROBOT_RADIUS*(angle/360))/TIRE_CIRC)*FULL_ROTATION)*TURN_ERROR
        log("INFO: Calculated Angle to turn: " + str(angle) + " degrees...")

        # Turn Tires
        self.left_motor. run_angle(self.tireRPM, -(steering_angle), wait=False)
        self.right_motor.run_angle(self.tireRPM, (steering_angle))
        self.update_sensors()
        log("INFO: Current Robot Orientation: " + str(self.orientation) + " degrees...")
        
    def run(self) -> None:
        """
        The motor accelerates to TIRE_RPM 
        and keeps running at this speed 
        until you give a new command.
        """
        self.left_motor.run(self.tireRPM)
        self.right_motor.run(self.tireRPM)

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
        """Function updates all of the robot's sensor values and stores these values in the robot object"""
        self.distanceToWall = self.usensor.distance()
        self.current_color = self.csensor.color()
        self.orientation = self.navigator.normalize_angle(self.navigator.gyro.angle()) 
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
    
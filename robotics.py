"""Acts as a replacement for pybricks robotics module. Module is
responsible for moving the robot, angle tracking, and robot behavior proccesing"""

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor, UltrasonicSensor)
from pybricks.parameters import Port, Stop, Direction, Color
from globals import *
from logger import log
import heapq
from behaviors import (RobotBehavior, FindBall, HasBall)
import math

class Navigator:
    """Class responsible for keeping track of the robot's logical orientation
    """
    def __init__(self):
        self.orientation = 0 # orientation [deg]
        self.orientations = [self.orientation] # List of orientations [deg]
        log("Current Robot Orientation: " + str(self.orientation) + " degrees...")

    def update_nav(self, angle) -> None:
        """
        Updates the logical orientation of the robot and keeps track of previous positions\n
        Args: angle (int): angle that the robot needs to be be turned by
        """
        # Update the orientation based on the turn and keep within the range of -180 to 180 degrees
        self.orientation = (self.orientation + angle) % 360
        self.orientations.append(self.orientation)
        

class Robot:
    """Custom defined Robot class for the lego ev3 robot. Responsible for moving and turning the robot.
    """
    def __init__(self, left_motor: Motor, right_motor: Motor, navigator: Navigator, 
                 color: ColorSensor, sonic: UltrasonicSensor, infrared):
        
        self.left_motor = left_motor # controls left tire
        self.right_motor = right_motor # controls right tire
        self.navigator = navigator # object to keep track of robot's orientation
        self.csensor = color # color sensor
        self.usensor = sonic # ultrasonic sensor
        self.isensor = infrared

        self.queue = [] # priority queue
        heapq.heapify(self.queue)

        self.distanceToWall = sonic.distance() # distance to wall [mm]
        self.current_color = color.color() # Color detected by Color Sensor
        self.hasBall = False
        self.isFindingBall = False # True if Robot is finding Ball
        self.notInGoal = True # True if Ball not in our goal

        self.mxSignal = max([int.from_bytes(self.isensor.read(0x43+i, length=1), "little") for i in range(5)])
        self.strengths = [int.from_bytes(self.isensor.read(0x43+i, length=1), "little") for i in range(5)]


        self.inTeamGoal = True
        self.onTeamSide = False
        self.onOpponentSide = False
        self.inOpponentGoal = False


        self.teamColor = TEAM_COLOR
        self.opponentColor = OPPONENT_COLOR
        self.midfield = MIDFIELD
        self.threshold = SIGNAL_THRESHOLD
        self.currentSide = TEAM_COLOR
        self.hasTurnedWithBall = False



    
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
        log("Robot supposed to turn: " + str(angle) + " degrees...")

    
        # Turn Tires
        self.left_motor. run_angle(TIRE_RPM, -(steering_angle), wait=False)
        self.right_motor.run_angle(TIRE_RPM, (steering_angle))
        self.navigator.update_nav(steering_angle)
            
        log("Current Robot Orientation: " + str(self.navigator.orientation) + " degrees...")
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

        log("Priority queue BEFORE Pop..." + str(self.queue))
        behavior = heapq.heappop(self.queue)
        log("Priority after AFTER Pop..." + str(self.queue))
        
        if behavior.priority == HAS_BALL:
            log("Robot has Ball...")
            behavior.run(self)
            log("Finished Behavior of having Ball...")

        if behavior.priority == FIND_BALL:
            log("Trying to find ball ...")
            behavior.run(self)


    def update_sensors(self) -> None:
        """Function updates all of the robot's sensor values and stores these values in the robot object
        """
        self.distanceToWall = self.usensor.distance()
        self.current_color = self.csensor.color()
        self.mxSignal = max([int.from_bytes(self.isensor.read(0x43+i, length=1), "little") for i in range(5)])
        self.strengths = [int.from_bytes(self.isensor.read(0x43+i, length=1), "little") for i in range(5)]

        log("Signal Strengths {}".format(str(self.strengths)))
        # log("Chose Zone {} with a value of {}".format(zone_val, max_val))
        

    def update_queue(self) -> None:
        """
        Updates the priority queue using the robot's sensor values. Defaults to FindBall if
        priority queue is empty.
        """
        if not self.queue:
            if not any(isinstance(behavior, FindBall) for behavior in self.queue):
                self.queue.append(FindBall())

        if self.mxSignal > SIGNAL_THRESHOLD:
           if not any(isinstance(behavior, HasBall) for behavior in self.queue):
                self.queue.append(HasBall())
    
"""responsible for coordinating the behaviors that the robot will take given the sensor data"""

from random import randint
import math
from logger import log
from globals import *
from pybricks.parameters import Color
import time

class RobotBehavior:
    """Coordinates the behaviors of the Robot given the priority of the behavior"""

    def __init__(self, priority):
        self.priority = priority

    def run(self, robot):
        """
        Starts a behavior and executes their corresponding functions\n
        Args: robot (Robot): the robot object
        """
        pass

    def stop_behavior(self, robot):
        """
        Stops the current behavior and resets the neccessary variables\n
        Args: robot (Robot): the robot object
        """
        pass

    def __lt__(self, other):
        return self.priority < other.priority



class FindBall(RobotBehavior):
    """
    Default behavior of robot. Performs these actions until 
    the robot sensors detect somtheing. Priority of 1
    """
    def __init__(self):
        super().__init__(1)
    
    def run(self, robot):
        self.FindBall(robot)

    def find_ball(self, robot):
        """
        Robot tries to find ball
        """
        robot.isFindingBall = True

        # FindBall until higher priority occurs 
        while robot.isFindingBall:
            robot.run()
            robot.update_sensors()

            if robot.ballDistace > 90 :
                self.stop_behavior(robot, "Stopped FindBalling because close to wall...")

    def stop_behavior(self, robot, msg):
        robot.stop()
        robot.isFindingBall = False
        log(msg)

    def recalibrate_front(self, robot) -> None:
        """
        Given that the robot ran into a wall,
        backup the robot 200 mm and turn it .
        """
        random_angle = randint(ANGLE_LOW_BOUND, ANGLE_UPPER_BOUND)
        robot.move(BACKUP_DISTANCE)
        robot.turn(random_angle)

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
            self.turn_move_to_ball(robot)
            robot.run()
            robot.update_sensors()

            if robot.ballDistace > 90 :
                self.stop_behavior(robot, "Stopped FindBalling because close to wall...")

    def stop_behavior(self, robot, msg):
        robot.stop()
        robot.isFindingBall = False
        log(msg)

    def turn_move_to_ball(self, robot):

        max_val = 0
        zone_val = 0

        for i in robot.strengths:
            if robot.strengths[i] > max_val:
                max_val = robot.strengths[i]
                zone_val = i

        if zone_val == 0:
            robot.turn(-120)
            robot.move(80)

        elif zone_val == 1:
            robot.turn(-60)
            robot.move(80)

        elif zone_val == 2:
            robot.move(80)

        elif zone_val == 3:
            robot.turn(60)
            robot.move(80)

        elif zone_val == 4:
            robot.turn(120)
            robot.move(80)

        

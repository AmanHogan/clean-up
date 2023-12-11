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
    
    def crossed_oponent_side(self, robot):
        log("Accidentally moved on opponents side. Backing up... Color Detected: " + str(robot.current_color))
        self.recalibrate_position(robot)
    
    def crossed_midline_side(self, robot):
        if robot.onTeamSide == True:
            robot.onOpponentSide = True
            robot.onTeamSide = False

        elif robot.onOpponentSide == True:
            robot.onOpponentSide = False
            robot.onTeamSide = False

        log("Crossed Middle line...")

    def crossed_team_side(self, robot):
        if robot.onTeamSide == True:
            robot.inTeamGoal  = True
            robot.onTeamSide = False

        elif robot.inTeamGoal == True:
            robot.onTeamSide == True
            robot.inTeamGoal = False

        log("Crossed own Team's line..." + str(robot.current_color))

    def recalibrate_position(self, robot):
        robot.move(BACKUP_DISTANCE)
        robot.turn(TURN_ANGLE)


class FindBall(RobotBehavior):
    """
    Default behavior of robot. Performs these actions until 
    the robot sensors detect somtheing. Priority of 1
    """
    def __init__(self):
        super().__init__(1)
        self.start_time = time.time()
    
    def run(self, robot):
        self.find_ball(robot)
        

    def find_ball(self, robot):
        """
        Robot tries to find ball
        """
        
        robot.isFindingBall = True
        robot.run()

        # FindBall until higher priority occurs 
        while robot.isFindingBall:
            
            elapsed_time = time.time() - self.start_time
            robot.update_sensors()

            if elapsed_time >= 0.5:
                self.turn_move_to_ball(robot)
                self.start_time = time.time()  # Reset the start time

            # If there is no longer a signal
            if robot.mxSignal > SIGNAL_THRESHOLD :
                self.stop_behavior(robot, "Stopped Finding Ball because ball is in front of sensor")

            # If on oppenent's side, backup and turn
            if robot.current_color == OPPONENT_COLOR :
                self.crossed_oponent_side(robot)

            # near wall
            if robot.distanceToWall < SIGNAL_THRESHOLD :
                self.align_robot_with_wall(robot)
                log("Very close to wall... Aligning self with wall")

            # Pass team line, change certain part
            if robot.current_color == TEAM_COLOR :
                self.crossed_team_side(robot)

            # If near wall, backup and turn
            if robot.distanceToWall < WALL_DISTANCE:
                self.recalibrate_position(robot)

            # passes middle line
            if robot.current_color == MIDFIELD :
                self.crossed_midline_side(robot)
                
            robot.run()

            
    def stop_behavior(self, robot, msg):
        robot.stop()
        robot.isFindingBall = False
        log(msg)

    def turn_move_to_ball(self, robot):
        max_val, zone_val = max((val, i) for i, val in enumerate(robot.strengths))

        if zone_val == 0:
            angle = -120
        elif zone_val == 1:
            angle = -60
        elif zone_val == 2:
            angle = 0
        elif zone_val == 3:
            angle = 60
        elif zone_val == 4:
            angle = 120

        robot.turn(angle)
        log("Chose Zone {} with a value of {}".format(zone_val, max_val))

    def align_robot_with_wall(self, robot):
        """
        Aligns the robot with the wall. Does arctan(Width / Distance) to find angle to turn\n
        Args: robot (Robot): the robot object
        """
        # Turn robot to orient itself with the wall
        angle_to_turn = math.degrees(math.atan(ROBOT_LENGTH / robot.distanceToWall))
        robot.turn(angle_to_turn)

      
class HasBall(RobotBehavior):
    """
    Default behavior of robot. Performs these actions until 
    the robot sensors detect somtheing. Priority of 1
    """
    def __init__(self):
        super().__init__(0)
    
    def run(self, robot):
        self.has_ball(robot)

    def has_ball(self, robot):
        """
        Robot has ball
        """
        robot.hasBall = True

        robot.run()

        while robot.hasBall:
            robot.update_sensors()

            # no ball
            if robot.mxSignal < SIGNAL_THRESHOLD :
                self.stop_behavior(robot, "Robot Does not have Ball anymore")

            # near wall
            if robot.distanceToWall < SIGNAL_THRESHOLD :
                self.recalibrate_position(robot)
                log("Very close to wall... Aligning self with wall")
                
            # on team side with ball
            if robot.onTeamSide == True:
                robot.turn(TURN_ANGLE)
                log("We have the ball, but we are on our own side. Turn around.")

            # on opponents side with ball
            if robot.onOpponentSide == True:
                log("We have the ball, and are on the opponent's side. Make a break for it!")
                
            # passes oponents side
            if robot.current_color == OPPONENT_COLOR :
                self.crossed_oponent_side(robot)

            # passes team side
            if robot.current_color == TEAM_COLOR :
                self.crossed_team_side(robot)

            # passes middle line
            if robot.current_color == MIDFIELD :
                self.crossed_midline_side(robot)
              
            robot.run()

    def align_robot_with_wall(self, robot):
        """
        Aligns the robot with the wall. Does arctan(Width / Distance) to find angle to turn\n
        Args: robot (Robot): the robot object
        """
        # Turn robot to orient itself with the wall
        angle_to_turn = math.degrees(math.atan(ROBOT_LENGTH / robot.distanceToWall))
        robot.turn(angle_to_turn)




    def stop_behavior(self, robot, msg):
        robot.stop()
        robot.hasBall = False
        robot.hasTurnedWithBall = False
        log(msg)
        

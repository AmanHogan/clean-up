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
        """
        If crossed opponents line, backup and turn\n
        Args: robot (Robot): robot
        """

        log("FUNC: Moved accross opponent's line. Backing up. " + str(robot.current_color))
        self.recalibrate_position(robot)
    
    def crossed_midline_side(self, robot):
        """
        If crossed middline, update the side that the robot thinks its on.\n
        Args: robot (Robot): robot
        """
        if robot.onTeamSide == True:
            log("FUNC: Crossed middleline. Went from TEAM SIDE to OPPONENT SIDE")
            robot.onOpponentSide = True
            robot.onTeamSide = False
            robot.ev3.speaker.say("Went from team side to opponent side")
            

        elif robot.onOpponentSide == True:
            log("FUNC: Crossed middleline. Went from OPPONENT SIDE to TEAM SIDE")
            robot.onOpponentSide = False
            robot.onTeamSide = True
            robot.ev3.speaker.say("Went from opponent side to team side")

        

    def crossed_team_side(self, robot):
        """
        If crossed team's line, update boolean variables\n
        Args: robot (Robot): robot
        """
        if robot.onTeamSide == True:
            log("FUNC: Crossed Team Line. Went from TEAM SIDE to TEAM GOAL")
            robot.inTeamGoal  = True
            robot.onTeamSide = False
            robot.ev3.speaker.say("Went from team side to team goal")


        elif robot.inTeamGoal == True:
            log("FUNC: Crossed Team Line. Went TEAM GOAL to TEAM SIDE")
            robot.onTeamSide = True
            robot.inTeamGoal = False
            robot.ev3.speaker.say("Went from team goal to team side")


        

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
        self.detectedColor = 0
        self.elapsedTime = time.time() - self.start_time
    
    def run(self, robot):
        self.find_ball(robot)
        
    def find_ball(self, robot):
        """
        Robot tries to find the ball\n
        Args: robot (Robot): robot
        """
        
        robot.isFindingBall = True
        robot.run()

        # While the robot is trying to find the ball ...
        while robot.isFindingBall:

            robot.update_sensors()
            
            turn_time = time.time() - self.start_time
            self.elapsedTime = time.time() - self.start_time
            
            # move to ball every .5 seconds
            if turn_time >= 1:
                self.turn_to_ball(robot)
                self.start_time = time.time() 

            # Found high singal
            if robot.mxSignal > SIGNAL_THRESHOLD :
                self.stop_behavior(robot, "Stopped finding ball because found a signal higher than threshold")

            # near wall
            if robot.distanceToWall < WALL_DISTANCE :
                self.recalibrate_position(robot)
                log("FUNC: Close to wall, backing up")

            # passed opponent line
            if robot.current_color == OPPONENT_COLOR :
                self.crossed_oponent_side(robot)
                robot.move(60)
                
            # passed team line
            if robot.current_color == TEAM_COLOR :
                self.crossed_team_side(robot)
                robot.move(60)

            # passed middle line
            if robot.current_color == MIDFIELD:
                self.crossed_midline_side(robot)
                robot.move(60)

            if self.elapsedTime > 5:
                self.elapsedTime = 0
                robot.turn(180)
                
            robot.run()
      
    def stop_behavior(self, robot, msg):
        robot.stop()
        robot.isFindingBall = False
        log("BEHAVIOR ENDED: " + msg)

    def turn_to_ball(self, robot) -> None:
        """
        Robot finds the max strength signal from the IR Sensor that was stored in the
        strengths array. Robot turns in increments depending on the index of the max
        strength in the strength array.
        """
        
        max_val, zone_val = max((val, i) for i, val in enumerate(robot.strengths))

        if zone_val == 0:
            angle = -(IR_TURN*2)
        elif zone_val == 1:
            angle = -(IR_TURN)
        elif zone_val == 2:
            angle = 0
        elif zone_val == 3:
            angle = (IR_TURN)
        elif zone_val == 4:
            angle = (IR_TURN*2)

        robot.turn(angle)
        robot.run()
        log("FUNC: Chose Zone {} with a value of {}".format(zone_val, max_val))

      
class HasBall(RobotBehavior):
    """
    Behavior when the robot detects it has a ball. Priority of 0.
    """
    def __init__(self):
        super().__init__(0)
    
    def run(self, robot):
        self.has_ball(robot)

    def has_ball(self, robot):
        """
        Robot currently has the ball
        Args: robot (Robor): robot
        """
        robot.hasBall = True
        robot.tireRPM = TIRE_RPM * 1.2
        robot.run()
        timeout_start = time.time()

        while robot.hasBall:

            robot.update_sensors()
            if (timeout_start - time.time()) > 5:
                robot.turn(120)

            # lost ball
            if robot.mxSignal < SIGNAL_THRESHOLD :
                self.stop_behavior(robot, "Robot Does not have Ball anymore")

            # near wall
            if robot.distanceToWall < WALL_DISTANCE :
                self.recalibrate_position(robot)
                log("FUNC: Very close to wall... Aligning self with wall")
                
            # on team side with ball
            if robot.onTeamSide == True or robot.inTeamGoal == True:
                robot.turn(TURN_ANGLE)
                log("FUNC: We have the ball, but we are on our own side or in own goal. Turn around.")

            # on opponents side with ball
            if robot.onOpponentSide == True:
                log("FUNC: We have the ball, and are on the opponent's side. Make a break for it!")
                
            # passes oponents line
            if robot.current_color == OPPONENT_COLOR :
                self.crossed_oponent_side(robot)
                robot.move(60)

            # passes team line
            if robot.current_color == TEAM_COLOR :
                self.crossed_team_side(robot)
                robot.move(60)

            # passes middle line
            if robot.current_color == MIDFIELD:
                self.crossed_midline_side(robot)
                robot.move(60)
              
            robot.run()

    def stop_behavior(self, robot, msg):
        robot.stop()
        robot.hasBall = False
        robot.tireRPM = TIRE_RPM
        log("BEHAVIOR ENDED: " + msg)
        

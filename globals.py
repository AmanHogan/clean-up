"""Contains constants and varaibles that will be modified before compile time"""

from pybricks.parameters import Color

################################ COMPILE TIME CONSTANTS ####################
# error factor when moving on surface
TILE_ERROR_DISTANCE = 1.00
DISTANCE_ERROR = TILE_ERROR_DISTANCE # Change this as needed

# error factor when moving on surface
TILE_ERROR_TURN = .950
TURN_ERROR = TILE_ERROR_TURN # Change this as needed
#########################################################################

####################### ROBOT CONSTANTS #################################
TIRE_CIRC = 178 # Circumference of the tire in [mm]
FULL_ROTATION = 360 # [#]
TIRE_RPM = 700 # Revolutions per minute [r/min] # Change this as needed
ROBOT_LENGTH = 105 # Length of the robot in [mm]
DIST_BTWN_WHEELS = 158.0000 # Distance between the wheels in [mm]
ROBOT_RADIUS = (DIST_BTWN_WHEELS/2.0000) # Radius of robot tire axel [mm]
M_PI = 3.14159265359  # pi constant [#]

# Behviour Enums
HAS_BALL = 0
FIND_BALL = 1
###########################################################################

################################ ALARM CONSTANTS ##########################
TEAM_COLOR = Color.BLUE
OPPONENT_COLOR = Color.RED
MIDFIELD = Color.GREEN
SIGNAL_THRESHOLD = 80
WALL_DISTANCE = 80
BACKUP_DISTANCE = -250
TURN_ANGLE = 180
###########################################################################
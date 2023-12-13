"""Contains constants and varaibles that will be modified before compile time"""

from pybricks.parameters import Color

# Error factors when moving or turning on a surface
TILE_ERROR_DISTANCE = 1.00
DISTANCE_ERROR = TILE_ERROR_DISTANCE
TILE_ERROR_TURN = .950
TURN_ERROR = TILE_ERROR_TURN

# Robot consts [mm]
TIRE_CIRC = 178 
FULL_ROTATION = 360
TIRE_RPM = 200
 
ROBOT_LENGTH = 105 
DIST_BTWN_WHEELS = 158.0000 
ROBOT_RADIUS = (DIST_BTWN_WHEELS/2.0000) 
M_PI = 3.14159265359

# Behviour Enums
HAS_BALL = 0
FIND_BALL = 1

# Soccer consts
TEAM_COLOR = Color.BLUE 
OPPONENT_COLOR = Color.RED
MIDFIELD = Color.GREEN
SIGNAL_THRESHOLD = 80
WALL_DISTANCE = 130
BACKUP_DISTANCE = -250

TURN_ANGLE = 180
""" Angle robot turns when it needs to turn away from an object"""

IR_TURN = 60
""" The angle seperation between each zone in the strengths array """

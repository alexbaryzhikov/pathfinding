from pygame.locals import *

""" Screen """
SCREEN_W = 1800
SCREEN_H = 1200
SCREEN_MODE = 0
# SCREEN_W = 2560
# SCREEN_H = 1440
# SCREEN_MODE = 0 | FULLSCREEN #| DOUBLEBUF | HWSURFACE | RESIZABLE | NOFRAME
MAX_FPS = 120
FPS_BUF_SIZE = 100

""" Colors """
BG_COLOR = (12, 15, 22)
EDGE_COLOR = (80, 114, 27)
WALL_COLOR = (100, 0, 0)
PATH_COLOR = (150, 0, 0)

""" Agent """
AGENT_RADIUS = 35
AGENT_SPEED = 500
WP_TOLERANCE = 50               # switch to next waypoint when current is in vicinity

""" Obstacles """
COLLIDABLE_NUM = 50
COLLIDABLE_R_MIN = 10
COLLIDABLE_R_MAX = 80
COLLIDABLE_MIN_DIST = 2         # min allowed dist. betw. agent and obstacle

""" Grid """
GRID_NODE_WIDTH = 40
MOVE_COST = 10
MOVE_COST_DIAG = 14

""" Mesh """
BORDER_POINTS_THRESHOLD = 80    # max dist. betw. obstacle and border to spawn border point
BORDER_POINTS_MERGE_DIST = 60   # don't spawn border points closer that this

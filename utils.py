import pygame

# some global constants
WIDTH = 800
HEIGHT = 700

GRID_WIDTH = 640
GRID_HEIGHT = 640

MARGIN = 30

# colors.
# if you find it more suitable, change this dictionary to standalone constants like: RED = (255, 0, 0)
COLORS = {
    'RED': (255, 0, 0),           # closed nodes
    'GREEN': (0, 255, 0),         # open nodes
    'DARK_GREEN': (50, 140, 90),
    'BLUE': (0, 0, 255),          # start node
    'DARK_BLUE': (50, 90, 140),
    'YELLOW': (255, 255, 0),      # end node
    'WHITE': (255, 255, 255),     # unvisited nodes
    'BLACK': (0, 0, 0),           # barrier
    'PURPLE': (128, 0, 128),      # path
    'ORANGE': (255, 165 ,0),      # nodes being considered
    'GREY': (128, 128, 128),      # grid lines
    'TURQUOISE': (64, 224, 208)   # neighbor nodes
}
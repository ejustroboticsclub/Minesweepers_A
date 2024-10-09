import pygame
from enum import Enum, auto


class MineType(Enum):
    NO_MINE = 0
    SURFACE = 1
    BURIED = 2


class CornerPosition(Enum):
    BOTTOM_LEFT = auto()
    BOTTOM_RIGHT = auto()
    TOP_LEFT = auto()
    TOP_RIGHT = auto()


TILE_SIZE = 70  # Size of each tile in pixels
MAP_WIDTH = 10  # Width of the map in tiles
MAP_HEIGHT = 10  # Height of the map in tiles
FONT_SIZE = 30
TABLE_WIDTH = 250
TABLE_HEIGHT = MAP_HEIGHT * TILE_SIZE
TABLE_PADDING = 10
TABLE_SPACING = 10  # Space between the two tables


# Load images
EMPTY_IMG = pygame.Surface((TILE_SIZE, TILE_SIZE))
EMPTY_IMG.fill((255, 255, 255))  # White for empty space
SURFACE_MINE_IMG = pygame.Surface((TILE_SIZE, TILE_SIZE))
SURFACE_MINE_IMG.fill((0, 255, 255))  # Cyan for surface mines
BURIED_MINE_IMG = pygame.Surface((TILE_SIZE, TILE_SIZE))
BURIED_MINE_IMG.fill((0, 128, 128))  # Teal for buried mines
ROBOT_IMG = pygame.transform.scale(
    pygame.image.load("/home/ahmed/catkin_ws/src/map/src/robot.png"),
    (TILE_SIZE, TILE_SIZE),
)


# Button settings
BUTTON_WIDTH = 60
BUTTON_HEIGHT = 30
CORNER_COLOR = (255, 183, 178)
ORIENTATION_COLOR = (255, 218, 193)
SELECTED_COLOR = (255, 154, 162)
TEXT_COLOR = (255, 255, 255)


# Corner and orientation positions (in grid coordinates)
CORNER_POSITIONS = {
    CornerPosition.BOTTOM_LEFT: (0, 0),
    CornerPosition.BOTTOM_RIGHT: (MAP_WIDTH - 1, 0),
    CornerPosition.TOP_LEFT: (0, MAP_HEIGHT - 1),
    CornerPosition.TOP_RIGHT: (MAP_WIDTH - 1, MAP_HEIGHT - 1),
}

# Orientation cells relative to corner positions (adjacent cells)
ORIENTATION_OFFSETS = {
    CornerPosition.BOTTOM_LEFT: ((1, 0), (0, 1)),  # Right and Above
    CornerPosition.BOTTOM_RIGHT: ((0, 1), (-1, 0)),  # Above and Left
    CornerPosition.TOP_LEFT: ((1, 0), (0, -1)),  # Right and Below
    CornerPosition.TOP_RIGHT: ((0, -1), (-1, 0)),  # Below and Left
}
# Orientation directions (relative to the corner cells)
# To verify you have to check the location of the imu sensor on the robot
ORIENTATIONS = {
    CornerPosition.BOTTOM_LEFT: (0, 90),
    CornerPosition.BOTTOM_RIGHT: (90, 180),
    CornerPosition.TOP_LEFT: (0, -90),
    CornerPosition.TOP_RIGHT: (-90, 180),
}

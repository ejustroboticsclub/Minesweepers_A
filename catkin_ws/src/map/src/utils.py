import math
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from map import Robot_Coordinates


# Function to calculate mine position based on robot's position and orientation
def calculate_mine_position(
    rel_x: int, rel_y: int, robot_coords: "Robot_Coordinates"
) -> "tuple[int, int]":
    theta_rad = math.radians(robot_coords.theta)

    # Calculate absolute mine position based on robot's position and orientation
    abs_x = robot_coords.x + rel_x * math.cos(theta_rad) - rel_y * math.sin(theta_rad)
    abs_y = robot_coords.y + rel_x * math.sin(theta_rad) + rel_y * math.cos(theta_rad)

    # Convert to grid coordinates (assuming the robot's coordinates are also in grid units)
    grid_x = int(round(abs_x))
    grid_y = int(round(abs_y))

    return grid_x, grid_y

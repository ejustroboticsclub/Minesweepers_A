"""
Task Description:

This script uses Pygame to display a minefield map, integrated with ROS for real-time updates. 

- **Map Visualization**: Shows a grid-based map with tiles for empty spaces, surface mines, and buried mines.
- **ROS Integration**: Subscribes to topics for robot pose and mine detection, updating the map accordingly.
- **Coordinate System**: The map's origin (0,0) is at the bottom-left, with Y-axis increasing upwards.
- **Dynamic Updates**: Mines are placed based on the robot's position and orientation, displayed in two tables (surface and buried).

The script initializes Pygame, sets up ROS subscribers, and updates the map until the application exits.

*****************IMPORTANT: change the pkg name to 'map'**************************
"""


#!/usr/bin/env python3

import pygame
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
import math
import tf.transformations as tf


# Initialize Pygame
pygame.init()

# Constants
TILE_SIZE = 32  # Size of each tile in pixels
MAP_WIDTH = 20  # Width of the map in tiles
MAP_HEIGHT = 20  # Height of the map in tiles
FONT_SIZE = 14
TABLE_WIDTH = 250
TABLE_HEIGHT = MAP_HEIGHT * TILE_SIZE
TABLE_PADDING = 10
TABLE_SPACING = 10  # Space between the two tables

# Create the display
screen = pygame.display.set_mode((MAP_WIDTH * TILE_SIZE + 2 * (TABLE_WIDTH + TABLE_SPACING), MAP_HEIGHT * TILE_SIZE + 100))
pygame.display.set_caption("Minefield Map")

# Define the initial map (0 for empty space, 1 for surface mine, 2 for buried mine)
map_data = [[0] * MAP_WIDTH for _ in range(MAP_HEIGHT)]

# Load images
empty_img = pygame.Surface((TILE_SIZE, TILE_SIZE))
empty_img.fill((255, 255, 255))  # White for empty space
surface_mine_img = pygame.Surface((TILE_SIZE, TILE_SIZE))
surface_mine_img.fill((0, 255, 255))  # Cyan for surface mines
buried_mine_img = pygame.Surface((TILE_SIZE, TILE_SIZE))
buried_mine_img.fill((0, 128, 128))  # Teal for buried mines

# Set up font
font = pygame.font.Font(None, FONT_SIZE)

# Button settings
BUTTON_WIDTH = 60
BUTTON_HEIGHT = 30
CORNER_COLOR = (195, 177, 225)  # Semi-transparent blue for corners
ORIENTATION_COLOR = (248, 200, 220)  # Semi-transparent orange for orientations
SELECTED_COLOR = (0, 255, 0, 200)  # Semi-transparent green for selected
TEXT_COLOR = (255, 255, 255)

# Orientation directions (relative to the corner cells)
orientations = {
    "Bottom-left": [0, math.pi / 2],
    "Bottom-right": [math.pi / 2, math.pi],
    "Top-left": [0, -math.pi / 2],
    "Top-right": [-math.pi / 2, math.pi]
}

# Corner and orientation positions (in grid coordinates)
corner_positions = {
    "Bottom-left": (0, 0),
    "Bottom-right": (MAP_WIDTH - 1, 0),
    "Top-left": (0, MAP_HEIGHT - 1),
    "Top-right": (MAP_WIDTH - 1, MAP_HEIGHT - 1)
}

# Orientation cells relative to corner positions (adjacent cells)
orientation_offsets = {
    "Bottom-left": [(1, 0), (0, 1)],  # Right and above
    "Bottom-right": [(-1, 0), (0, 1)],  # Left and above
    "Top-left": [(1, 0), (0, -1)],  # Right and below
    "Top-right": [(-1, 0), (0, -1)]  # Left and below
}

# Robot's current position and orientation
robot_x, robot_y, robot_theta = 0, 0, 0  # Initialize with default values

# Active corner and orientation
active_corner = None
active_orientation = None
selection_complete = False  # New variable to track if selection is complete

# Function to calculate mine position based on robot's position and orientation
def calculate_mine_position(rel_x, rel_y, theta):
    theta_rad = math.radians(theta)
    
    # Calculate absolute mine position based on robot's position and orientation
    abs_x = robot_x + rel_x * math.cos(theta_rad) - rel_y * math.sin(theta_rad)
    abs_y = robot_y + rel_x * math.sin(theta_rad) + rel_y * math.cos(theta_rad)
    
    # Convert to grid coordinates (assuming the robot's coordinates are also in grid units)
    grid_x = int(round(abs_x))
    grid_y = int(round(abs_y))
    
    return grid_x, grid_y

# ROS callback for robot position and orientation
def robot_pose_callback(data):
    global robot_x, robot_y, robot_theta
    try:
        pose = data.pose.pose
        robot_x = pose.position.x - 1
        robot_y = pose.position.y - 1
        orientation_q = pose.orientation

        # Convert quaternion to Euler angles
        robot_theta = orientation_q.z

        rospy.loginfo(f"Robot position: ({robot_x}, {robot_y}), Orientation: {robot_theta:.2f} degrees")
    except Exception as e:
        rospy.logerr(f"Exception in robot_pose_callback: {e}")

# ROS callback for mine detection
def mine_detection_callback(data):
    global map_data, mine_type
    mine_type = data.data  # "surface", "buried", or "no mine"
    
    if mine_type != "no mine":
        # Assuming the mine is always 1 unit in front of the robot
        rel_x, rel_y = 1, 0  # Relative position of the mine (1 unit in front)
        mine_x, mine_y = calculate_mine_position(rel_x, rel_y, robot_theta)
        
        if 0 <= mine_x < MAP_WIDTH and 0 <= mine_y < MAP_HEIGHT:
            if mine_type == "surface":
                map_data[mine_y][mine_x] = 1
            elif mine_type == "buried":
                map_data[mine_y][mine_x] = 2
        else:
            rospy.logwarn(f"Mine position out of bounds: ({mine_x}, {mine_y})")
    else:
        rospy.loginfo("No mine detected.")

# Function to draw the map, including the robot
def draw_map(screen):
    for y, row in enumerate(map_data):
        for x, tile in enumerate(row):
            # Draw the appropriate tile
            if tile == 1:
                screen.blit(surface_mine_img, (x * TILE_SIZE, (MAP_HEIGHT - y - 1) * TILE_SIZE))
            elif tile == 2:
                screen.blit(buried_mine_img, (x * TILE_SIZE, (MAP_HEIGHT - y - 1) * TILE_SIZE))
            else:
                screen.blit(empty_img, (x * TILE_SIZE, (MAP_HEIGHT - y - 1) * TILE_SIZE))

            # Draw grid lines
            pygame.draw.rect(screen, (0, 0, 0), (x * TILE_SIZE, (MAP_HEIGHT - y - 1) * TILE_SIZE, TILE_SIZE, TILE_SIZE), 1)

            # Draw coordinates
            coord_text = font.render(f"{chr(65 + x)},{y + 1}", True, (0, 0, 0))
            screen.blit(coord_text, (x * TILE_SIZE + 2, (MAP_HEIGHT - y - 1) * TILE_SIZE + 2))

    # Draw the robot as a blue circle
    robot_screen_x = int(robot_x * TILE_SIZE)
    robot_screen_y = int((MAP_HEIGHT - robot_y - 1) * TILE_SIZE)  # Convert map coordinates to screen coordinates
    pygame.draw.circle(screen, (0, 0, 255), (robot_screen_x + TILE_SIZE // 2, robot_screen_y + TILE_SIZE // 2), TILE_SIZE // 3)

    # Draw the arrow to indicate the robot's orientation
    arrow_length = TILE_SIZE // 2  # Length of the arrow
    arrow_x = robot_screen_x + TILE_SIZE // 2 + arrow_length * math.cos(robot_theta)
    arrow_y = robot_screen_y + TILE_SIZE // 2 - arrow_length * math.sin(robot_theta)  # Y axis is inverted

    pygame.draw.line(screen, (255, 0, 0), (robot_screen_x + TILE_SIZE // 2, robot_screen_y + TILE_SIZE // 2), (arrow_x, arrow_y), 3)

# Function to draw the corner and orientation cells

def draw_corner_orientation_cells(screen):
    global active_corner, active_orientation, selection_complete

    if selection_complete:
        return  # No drawing if selection is complete

    if active_corner:
        # Draw only the active corner cell
        corner_x, corner_y = corner_positions[active_corner]
        pygame.draw.rect(screen, SELECTED_COLOR, (corner_x * TILE_SIZE, (MAP_HEIGHT - corner_y - 1) * TILE_SIZE, TILE_SIZE, TILE_SIZE))

        # Draw the adjacent cells for orientation
        for i, (ox, oy) in enumerate(orientation_offsets[active_corner]):
            orientation_x = corner_x + ox
            orientation_y = corner_y + oy
            if 0 <= orientation_x < MAP_WIDTH and 0 <= orientation_y < MAP_HEIGHT:
                color = SELECTED_COLOR if active_orientation == (active_corner, i) else ORIENTATION_COLOR
                pygame.draw.rect(screen, color, (orientation_x * TILE_SIZE, (MAP_HEIGHT - orientation_y - 1) * TILE_SIZE, TILE_SIZE, TILE_SIZE))

    # When a corner and orientation are selected, do not draw any corners or orientations
    elif not active_corner:
        # Draw all corners initially
        for corner, (cx, cy) in corner_positions.items():
            pygame.draw.rect(screen, CORNER_COLOR, (cx * TILE_SIZE, (MAP_HEIGHT - cy - 1) * TILE_SIZE, TILE_SIZE, TILE_SIZE))

        # Draw orientation cells around all corners
        for corner, (cx, cy) in corner_positions.items():
            for i, (ox, oy) in enumerate(orientation_offsets[corner]):
                orientation_x = cx + ox
                orientation_y = cy + oy
                if 0 <= orientation_x < MAP_WIDTH and 0 <= orientation_y < MAP_HEIGHT:
                    pygame.draw.rect(screen, ORIENTATION_COLOR, (orientation_x * TILE_SIZE, (MAP_HEIGHT - orientation_y - 1) * TILE_SIZE, TILE_SIZE, TILE_SIZE))

# Function to handle mouse click events
def handle_mouse_click(pos):
    global robot_x, robot_y, robot_theta, active_corner, active_orientation, selection_complete

    if selection_complete:
        return  # Ignore clicks if selection is complete

    mouse_x, mouse_y = pos
    grid_x = mouse_x // TILE_SIZE
    grid_y = MAP_HEIGHT - 1 - mouse_y // TILE_SIZE

    # Check if the click is on a corner cell
    for corner, (cx, cy) in corner_positions.items():
        if grid_x == cx and grid_y == cy:
            if active_corner is None:  # Allow selecting a corner only if none is selected
                active_corner = corner
                robot_x, robot_y = cx, cy
                active_orientation = None  # Reset active orientation
                print(f"Selected corner: {corner}")
                return

    # Check if the click is on an orientation cell
    if active_corner:
        for i, (ox, oy) in enumerate(orientation_offsets[active_corner]):
            orientation_x = corner_positions[active_corner][0] + ox
            orientation_y = corner_positions[active_corner][1] + oy
            if grid_x == orientation_x and grid_y == orientation_y:
                active_orientation = (active_corner, i)
                robot_theta = orientations[active_corner][i]
                print(f"Selected orientation: {robot_theta} radians")
                selection_complete = True  # Mark selection as complete
                return

# Function to draw the tables for surface and buried mines
def draw_tables(screen, surface_table, buried_table_surface):
    surface_table.fill((255, 255, 255))  # White background
    buried_table_surface.fill((255, 255, 255))  # White background
    pygame.draw.rect(surface_table, (0, 0, 0), (0, 0, TABLE_WIDTH, TABLE_HEIGHT), 2)  # Border for surface table
    pygame.draw.rect(buried_table_surface, (0, 0, 0), (0, 0, TABLE_WIDTH, TABLE_HEIGHT), 2)  # Border for buried table
    
    y_offset = TABLE_PADDING
    for y in range(MAP_HEIGHT):
        for x in range(MAP_WIDTH):
            if map_data[y][x] == 1:  # Surface mines
                mine_type = "Surface"
                text = font.render(f"{chr(65 + x)},{y + 1}: {mine_type}", True, (0, 0, 0))  # Convert index to char
                surface_table.blit(text, (TABLE_PADDING, y_offset))
                y_offset += FONT_SIZE + 2
                if y_offset > TABLE_HEIGHT - FONT_SIZE:
                    return
    y_offset = TABLE_PADDING
    for y in range(MAP_HEIGHT):
        for x in range(MAP_WIDTH):
            if map_data[y][x] == 2:  # Buried mines
                mine_type = "Buried"
                text = font.render(f"{chr(65 + x)},{y + 1}: {mine_type}", True, (0, 0, 0))  # Convert index to char
                buried_table_surface.blit(text, (TABLE_PADDING, y_offset))
                y_offset += FONT_SIZE + 2
                if y_offset > TABLE_HEIGHT - FONT_SIZE:
                    return

# Create the table surfaces
surface_table_surface = pygame.Surface((TABLE_WIDTH, TABLE_HEIGHT))
buried_table_surface = pygame.Surface((TABLE_WIDTH, TABLE_HEIGHT))

def main():
    global robot_x, robot_y, robot_theta, active_corner, active_orientation, selection_complete

    # Initialize ROS node
    rospy.init_node('minefield_map', anonymous=True)
    rospy.Subscriber('robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, robot_pose_callback)
    rospy.Subscriber('mine_type', String, mine_detection_callback)

    rate = rospy.Rate(10)  # 10 Hz

    running = True
    while running and not rospy.is_shutdown():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                # Handle click on map (corner or orientation)
                handle_mouse_click(event.pos)

        # Draw the map
        draw_map(screen)

        # Draw the corner and orientation cells
        draw_corner_orientation_cells(screen)

        # Draw the tables
        draw_tables(screen, surface_table_surface, buried_table_surface)
        screen.blit(surface_table_surface, (MAP_WIDTH * TILE_SIZE + TABLE_SPACING, 0))
        screen.blit(buried_table_surface, (MAP_WIDTH * TILE_SIZE + TABLE_SPACING + TABLE_WIDTH + TABLE_SPACING, 0))

        pygame.display.flip()
        rate.sleep()

    pygame.quit()

if __name__ == '__main__':
    main()

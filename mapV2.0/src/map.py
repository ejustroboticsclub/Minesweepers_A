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






#!/usr/bin/env python

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
MAP_WIDTH = 20  # Width of the map in tiles (Horizontal)
MAP_HEIGHT = 20  # Height of the map in tiles (Vertical)
FONT_SIZE = 14
TABLE_WIDTH = 250
TABLE_HEIGHT = MAP_HEIGHT * TILE_SIZE
TABLE_PADDING = 10
TABLE_SPACING = 10  # Space between the two tables

# Create the display
screen = pygame.display.set_mode((MAP_WIDTH * TILE_SIZE + 2 * (TABLE_WIDTH + TABLE_SPACING), MAP_HEIGHT * TILE_SIZE))
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

# Robot's current position and orientation
robot_x, robot_y, robot_theta = 0, 0, 0  # Initialize with default values

# Current mine type (surface, buried, or no mine)
mine_type = "no mine"

# Function to calculate mine position based on robot's position and orientation
def calculate_mine_position(rel_x, rel_y, theta):
    # Convert theta to radians
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
        rospy.loginfo(f"orientation_q: {orientation_q}")
        orientation_q = pose.orientation

        # Convert quaternion to Euler angles
        # (_roll, _pitch, yaw) = tf.euler_from_quaternion(
        #     [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        # )
        # robot_theta = yaw
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

# Function to draw the map
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


# Function to draw the tables
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
        
        # Draw the map
        draw_map(screen)

        # Draw the tables
        draw_tables(screen, surface_table_surface, buried_table_surface)
        screen.blit(surface_table_surface, (MAP_WIDTH * TILE_SIZE + TABLE_SPACING, 0))
        screen.blit(buried_table_surface, (MAP_WIDTH * TILE_SIZE + TABLE_SPACING + TABLE_WIDTH + TABLE_SPACING, 0))

        pygame.display.flip()
        rate.sleep()

    pygame.quit()

if __name__ == '__main__':
    main()

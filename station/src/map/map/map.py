#!/usr/bin/env python3

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

import time
import os
import math
import pygame
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool

from utils import calculate_mine_position
import const
from const import MineType, CornerPosition

pygame.init()
pygame.display.set_caption("Minefield Map")
FONT = pygame.font.Font(None, const.FONT_SIZE)

orientation_offset: "float | None" = None
map_data = [[MineType.NO_MINE] * const.MAP_WIDTH for _ in range(const.MAP_HEIGHT)]
first_detection: bool = True
camera_detected: "bool | None" = None
metal_detected: "bool | None" = None
active_corner: "CornerPosition | None" = None
active_orientation = None
selection_complete: bool = False


@dataclass
class RobotCoordinates:
    x: float = 0
    y: float = 0
    theta: float = 0

    def update_pose(self, data: Pose2D) -> None:
        global orientation_offset, active_corner
        try:
            if orientation_offset is not None:
                self.theta = data.theta + orientation_offset
                # Handle corner-based coordinate adjustments
                if active_corner == CornerPosition.BOTTOM_LEFT:
                    self.x = data.x
                    self.y = data.y
                elif active_corner == CornerPosition.BOTTOM_RIGHT:
                    self.x = const.MAP_WIDTH - data.x - 1
                    self.y = data.y
                elif active_corner == CornerPosition.TOP_LEFT:
                    self.x = data.x
                    self.y = const.MAP_HEIGHT - data.y - 1
                elif active_corner == CornerPosition.TOP_RIGHT:
                    self.x = const.MAP_WIDTH - data.x - 1
                    self.y = const.MAP_HEIGHT - data.y - 1
            print(
                f"Robot position: ({self.x}, {self.y}), Orientation: {self.theta:.2f}°"
            )
        except Exception as e:
            print(f"Exception in robot_pose_callback: {e}")


robot_coordinates = RobotCoordinates()


class MinefieldMapNode(Node):
    def __init__(self):
        super().__init__("minefield_map")
        # Subscribers
        self.create_subscription(
            Pose2D, "pose_combined", robot_coordinates.update_pose, 10
        )
        self.create_subscription(
            Bool, "camera_detection", self.camera_detection_callback, 10
        )
        self.create_subscription(Bool, "detection", self.metal_detector_callback, 10)

    def camera_detection_callback(self, msg: Bool):
        global camera_detected
        camera_detected = msg.data
        self.get_logger().info(
            f"Camera detection: {'Mine detected' if camera_detected else 'No mine detected'}"
        )

    def metal_detector_callback(self, msg: Bool):
        global metal_detected
        metal_detected = msg.data
        self.get_logger().info(
            f"Metal detection: {'Mine detected' if metal_detected else 'No mine detected'}"
        )


def mine_detection_callback():
    global first_detection, map_data
    if not selection_complete:
        print("Selection of corner and orientation not complete.")
        return
    if metal_detected:
        mine_type = MineType.SURFACE if camera_detected else MineType.BURIED
    else:
        mine_type = MineType.NO_MINE
    if first_detection:
        glitch_x, glitch_y = calculate_mine_position(1, 0, robot_coordinates)
        if 0 <= glitch_x < const.MAP_WIDTH and 0 <= glitch_y < const.MAP_HEIGHT:
            map_data[glitch_y][glitch_x] = MineType.NO_MINE
        first_detection = False
        print("Handled the first detection glitch.")
        return
    if mine_type != MineType.NO_MINE:
        mine_x, mine_y = calculate_mine_position(1, 0, robot_coordinates)
        if 0 <= mine_x < const.MAP_WIDTH and 0 <= mine_y < const.MAP_HEIGHT:
            map_data[mine_y][mine_x] = mine_type
        else:
            print(f"Mine position out of bounds: ({mine_x}, {mine_y})")


def main():
    rclpy.init()
    node = MinefieldMapNode()
    screen = pygame.display.set_mode(
        (
            const.MAP_WIDTH * const.TILE_SIZE
            + 2 * (const.TABLE_WIDTH + const.TABLE_SPACING),
            const.MAP_HEIGHT * const.TILE_SIZE,
        )
    )
    surface_table_surface = pygame.Surface((const.TABLE_WIDTH, const.TABLE_HEIGHT))
    buried_table_surface = pygame.Surface((const.TABLE_WIDTH, const.TABLE_HEIGHT))

    try:
        while rclpy.ok():
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    rclpy.shutdown()
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    handle_mouse_click(event.pos)

            mine_detection_callback()
            draw_map(screen)
            draw_corner_orientation_cells(screen)
            draw_tables(surface_table_surface, buried_table_surface)
            screen.blit(
                surface_table_surface,
                (const.MAP_WIDTH * const.TILE_SIZE + const.TABLE_SPACING, 0),
            )
            screen.blit(
                buried_table_surface,
                (
                    const.MAP_WIDTH * const.TILE_SIZE
                    + const.TABLE_SPACING
                    + const.TABLE_WIDTH
                    + const.TABLE_SPACING,
                    0,
                ),
            )
            pygame.display.flip()
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        save_map_screenshot("/home/ahmed/Map_Screenshots", screen)
        pygame.quit()
        node.destroy_node()


if __name__ == "__main__":
    main()

import pygame
import rospy
from std_msgs.msg import String

# Initialize Pygame
pygame.init()

# Constants
TILE_SIZE = 32  # Size of each tile in pixels
MAP_WIDTH = 20  # Width of the map in tiles
MAP_HEIGHT = 19  # Height of the map in tiles
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

# ROS callbacks
def metal_detector_callback(data):
    global map_data
    # Parse the data and update the map
    positions = data.data.split(',')
    for pos in positions:
        x, y, mine_type = pos.split(':')
        x, y = int(x), int(y)
        if mine_type == 'buried':
            map_data[y][x] = 2

def yolo_callback(data):
    global map_data
    # Parse the data and update the map
    positions = data.data.split(',')
    for pos in positions:
        x, y, mine_type = pos.split(':')
        x, y = int(x), int(y)
        if mine_type == 'surface':
            map_data[y][x] = 1

# Initialize ROS node
rospy.init_node('minefield_map', anonymous=True)
rospy.Subscriber('metal_detector', String, metal_detector_callback)
rospy.Subscriber('yolo_model', String, yolo_callback)

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
                text = font.render(f"{x},{y}: {mine_type}", True, (0, 0, 0))
                surface_table.blit(text, (TABLE_PADDING, y_offset))
                y_offset += FONT_SIZE + 2
                if y_offset > TABLE_HEIGHT - FONT_SIZE:
                    return
    y_offset = TABLE_PADDING
    for y in range(MAP_HEIGHT):
        for x in range(MAP_WIDTH):
            if map_data[y][x] == 2:  # Buried mines
                mine_type = "Buried"
                text = font.render(f"{x},{y}: {mine_type}", True, (0, 0, 0))
                buried_table_surface.blit(text, (TABLE_PADDING, y_offset))
                y_offset += FONT_SIZE + 2
                if y_offset > TABLE_HEIGHT - FONT_SIZE:
                    return

# Create the table surfaces
surface_table_surface = pygame.Surface((TABLE_WIDTH, TABLE_HEIGHT))
buried_table_surface = pygame.Surface((TABLE_WIDTH, TABLE_HEIGHT))

# Main loop
running = True
while running and not rospy.is_shutdown():
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    
    # Draw the map
    for y, row in enumerate(map_data):
        for x, tile in enumerate(row):
            if tile == 1:
                screen.blit(surface_mine_img, (x * TILE_SIZE, y * TILE_SIZE))
            elif tile == 2:
                screen.blit(buried_mine_img, (x * TILE_SIZE, y * TILE_SIZE))
            else:
                screen.blit(empty_img, (x * TILE_SIZE, y * TILE_SIZE))

            # Draw grid lines
            pygame.draw.rect(screen, (0, 0, 0), (x * TILE_SIZE, y * TILE_SIZE, TILE_SIZE, TILE_SIZE), 1)

            # Draw coordinates
            coord_text = font.render(f"{x},{y}", True, (0, 0, 0))
            screen.blit(coord_text, (x * TILE_SIZE + 2, y * TILE_SIZE + 2))

    # Draw the tables
    draw_tables(screen, surface_table_surface, buried_table_surface)
    screen.blit(surface_table_surface, (MAP_WIDTH * TILE_SIZE + TABLE_SPACING, 0))
    screen.blit(buried_table_surface, (MAP_WIDTH * TILE_SIZE + TABLE_SPACING + TABLE_WIDTH + TABLE_SPACING, 0))

    pygame.display.flip()

pygame.quit()

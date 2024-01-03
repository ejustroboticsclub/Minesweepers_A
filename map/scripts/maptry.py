import matplotlib.pyplot as plt

# Define the grid size
rows = 10
cols = 10

# Create a 2D grid to represent the map
grid = [[' ' for _ in range(cols)] for _ in range(rows)]

# Define a mapping from numerical Y coordinates to alphabet characters
y_coordinate_mapping = {i: chr(ord('A') + i) for i in range(rows)}

def plot_grid(grid):
    fig, ax = plt.subplots()
    ax.matshow([[0]], cmap='RdGy', extent=(0, cols, 0, rows))
    
    for i in range(rows):
        for j in range(cols):
            if grid[i][j] == 'X':
                ax.add_patch(plt.Rectangle((j, i), 1, 1, color='red'))
            ax.text(j + 0.5, i + 0.5, grid[i][j], va='center', ha='center')

    ax.set_xticks(range(cols + 1))
    ax.set_yticks(range(rows + 1))
    ax.grid(which='both', color='black', linewidth=2)
    plt.gca().invert_yaxis()
    plt.show()

# Example usage: Mark a location with 'X' based on user input
x = int(input("Enter the numerical X coordinate (1 to 10): ")) - 1
y = int(input("Enter the numerical Y coordinate (1 to 10): ")) - 1

if 0 <= x < cols and 0 <= y < rows:
    grid[y][x] = 'X'
    plot_grid(grid)
else:
    print("Invalid coordinates.")

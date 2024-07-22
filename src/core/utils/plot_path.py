import numpy as np
import matplotlib.pyplot as plt

def read_grid(file_path):
    with open(file_path, 'r') as file:
        grid = [list(map(int, line.split())) for line in file]
    return np.array(grid)

def read_path(file_path):
    with open(file_path, 'r') as file:
        path = [tuple(map(float, line.split())) for line in file]
    return path

def plot_grid_and_path(grid, path):
    plt.figure(figsize=(10, 10))
    plt.imshow(grid, cmap='gray_r')

    path_x = [state[0] for state in path]
    path_y = [state[1] for state in path]

    plt.plot(path_x, path_y, color='red', linewidth=2, marker='o')
    plt.scatter(path_x[0], path_y[0], color='green', s=100, label='Start')
    plt.scatter(path_x[-1], path_y[-1], color='blue', s=100, label='Goal')

    plt.title('Hybrid A* Path')
    plt.legend()
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    grid = read_grid("../build/grid.txt")
    path = read_path("../build/path.txt")
    plot_grid_and_path(grid, path)

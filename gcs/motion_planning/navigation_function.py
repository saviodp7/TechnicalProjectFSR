import numpy as np
import timeit
from gridmap import GridMap, CELL_SIZE, OFFSET
import pygame
import sys


class NavigationFunction:
    def __init__(self, gridmap, start, goal):
        self.gridmap = gridmap
        self.start = start
        self.goal = goal

    def get_adj_orth_cells(self, cell_list, nav_map):
        adjacent_cells = list()
        for cell in cell_list:
            if cell[0] - 1 >= 0:
                adjacent_cells.append([cell[0] - 1, cell[1]])
            if cell[0] + 1 < np.shape(nav_map)[0]:
                adjacent_cells.append([cell[0] + 1, cell[1]])
            if cell[1] - 1 >= 0:
                adjacent_cells.append([cell[0], cell[1] - 1])
            if cell[1] + 1 < np.shape(nav_map)[1]:
                adjacent_cells.append([cell[0], cell[1] + 1])
        return adjacent_cells

    def get_adj_cells(self, cell_list, nav_map):
        adjacent_cells = list()
        for cell in cell_list:
            if cell[0] - 1 >= 0 and cell[1] - 1 >= 0:
                adjacent_cells.append([cell[0] - 1, cell[1] - 1])
            if cell[0] - 1 >= 0:
                adjacent_cells.append([cell[0] - 1, cell[1]])
            if cell[0] - 1 >= 0 and cell[1] + 1 < np.shape(nav_map)[1]:
                adjacent_cells.append([cell[0] - 1, cell[1] + 1])
            if cell[1] + 1 < np.shape(nav_map)[1]:
                adjacent_cells.append([cell[0], cell[1] + 1])
            if cell[1] + 1 < np.shape(nav_map)[1] and cell[0] + 1 < np.shape(nav_map)[0]:
                adjacent_cells.append([cell[0] + 1, cell[1] + 1])
            if cell[0] + 1 < np.shape(nav_map)[0]:
                adjacent_cells.append([cell[0] + 1, cell[1]])
            if cell[0] + 1 < np.shape(nav_map)[0] and cell[1] - 1 >= 0:
                adjacent_cells.append([cell[0] + 1, cell[1] - 1])
            if cell[1] - 1 >= 0:
                adjacent_cells.append([cell[0], cell[1] - 1])
        return adjacent_cells

    def compute_cost_map(self, cells, nav_map, exploration_func):
        nav_map[self.gridmap == 1] = -1
        nav_map[self.gridmap == 2] = -1
        while cells:
            cell = cells.pop(0)
            for adj_cell in exploration_func([cell], nav_map):
                if self.gridmap[adj_cell[0], adj_cell[1]] == 0 and nav_map[adj_cell[0], adj_cell[1]] == 0:
                    nav_map[adj_cell[0], adj_cell[1]] = nav_map[cell[0], cell[1]] + 1
                    cells.append(adj_cell)

    def steepest_descent(self, start_cell, nav_map, exploration_func):
        path = [start_cell]
        current_cell = start_cell
        while not np.array_equal(current_cell, self.goal):
            adjacent_cells = exploration_func([current_cell], nav_map)
            min_cost = float('inf')
            next_cell = current_cell
            for cell in adjacent_cells:
                if nav_map[cell[0], cell[1]] >= 0 and nav_map[cell[0], cell[1]] < min_cost:
                    min_cost = nav_map[cell[0], cell[1]]
                    next_cell = cell
            if np.array_equal(next_cell, current_cell):
                break
            path.append(next_cell)
            current_cell = next_cell
        return path

    def invert_grid_values(self):
        for y in range(self.shape[0]):
            for x in range(self.shape[1]):
                if self[y, x] in [1, 2]:  # Converts both 1 and 2 to 0
                    self[y, x] = 0
                elif self[y, x] == 0:
                    self[y, x] = 1
        return self

def main():
    gmap = GridMap(1.5, 1.5, 0.1)
    gmap.add_obstacle(3, 3, (4, 4))
    gmap.add_obstacle(40, 10, (110, 30))
    gmap.add_obstacle(10, 30, (50, 40))
    gmap.add_obstacle(10, 15, (10, 90))
    gmap.draw()


    start = [1, 1]
    goal = [10, 10]

    navigator = NavigationFunction(gmap, start, goal)

    # 4-ADJACENT EXPLORATION #
    print("4-Adjacent Exploration:")
    start = timeit.default_timer()
    num_nav_map = np.zeros_like(navigator.gridmap, dtype=int)
    num_nav_map[navigator.goal[0], navigator.goal[1]] = 1
    navigator.compute_cost_map([navigator.goal], num_nav_map, navigator.get_adj_orth_cells)
    num_nav_map[num_nav_map == 1] = 0
    num_nav_map[navigator.goal[0], navigator.goal[1]] = 0
    print(f'Computed cost map:\n{num_nav_map}\nExecution time: {timeit.default_timer() - start:.6f}s')

    start = timeit.default_timer()
    path = navigator.steepest_descent(navigator.start, num_nav_map, navigator.get_adj_orth_cells)
    print(f'Best path: {path}')
    print(f'Path length: {len(path) - 1}')
    print(f'Execution time: {timeit.default_timer() - start:.6f}s\n')




if __name__ == "__main__":
    main()

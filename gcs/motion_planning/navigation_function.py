import numpy as np
import timeit
from gridmap import GridMap

class NavigationFunction:
    def __init__(self, grid_map, q_start, q_goal):
        self.grid_map = grid_map
        self.q_start = q_start
        self.q_goal = q_goal
        self.compute_cost_map_counter = 0

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
        nav_map[self.grid_map == 0] = -1
        while cells:
            cell = cells.pop(0)
            for adj_cell in exploration_func([cell], nav_map):
                if self.grid_map[adj_cell[0], adj_cell[1]] == 1 and nav_map[adj_cell[0], adj_cell[1]] == 0:
                    nav_map[adj_cell[0], adj_cell[1]] = nav_map[cell[0], cell[1]] + 1
                    cells.append(adj_cell)

    def steepest_descent(self, start_cell, nav_map, exploration_func):
        path = [start_cell]
        current_cell = start_cell
        while not np.array_equal(current_cell, self.q_goal):
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

    def main(self, exploration_func):
        start = timeit.default_timer()
        num_nav_map = np.zeros_like(self.grid_map, dtype=int)
        num_nav_map[self.q_goal[0], self.q_goal[1]] = 1
        self.compute_cost_map([self.q_goal], num_nav_map, exploration_func)
        num_nav_map[num_nav_map == 1] = 0
        num_nav_map[self.q_goal[0], self.q_goal[1]] = 0
        print(f'Computed cost map:\n{num_nav_map}\nExecution time: {timeit.default_timer() - start:.6f}s')

        start = timeit.default_timer()
        path = self.steepest_descent(self.q_start, num_nav_map, exploration_func)
        print(f'Best path: {path}')
        print(f'Path length: {len(path) - 1}')
        print(f'Execution time: {timeit.default_timer() - start:.6f}s\n')


if __name__ == "__main__":
    gmap = GridMap(150, 150, 10)
    gmap.add_obstacle(3, 3, (4, 4))
    gmap.add_obstacle(40, 10, (110, 30))
    gmap.add_obstacle(10, 30, (50, 40))
    gmap.add_obstacle(10, 15, (10, 90))

    print("Grid Map (0=obstacle, 1=free space):")
    print(gmap.invert_grid_values())

    q_start = [14, 1]
    q_goal = [8, 0]

    navigator = NavigationFunction(gmap, q_start, q_goal)

    # 4-ADJACENT EXPLORATION #
    print("4-Adjacent Exploration:")
    navigator.main(navigator.get_adj_orth_cells)
    # 8-ADJACENT EXPLORATION #
    #print("8-Adjacent Exploration:")
    #navigator.main(navigator.get_adj_cells)

import numpy as np
import os
import sys
if __name__ == "__main__":
    from gridmap import GridMap, CELL_SIZE, OFFSET
else:
    from .gridmap import GridMap, CELL_SIZE, OFFSET
import pygame

NAV_4_ADJ = 1
NAV_8_ADJ = 2


class NavigationFunction:
    def __init__(self, gridmap, start, goal, mode=NAV_4_ADJ):
        self.gridmap = gridmap
        self.nav_map = np.zeros_like(self.gridmap, dtype=int)
        self.start = start
        self.goal = goal
        self.path_cost = 0
        self.steep_desc = 0

        if mode == NAV_4_ADJ:
            self.exploration_func = self.get_adj_orth_cells
        elif mode == NAV_8_ADJ:
            self.exploration_func = self.get_adj_cells
        else:
            raise ValueError("Exploration function not valid!")
        adj_start_cell = self.exploration_func([self.goal])
        self.nav_map[self.gridmap != 0] = -1
        self.compute_cost_map(adj_start_cell)
        self.nav_map[self.goal[0], self.goal[1]] = 0
        self.best_path = [self.start]
        self.steepest_descent(self.start)
        print(self.best_path)

    def get_adj_orth_cells(self, cell_list):
        adjacent_cells = list()
        for cell in cell_list:
            if cell[0] - 1 >= 0:
                adjacent_cells.append([cell[0] - 1, cell[1]])
            if cell[0] + 1 < np.shape(self.nav_map)[0]:
                adjacent_cells.append([cell[0] + 1, cell[1]])
            if cell[1] - 1 >= 0:
                adjacent_cells.append([cell[0], cell[1] - 1])
            if cell[1] + 1 < np.shape(self.nav_map)[1]:
                adjacent_cells.append([cell[0], cell[1] + 1])
        return adjacent_cells

    def get_adj_cells(self, cell_list):
        adjacent_cells = list()
        for cell in cell_list:
            if cell[0] - 1 >= 0 and cell[1] - 1 >= 0:
                adjacent_cells.append([cell[0] - 1, cell[1] - 1])
            if cell[0] - 1 >= 0:
                adjacent_cells.append([cell[0] - 1, cell[1]])
            if cell[0] - 1 >= 0 and cell[1] + 1 < np.shape(self.nav_map)[1]:
                adjacent_cells.append([cell[0] - 1, cell[1] + 1])
            if cell[1] + 1 < np.shape(self.nav_map)[1]:
                adjacent_cells.append([cell[0], cell[1] + 1])
            if cell[1] + 1 < np.shape(self.nav_map)[1] and cell[0] + 1 < np.shape(self.nav_map)[0]:
                adjacent_cells.append([cell[0] + 1, cell[1] + 1])
            if cell[0] + 1 < np.shape(self.nav_map)[0]:
                adjacent_cells.append([cell[0] + 1, cell[1]])
            if cell[0] + 1 < np.shape(self.nav_map)[0] and cell[1] - 1 >= 0:
                adjacent_cells.append([cell[0] + 1, cell[1] - 1])
            if cell[1] - 1 >= 0:
                adjacent_cells.append([cell[0], cell[1] - 1])
        return adjacent_cells

    def compute_cost_map(self, cells):
        self.path_cost += 1
        to_explore = list()
        if 0 in self.nav_map:
            while cells:
                cell = cells.pop()
                if self.gridmap[cell[0], cell[1]] == 0 and self.nav_map[cell[0], cell[1]] == 0:
                    self.nav_map[cell[0], cell[1]] = self.path_cost
                    to_explore.append(cell)
            adjacent_cells = self.exploration_func(to_explore)
            self.compute_cost_map(adjacent_cells)

    def steepest_descent(self, start_cell):
        self.steep_desc += 1
        feasible_cells = dict()
        if not self.nav_map[start_cell[0], start_cell[1]] == 0:
            adjacent_cells = self.exploration_func([start_cell])
            for cell in adjacent_cells:
                if self.nav_map[cell[0], cell[1]] >= 0:
                    feasible_cells[self.nav_map[cell[0], cell[1]]] = cell
            best_cell = feasible_cells[min(feasible_cells.keys())]
            self.best_path.append(tuple(best_cell))
            self.steepest_descent(best_cell)

    def draw(self, screen):
        max_cost = np.max(self.nav_map)

        for y in range(self.nav_map.shape[0]):
            for x in range(self.nav_map.shape[1]):
                cost = self.nav_map[y, x]
                if cost > 0:
                    # Calcola l'intensità invertita: valori alti -> più chiaro (vicino al bianco)
                    intensity = int(255 - (cost / max_cost) * 255)
                    color = (intensity, intensity, intensity)
                    rect = pygame.Rect(x * CELL_SIZE, y * CELL_SIZE, CELL_SIZE, CELL_SIZE)
                    pygame.draw.rect(screen, color, rect)

        # Disegna il percorso migliore in rosso con dimensioni più piccole e offset
        for (x, y) in self.best_path:
            rect = pygame.Rect(y * CELL_SIZE + OFFSET/2, x * CELL_SIZE + OFFSET/2, CELL_SIZE / 2, CELL_SIZE / 2)
            pygame.draw.rect(screen, (255, 0, 0), rect)


def main():
    os.environ['SDL_VIDEO_WINDOW_POS'] = "%d,%d" % (100, 100)

    gmap = GridMap(1, 1.5, 0.01)
    gmap.add_obstacle(30, 30, (30, 30))
    gmap.draw()
    bg = pygame.image.load('gridmap.png')
    start = (30, 20)
    # goal = (gmap.shape[0]-1, gmap.shape[1]-1)
    goal = (70, 120)
    navigator = NavigationFunction(gmap, start, goal, mode=NAV_8_ADJ)

    # Setup app
    pygame.init()
    pygame.display.set_caption('Motion Planning Graph')
    width, height = gmap.shape
    screen = pygame.display.set_mode((height * CELL_SIZE, width * CELL_SIZE))

    clock = pygame.time.Clock()
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

        # Background
        screen.blit(pygame.transform.scale(bg, (height * CELL_SIZE, width * CELL_SIZE)), (0, 0))
        navigator.draw(screen)
        pygame.draw.circle(screen, 'orange', (start[1] * CELL_SIZE + OFFSET, start[0] * CELL_SIZE + OFFSET), 5)
        pygame.draw.circle(screen, 'green', (goal[1] * CELL_SIZE + OFFSET, goal[0] * CELL_SIZE + OFFSET), 5)
        # Update screen

        clock.tick(30)  # Limit to 30 FPS
        pygame.display.flip()


if __name__ == "__main__":
    main()

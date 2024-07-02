import numpy as np
import pygame
import random
from scipy.spatial import KDTree
from gridmap import GridMap
import heapq


class MotionPlanning:

    def __init__(self, gridmap):
        self.gridmap = gridmap
        self.graph = None
        self.nodes = []
        self.edges = []

    def is_free(self, point):
        x, y = point
        return self.gridmap[y, x] == 0

    def get_random_point(self):
        x = random.randint(0, self.gridmap.shape[1] - 1)
        y = random.randint(0, self.gridmap.shape[0] - 1)
        return x, y

    def nearest_node(self, point):
        tree = KDTree(self.nodes)
        dist, index = tree.query(point)
        return self.nodes[index]

    def prm(self, num_samples=100, k=5):
        self.nodes = []
        while len(self.nodes) < num_samples:
            point = self.get_random_point()
            if self.is_free(point):
                self.nodes.append(point)

        self.edges = []
        tree = KDTree(self.nodes)
        for node in self.nodes:
            dists, indices = tree.query(node, k=k + 1)
            for dist, index in zip(dists[1:], indices[1:]):
                neighbor = self.nodes[index]
                if self.is_free(neighbor):
                    self.edges.append((node, neighbor))

        self.graph = (self.nodes, self.edges)
        return self.graph

    def rrt(self, start, goal, max_iteration=300, delta=2):
        n_rows, n_columns = self.gridmap.shape
        qi = np.array(start)
        qf = np.array(goal)

        # Assicurati che i punti iniziale e finale siano all'interno della griglia
        if not self.is_point_in_grid(start) or not self.is_point_in_grid(goal):
            print("Start or goal point is outside the grid.")
            return

        self.nodes = [start]
        self.edges = []
        goal_reached = False

        for iter in range(max_iteration):
            q_rand = np.array([random.randint(0, n_rows - 1), random.randint(0, n_columns - 1)])

            distances = np.linalg.norm(np.array(self.nodes) - q_rand, axis=1)
            nearest_node_idx = np.argmin(distances)
            q_near = np.array(self.nodes[nearest_node_idx])

            theta = np.arctan2(q_rand[0] - q_near[0], q_rand[1] - q_near[1])
            q_new = np.array([int(q_near[0] + delta * np.sin(theta)), int(q_near[1] + delta * np.cos(theta))])

            # Assicurati che il nuovo punto sia all'interno della griglia
            if not self.is_point_in_grid(q_new) or not self.is_free(q_new):
                continue

            line = np.round(np.linspace(q_near, q_new, 100)).astype(int)
            collision = any(
                self.gridmap[p[0], p[1]] == 1 for p in line if 0 <= p[0] < n_rows and 0 <= p[1] < n_columns)

            if not collision:
                self.nodes.append(tuple(q_new))
                self.edges.append((tuple(q_near), tuple(q_new)))

                if np.linalg.norm(q_new - qf) <= delta:
                    self.nodes.append(tuple(goal))
                    self.edges.append((tuple(q_new), tuple(goal)))
                    goal_reached = True
                    break

        if not goal_reached:
            print("Goal not reached within maximum iterations.")

        self.graph = (self.nodes, self.edges)
        return self.graph

    def is_point_in_grid(self, point):
        x, y = point
        return 0 <= x < self.gridmap.shape[1] and 0 <= y < self.gridmap.shape[0]

    def a_star_search(self, start, goal):
        start, goal = tuple(start), tuple(goal)
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {node: float('inf') for node in self.nodes}
        g_score[start] = 0
        f_score = {node: float('inf') for node in self.nodes}
        f_score[start] = self.heuristic(start, goal)

        while open_set:
            current = heapq.heappop(open_set)[1]

            if current == goal:
                return self.reconstruct_path(came_from, current)

            for neighbor in self.get_neighbors(current):
                tentative_g_score = g_score[current] + self.distance(current, neighbor)

                if tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    if neighbor not in [i[1] for i in open_set]:
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return []

    def heuristic(self, a, b):
        return np.linalg.norm(np.array(a) - np.array(b))

    def distance(self, a, b):
        return np.linalg.norm(np.array(a) - np.array(b))

    def get_neighbors(self, node):
        return [neighbor for neighbor in self.nodes if (node, neighbor) in self.edges or (neighbor, node) in self.edges]

    def reconstruct_path(self, came_from, current):
        total_path = [current]
        while current in came_from:
            current = came_from[current]
            total_path.append(current)
        return total_path[::-1]

    def draw_graph(self):
        pygame.init()
        cell_size = 7
        toolbar_height = 20
        screen_width = self.gridmap.shape[1] * cell_size
        screen_height = self.gridmap.shape[0] * cell_size + toolbar_height

        screen = pygame.display.set_mode((screen_width, screen_height))
        pygame.display.set_caption('Motion Planning Graph')

        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    mouse_x, mouse_y = event.pos
                    if mouse_y < toolbar_height:
                        if 0 <= mouse_x <= 200:
                            self.gridmap.add_obstacle(10, 10, (30, 70))
                        elif 200 < mouse_x <= 400:
                            self.gridmap.inflate_obstacle(1, 5)

            screen.fill((255, 255, 255))

            # Draw the grid
            for y in range(self.gridmap.shape[0]):
                for x in range(self.gridmap.shape[1]):
                    pygame.draw.rect(screen, (200, 200, 200),
                                     (x * cell_size, y * cell_size + toolbar_height, cell_size, cell_size), 1)

            # Draw obstacles and inflated areas
            for y in range(self.gridmap.shape[0]):
                for x in range(self.gridmap.shape[1]):
                    if self.gridmap[y, x] == 1:
                        pygame.draw.rect(screen, (0, 0, 0),
                                         (x * cell_size, y * cell_size + toolbar_height, cell_size, cell_size))
                    elif self.gridmap[y, x] == 2:
                        pygame.draw.rect(screen, (255, 0, 0),
                                         (x * cell_size, y * cell_size + toolbar_height, cell_size, cell_size))

            # Draw nodes
            for node in self.nodes:
                x, y = node
                pygame.draw.circle(screen, (0, 0, 255), (x * cell_size, y * cell_size + toolbar_height), 3)

            # Draw edges
            for edge in self.edges:
                (x1, y1), (x2, y2) = edge
                pygame.draw.line(screen, (0, 0, 0), (x1 * cell_size, y1 * cell_size + toolbar_height),
                                 (x2 * cell_size, y2 * cell_size + toolbar_height))

            pygame.display.flip()

        pygame.quit()

def main():
    gmap = GridMap(1, 1, 0.01)
    gmap.add_obstacle(10, 10, (30, 70))
    gmap.add_obstacle(5, 20, (20, 20))

    gmap.inflate_obstacle(1, 2)
    gmap.inflate_obstacle(2, 3)

    motion_planning = MotionPlanning(gmap)

    # Scegli l'algoritmo
    choice = "2"
    if choice == "1":
        # Genera e disegna il grafo PRM
        motion_planning.prm(num_samples=100, k=5)
        motion_planning.draw_graph()
    elif choice == "2":
        start = (1, 1)  # Punti iniziali e finali scelti all'interno della griglia
        goal = (99, 99)
        # Genera e disegna il grafo RRT
        motion_planning.rrt(start, goal, max_iteration=300, delta=10)
        motion_planning.draw_graph()
    else:
        print("Scelta non valida!")

if __name__ == "__main__":
    main()


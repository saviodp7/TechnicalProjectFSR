import pygame
import random
import heapq
import sys
import numpy as np
from scipy.spatial import KDTree
from gridmap import GridMap, CELL_SIZE, OFFSET

NODE_GEN_PRM = 0
NODE_GEN_RRT = 1


class MotionPlanning:

    def __init__(self, gridmap):
        self.gridmap = gridmap
        self.graph = None
        self.nodes = []
        self.edges = []

    def is_free(self, point):
        x, y = point
        return self.gridmap[y, x] == 0

    def is_path_free(self, point1, point2, num_checks=100):
        x1, y1 = point1
        x2, y2 = point2
        for i in range(num_checks + 1):
            t = i / num_checks
            x = int(x1 + t * (x2 - x1))
            y = int(y1 + t * (y2 - y1))
            if not self.is_free((x, y)):
                return False
        return True

    def is_point_in_grid(self, point):
        x, y = point
        return 0 <= x < self.gridmap.shape[1] and 0 <= y < self.gridmap.shape[0]

    def get_random_point(self):
        x = random.randint(0, self.gridmap.shape[1]-1)
        y = random.randint(0, self.gridmap.shape[0]-1)
        return x, y

    def get_neighbors(self, node):
        return [neighbor for neighbor in self.nodes if (node, neighbor) in self.edges or (neighbor, node) in self.edges]

    def prm(self, start, goal, num_samples=100, k=5):
        # TODO: Possono esserci in rari casi dei path non feasible aumentare i punti nel caso questo accada
        """
        Crea una serie di nodi per la navigazione con l'algoritmo PRM

        Args:
            start (int, int): cella di partenza.
            goal (int, int): cella di arrivo.
            num_samples (int): numero di punti da generare.
            k (float): numero di archi da ogni punto.

        Returns:
            KDTree: Grafo di navigazione
            """
        self.nodes = []
        self.edges = []
        if not self.is_point_in_grid(start) or not self.is_point_in_grid(goal):
            raise ValueError("Start or goal point is outside the grid.")
        # Generate all points
        while len(self.nodes) < num_samples:
            point = self.get_random_point()
            if self.is_free(point):
                self.nodes.append(point)
        tree = KDTree(self.nodes)
        connections = {i: [] for i in range(len(self.nodes))}
        for i, node in enumerate(self.nodes):
            dists, indices = tree.query(node, k=k+1)
            for j in range(1, int(k+1)):
                nearest_neighbor_index = indices[j]
                nearest_neighbor = self.nodes[nearest_neighbor_index]
                if nearest_neighbor not in connections[i] and self.is_path_free(node, nearest_neighbor):
                    self.edges.append((node, nearest_neighbor))
                    connections[i].append(nearest_neighbor)
                    connections[int(nearest_neighbor_index)].append(node)
        # Add start and goal
        self.nodes.append(start)
        self.nodes.append(goal)
        # Connect start
        dists, indices = tree.query(start, k=k+1)
        for j in range(int(k)):
            nearest_neighbor_index = indices[j]
            nearest_neighbor = self.nodes[nearest_neighbor_index]
            if self.is_path_free(start, nearest_neighbor):
                self.edges.append((start, nearest_neighbor))
                break  # Ensure only one connection
        # Connect goal
        dists, indices = tree.query(goal, k=k + 1)
        for j in range(int(k)):
            nearest_neighbor_index = indices[j]
            nearest_neighbor = self.nodes[nearest_neighbor_index]
            if self.is_path_free(goal, nearest_neighbor):
                self.edges.append((goal, nearest_neighbor))
                break
        self.graph = (self.nodes, self.edges)
        return self.graph

    def rrt(self, start, goal, iteration_increment=100, delta=2):
        qi = np.array(start)
        qf = np.array(goal)

        if not self.is_point_in_grid(start) or not self.is_point_in_grid(goal):
            raise ValueError("Start or goal point is outside the grid.")

        self.nodes = [start]
        self.edges = []
        goal_reached = False

        while not goal_reached:
            for _ in range(iteration_increment):
                q_rand = self.get_random_point()

                distances = np.linalg.norm(np.array(self.nodes) - q_rand, axis=1)
                nearest_node_idx = np.argmin(distances)
                q_near = np.array(self.nodes[nearest_node_idx])

                theta = np.arctan2(q_rand[0] - q_near[0], q_rand[1] - q_near[1])
                q_new = np.array([int(q_near[0] + delta * np.sin(theta)), int(q_near[1] + delta * np.cos(theta))])

                # Check if the q_new is inside the grid
                if not self.is_point_in_grid(q_new) or not self.is_free(q_new):
                    continue
                if self.is_path_free(tuple(q_near), tuple(q_new)):
                    self.nodes.append(tuple(q_new))
                    self.edges.append((tuple(q_near), tuple(q_new)))
                if np.linalg.norm(q_new - qf) <= delta:
                    if self.is_path_free(tuple(q_new), tuple(goal)):
                        self.nodes.append(tuple(goal))
                        self.edges.append((tuple(q_new), tuple(goal)))
                        goal_reached = True
                        break
        self.graph = (self.nodes, self.edges)
        return self.graph

    def bfs(self, start, goal):
        start, goal = tuple(start), tuple(goal)
        queue = [start]
        came_from = {}
        visited = set()
        visited.add(start)
        while queue:
            current = queue.pop(0)
            if current == goal:
                return self.reconstruct_path(came_from, current)
            for neighbor in self.get_neighbors(current):
                if neighbor not in visited:
                    visited.add(neighbor)
                    came_from[neighbor] = current
                    queue.append(neighbor)
        return []

    @staticmethod
    def reconstruct_path(came_from, current):
        total_path = [current]
        while current in came_from:
            current = came_from[current]
            total_path.append(current)
        return total_path[::-1]

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
                    # f_score[neighbor] = tentative_g_score + self.distance(current, neighbor)
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    if neighbor not in [i[1] for i in open_set]:
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))
        return []

    @staticmethod
    def heuristic(a, b):
        return np.linalg.norm(np.array(a) - np.array(b))

    @staticmethod
    def distance(a, b):
        return np.linalg.norm(np.array(a) - np.array(b))

    def draw_graph(self, screen):
        # Draw nodes
        for node in self.nodes:
            x, y = node
            pygame.draw.circle(screen, (0, 0, 255), (x * CELL_SIZE+OFFSET, y * CELL_SIZE+OFFSET), 3)
        # Draw edges
        for edge in self.edges:
            (x1, y1), (x2, y2) = edge
            pygame.draw.line(screen, (0, 0, 0), (x1 * CELL_SIZE+OFFSET, y1 * CELL_SIZE+OFFSET),
                             (x2 * CELL_SIZE+OFFSET, y2 * CELL_SIZE+OFFSET))

    @staticmethod
    def draw_path(screen, path, color='red'):
        for i in range(len(path) - 1):
            (x1, y1), (x2, y2) = path[i], path[i + 1]
            pygame.draw.line(screen, color, (x1 * CELL_SIZE+OFFSET, y1 * CELL_SIZE+OFFSET),
                             (x2 * CELL_SIZE+OFFSET, y2 * CELL_SIZE+OFFSET), 3)


def main(node_generation=NODE_GEN_PRM):

    # Initialization gridmap and Motion planning
    gmap = GridMap(1, 1.5, 0.01)
    gmap.add_obstacle(30, 30, (15, 15))
    gmap.inflate_obstacle(1, 3)
    gmap.draw()
    motion_planning = MotionPlanning(gmap)
    # Load background image
    bg = pygame.image.load('gridmap.png')

    # Setup app
    pygame.init()
    pygame.display.set_caption('Motion Planning Graph')
    width, height = gmap.shape
    screen = pygame.display.set_mode((height * CELL_SIZE, width * CELL_SIZE))

    start = (0, 0)
    goal = (gmap.shape[1]-1, gmap.shape[0]-1)

    path = None
    if node_generation == NODE_GEN_PRM:
        motion_planning.prm(start, goal, num_samples=100, k=5)
        path = motion_planning.bfs(start, goal)
        print(f"BFS Path: {path}")
        path_star = motion_planning.a_star_search(start, goal)
        print(f"A* Path: {path_star}")
    elif node_generation == NODE_GEN_RRT:
        motion_planning.rrt(start, goal, iteration_increment=100, delta=5)
        path = motion_planning.bfs(start, goal)
        print(f"BFS Path: {path}")
        path_star = motion_planning.a_star_search(start, goal)
        print(f"A* Path: {path_star}")
    else:
        raise ValueError("Not valid node generation algorithm!")

    while True:
        for evento in pygame.event.get():
            if evento.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

        # Background
        screen.blit(bg, (0, 0))
        pygame.draw.circle(screen, 'orange', (start[0] * CELL_SIZE+OFFSET, start[1] * CELL_SIZE+OFFSET), 5)
        pygame.draw.circle(screen, 'green', (goal[0] * CELL_SIZE+OFFSET, goal[1] * CELL_SIZE+OFFSET), 5)
        motion_planning.draw_graph(screen)
        if path:
            motion_planning.draw_path(screen, path, 'blue')
            motion_planning.draw_path(screen, path_star, 'red')
        # Update screen
        pygame.display.flip()


if __name__ == "__main__":
    main(NODE_GEN_RRT)

import pygame
import sys
from motion_planning.gridmap import GridMap, CELL_SIZE, OFFSET
from motion_planning.motion_planner import MotionPlanner, NODE_GEN_PRM, NODE_GEN_RRT


def main(node_generation=NODE_GEN_PRM):
    # Initialization gridmap and Motion planning
    gmap = GridMap(1, 1.5, 0.01)
    gmap.add_obstacle(10, 15, (15, 15))
    gmap.inflate_obstacle(1, 3)
    gmap.add_obstacle(40, 10, (110, 30))
    gmap.inflate_obstacle(2, 3)
    gmap.add_obstacle(10, 30, (50, 40))
    gmap.inflate_obstacle(3, 4)
    gmap.add_obstacle(10, 15, (10, 90))
    gmap.inflate_obstacle(4, 4)
    gmap.draw()
    motion_planner = MotionPlanner(gmap)
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
        motion_planner.prm(start, goal, num_samples=100, k=5)
        path = motion_planner.bfs(start, goal)
        print(f"BFS Path: {path}")
        path_star = motion_planner.a_star_search(start, goal)
        print(f"A* Path: {path_star}")
    elif node_generation == NODE_GEN_RRT:
        motion_planner.rrt(start, goal, iteration_increment=100, delta=10)
        path = motion_planner.bfs(start, goal)
        print(f"BFS Path: {path}")
        path_star = motion_planner.a_star_search(start, goal)
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
        motion_planner.draw_graph(screen)
        motion_planner.draw_path(screen, path, 'blue')
        motion_planner.draw_path(screen, path_star, 'red')
        # Update screen
        pygame.display.flip()


if __name__ == "__main__":
    main(NODE_GEN_PRM)

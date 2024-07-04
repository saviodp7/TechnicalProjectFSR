import pygame
import sys
from motion_planning.gridmap import GridMap, CELL_SIZE, OFFSET
from motion_planning.motion_planner import MotionPlanner, NODE_GEN_PRM, NODE_GEN_RRT
from trajectory_planning.trajectory_planner import TrajectoryPlanner, LINEAR_PROF, TRAP_VEL_PROF, CUBIC_POL_PROF
import matplotlib.pyplot as plt


def main():
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
    # Load background image
    bg = pygame.image.load('gridmap.png')

    # Setup app
    pygame.init()
    pygame.display.set_caption('Motion Planning Graph')
    width, height = gmap.shape
    screen = pygame.display.set_mode((height * CELL_SIZE, width * CELL_SIZE))

    start = (0, 0)
    goal = (gmap.shape[1] - 1, gmap.shape[0] - 1)
    motion_planner = MotionPlanner(gmap, NODE_GEN_PRM, start, goal, num_samples=100, k=5)

    trplanner = TrajectoryPlanner(gmap, path_list=motion_planner.a_star_path)
    x, y, x_dot, y_dot, theta, theta_dot = trplanner.cartesian_traj(f_s=20, profile=CUBIC_POL_PROF)

    # Modalità interattiva per Matplotlib
    plt.ion()

    plt.plot(x, [-elem for elem in y], color='blue', marker='o', linewidth=1, markersize=3)
    plt.xlabel('x (m)')
    plt.ylabel('y (m)')
    plt.title('Path')
    plt.draw()

    while True:
        for evento in pygame.event.get():
            if evento.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

        # Background
        screen.blit(bg, (0, 0))
        pygame.draw.circle(screen, 'orange', (start[0] * CELL_SIZE + OFFSET, start[1] * CELL_SIZE + OFFSET), 5)
        pygame.draw.circle(screen, 'green', (goal[0] * CELL_SIZE + OFFSET, goal[1] * CELL_SIZE + OFFSET), 5)
        motion_planner.draw_graph(screen)
        motion_planner.draw_path(screen, 'bfs', 'blue')
        motion_planner.draw_path(screen, 'a_star', 'red')
        # Update screen
        pygame.display.flip()

        # Pausa per aggiornare Matplotlib
        plt.pause(0.001)


if __name__ == "__main__":
    main()

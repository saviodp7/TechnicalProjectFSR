import pygame
import sys
from motion_planning.gridmap import GridMap, CELL_SIZE, OFFSET


# Initialization gridmap
gmap = GridMap(1, 1.5, 0.01)
gmap.add_obstacle(30, 30, (15, 15))
gmap.inflate_obstacle(1, 3)
cell_size = CELL_SIZE
offset = cell_size/2
# Load background image
bg = pygame.image.load('gridmap.png')

# Setup screen
pygame.init()
pygame.display.set_caption('GridMap')
width, height = gmap.shape
screen = pygame.display.set_mode((height*cell_size, width*cell_size))

# Posizioni dei punti
punti = [(100-OFFSET, 100-OFFSET), (700-OFFSET, 300-OFFSET), (400-OFFSET, 500-OFFSET)]

# Funzione principale
def main():
    while True:
        for evento in pygame.event.get():
            if evento.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

        # Background
        screen.blit(bg, (0, 0))

        # Disegna i punti rossi
        for punto in punti:
            pygame.draw.circle(screen, (255, 0, 0), punto, 5)  # Disegna un cerchio con raggio 5

        # Aggiorna la schermata
        pygame.display.flip()


if __name__ == "__main__":
    main()

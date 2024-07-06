import pygame
import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel, QLineEdit, QComboBox, QPushButton
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QFont
from motion_planning.gridmap import GridMap, CELL_SIZE, OFFSET
from motion_planning.motion_planner import MotionPlanner, NODE_GEN_PRM
from trajectory_planning.trajectory_planner import TrajectoryPlanner, CUBIC_POL_PROF
from communication.bluetooth import BluetoothInterface

class BluetoothWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.bluetooth = BluetoothInterface()

        self.setWindowTitle('Bluetooth Data Sender')
        self.setGeometry(100, 100, 500, 400)  # Imposta la dimensione della finestra

        # Creazione del widget principale e layout
        self.window = QWidget()
        self.setCentralWidget(self.window)
        self.workspace = QVBoxLayout()
        self.window.setLayout(self.workspace)

        # Widget per l'odometria e relativo layout
        self.odometria = QWidget()
        self.odometria.setStyleSheet("#odometria { border: 2px solid black; }")
        self.odometria.setObjectName("odometria")
        self.odometria_layout = QHBoxLayout()
        self.odometria.setLayout(self.odometria_layout)
        self.workspace.addWidget(self.odometria)

        # Widget per le informazioni di posizione e relativo layout
        self.odometria_posizione = QWidget()
        self.odometria_posizione_layout = QVBoxLayout()
        self.odometria_posizione.setLayout(self.odometria_posizione_layout)
        self.odometria_layout.addWidget(self.odometria_posizione)

        # Aggiunta delle etichette di posizione
        self.odo_x_label = QLabel("x: 0.0")
        font = self.odo_x_label.font()
        font.setPointSize(10)
        self.odo_x_label.setFont(font)
        self.odometria_posizione_layout.addWidget(self.odo_x_label)

        self.odo_y_label = QLabel("y: 0.0")
        font = self.odo_y_label.font()
        font.setPointSize(10)
        self.odo_y_label.setFont(font)
        self.odometria_posizione_layout.addWidget(self.odo_y_label)

        self.odo_theta_label = QLabel("θ: 0.0")
        font = self.odo_theta_label.font()
        font.setPointSize(10)
        self.odo_theta_label.setFont(font)
        self.odometria_posizione_layout.addWidget(self.odo_theta_label)

        # Widget per i controlli e relativo layout
        self.odometria_controlli = QWidget()
        self.odometria_controlli_layout = QVBoxLayout()
        self.odometria_controlli.setLayout(self.odometria_controlli_layout)
        self.odometria_layout.addWidget(self.odometria_controlli)

        # Etichette per i controlli
        self.cont_v_label = QLabel("v: 0.0")
        font = self.cont_v_label.font()
        font.setPointSize(10)
        self.cont_v_label.setFont(font)
        self.odometria_controlli_layout.addWidget(self.cont_v_label)

        self.cont_w_label = QLabel("ω: 0.0")
        font = self.cont_w_label.font()
        font.setPointSize(10)
        self.cont_w_label.setFont(font)
        self.odometria_controlli_layout.addWidget(self.cont_w_label)

        # ComboBox per la selezione dei controlli
        self.control_combo = QComboBox()
        self.control_combo.addItem('I/0 Linearization')
        self.control_combo.addItem('Posture regulation')
        combo_font = QFont("Arial", 14)  # Puoi scegliere un'altra dimensione e font se desideri
        self.control_combo.setFont(combo_font)
        self.control_combo.activated[str].connect(self.onChanged)
        self.workspace.addWidget(self.control_combo)

        self.obstacle = QWidget()
        self.obstacle.setStyleSheet("#obstacle { border: 2px solid black; }")
        self.obstacle.setObjectName("obstacle")
        self.obstacle_layout = QGridLayout()
        self.obstacle.setLayout(self.obstacle_layout)
        self.workspace.addWidget(self.obstacle)

        self.obs_x_label = QLabel("x:")
        font = self.obs_x_label.font()
        font.setPointSize(10)
        self.obs_x_label.setFont(font)
        self.obstacle_layout.addWidget(self.obs_x_label, 0, 0)

        self.obs_x_entry = QLineEdit()
        self.obs_x_entry.setPlaceholderText("0")
        font = self.obs_x_entry.font()
        font.setPointSize(10)
        self.obs_x_entry.setFont(font)
        self.obstacle_layout.addWidget(self.obs_x_entry, 0, 1)

        self.obs_y_label = QLabel("y:")
        font = self.obs_y_label.font()
        font.setPointSize(10)
        self.obs_y_label.setFont(font)
        self.obstacle_layout.addWidget(self.obs_y_label, 0, 2)

        self.obs_y_entry = QLineEdit()
        self.obs_y_entry.setPlaceholderText("0")
        font = self.obs_y_entry.font()
        font.setPointSize(10)
        self.obs_y_entry.setFont(font)
        self.obstacle_layout.addWidget(self.obs_y_entry, 0, 3)

        self.obs_w_label = QLabel("width:")
        font = self.obs_w_label.font()
        font.setPointSize(10)
        self.obs_w_label.setFont(font)
        self.obstacle_layout.addWidget(self.obs_w_label, 1, 0)

        self.obs_w_entry = QLineEdit()
        self.obs_w_entry.setPlaceholderText("0")
        font = self.obs_w_entry.font()
        font.setPointSize(10)
        self.obs_w_entry.setFont(font)
        self.obstacle_layout.addWidget(self.obs_w_entry, 1, 1)

        self.obs_h_label = QLabel("height:")
        font = self.obs_h_label.font()
        font.setPointSize(10)
        self.obs_h_label.setFont(font)
        self.obstacle_layout.addWidget(self.obs_h_label, 1, 2)

        self.obs_h_entry = QLineEdit()
        self.obs_h_entry.setPlaceholderText("0")
        font = self.obs_h_entry.font()
        font.setPointSize(10)
        self.obs_h_entry.setFont(font)
        self.obstacle_layout.addWidget(self.obs_h_entry, 1, 3)

        self.obs_id_label = QLabel("id:")
        font = self.obs_id_label.font()
        font.setPointSize(10)
        self.obs_id_label.setFont(font)
        self.obstacle_layout.addWidget(self.obs_id_label, 2, 0)

        self.obs_id_entry = QLineEdit()
        self.obs_id_entry.setPlaceholderText("0")
        font = self.obs_id_entry.font()
        font.setPointSize(10)
        self.obs_id_entry.setFont(font)
        self.obstacle_layout.addWidget(self.obs_id_entry, 2, 1)

        self.add_obstacle_button = QPushButton('+')
        self.add_obstacle_button.clicked.connect(self.handle_add_obstacle_click)
        self.obstacle_layout.addWidget(self.add_obstacle_button, 2, 2)

        self.delete_obstacle_button = QPushButton('+')
        self.delete_obstacle_button.clicked.connect(self.handle_delete_obstacle_click)
        self.obstacle_layout.addWidget(self.delete_obstacle_button, 2, 3)

        # Timer per aggiornare i dati
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_data)
        self.timer.start(200)  # 200 ms per ottenere 5 aggiornamenti al secondo

    def onChanged(self, text):
        print(f'Selected control: {text}')

    def handle_add_obstacle_click(self):
        print(f'handle_add_obstacle_click')

    def handle_delete_obstacle_click(self):
        print(f'handle_delete_obstacle_click')

    def update_data(self):
        # Recupera i dati ricevuti tramite Bluetooth
        try:
            x, y, theta = self.bluetooth.last_message[0:3]  # Assicurati che il metodo esista e ritorni questi dati
            self.odo_x_label.setText(f"x: {x:.1f}")
            self.odo_y_label.setText(f"y: {y:.1f}")
            self.odo_theta_label.setText(f"θ: {theta:.1f}")
        except Exception as e:
            pass


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
    font = pygame.font.Font(None, 36)

    start = (0, 0)
    goal = (gmap.shape[1] - 1, gmap.shape[0] - 1)
    motion_planner = MotionPlanner(gmap, NODE_GEN_PRM, start, goal, num_samples=100, k=5)

    # Create text elements
    a_star_text = font.render('A*', True, (255, 0, 0))  # Red color for A*
    a_star_text_rect = a_star_text.get_rect()
    a_star_text_rect.topleft = (screen.get_width() - 60, 5)
    bfs_text = font.render('BFS', True, (0, 0, 255))  # Blue color for BFS
    bfs_text_rect = bfs_text.get_rect()
    bfs_text_rect.topleft = (screen.get_width() - 60, 25)

    trplanner = TrajectoryPlanner(gmap, path_list=motion_planner.a_star_path)
    x, y, x_dot, y_dot, theta, theta_dot = trplanner.cartesian_traj(f_s=20, profile=CUBIC_POL_PROF)

    # Create and show the PyQt application
    app = QApplication(sys.argv)
    bt_window = BluetoothWindow()
    bt_window.show()

    clock = pygame.time.Clock()

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

        # Run the PyQt event loop
        app.processEvents()

        # Background
        screen.blit(pygame.transform.scale(bg, (height * CELL_SIZE, width * CELL_SIZE)), (0, 0))
        pygame.draw.circle(screen, 'orange', (start[0] * CELL_SIZE + OFFSET, start[1] * CELL_SIZE + OFFSET), 5)
        pygame.draw.circle(screen, 'green', (goal[0] * CELL_SIZE + OFFSET, goal[1] * CELL_SIZE + OFFSET), 5)
        motion_planner.draw_graph(screen)
        motion_planner.draw_path(screen, 'bfs', 'blue')
        motion_planner.draw_path(screen, 'a_star', 'red')

        # Draw the text
        screen.blit(a_star_text, a_star_text_rect)
        screen.blit(bfs_text, bfs_text_rect)
        pygame.display.flip()

        clock.tick(30)  # Limit to 30 FPS


if __name__ == "__main__":
    main()

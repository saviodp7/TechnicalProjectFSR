import pygame
import sys
from PyQt5 import QtCore
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel, QLineEdit, QComboBox, QPushButton
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QFont
from motion_planning.gridmap import GridMap, CELL_SIZE, OFFSET
from motion_planning.motion_planner import MotionPlanner, NODE_GEN_PRM, NODE_GEN_RRT
from trajectory_planning.trajectory_planner import TrajectoryPlanner, CUBIC_POL_PROF
from communication.bluetooth import BluetoothInterface
import os

IO_LINEARIZATION = 1
POSTURE_REGULATION = 2

# Gridmap
gmap = GridMap(1.5, 1, 0.01)
# gmap.add_obstacle(0.22, 0.22, (0.12, 0.46))
# gmap.add_obstacle(0.22, 0.22, (0.91, 0.51))
# gmap.inflate_obstacle(1, 0.1)
# gmap.inflate_obstacle(2, 0.1)
gmap.draw()
bg = pygame.image.load('gridmap.png')

# Motion Planner
start = (0, 0)
goal = (gmap.shape[1] - 1, gmap.shape[0] - 1)
motion_planner = MotionPlanner(gmap, NODE_GEN_PRM, start, goal)

# Trajectory Planner
trplanner = TrajectoryPlanner(gmap, path_list=motion_planner.a_star_path)
x, y, x_dot, y_dot, theta, theta_dot = trplanner.cartesian_traj(f_s=10, profile=CUBIC_POL_PROF)
# optimal_path, path_length = trplanner.reed_sheep()
#(y*gmap.resolution, x*gmap.resolution, theta) # TODO: Traiettoria con orientamento

class BluetoothWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.bluetooth = BluetoothInterface()

        self.setWindowTitle('GCS Drifty')
        self.setGeometry(100, 100, 500, 1000)  # Imposta la dimensione della finestra

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
        self.control_combo.addItem('I/O Linearization')
        self.control_combo.addItem('Posture regulation')
        combo_font = QFont("Arial", 14)
        self.control_combo.setFont(combo_font)
        self.control_combo.activated[str].connect(self.on_changed_control)
        self.workspace.addWidget(self.control_combo)

        # ComboBox per la selezione dei controlli
        self.motion_planner_combo = QComboBox()
        self.motion_planner_combo.addItem('PRM')
        self.motion_planner_combo.addItem('RRT')
        combo_font = QFont("Arial", 14)
        self.motion_planner_combo.setFont(combo_font)
        self.motion_planner_combo.activated[str].connect(self.on_changed_motion_planner)
        self.workspace.addWidget(self.motion_planner_combo)

        self.control_buttons = QWidget()
        self.control_buttons.setMaximumSize(99999, 150)
        self.control_buttons_layout = QHBoxLayout()
        self.control_buttons.setLayout(self.control_buttons_layout)
        self.workspace.addWidget(self.control_buttons)

        self.process_button = QPushButton('PROCESS')
        self.process_button.setMinimumSize(100, 100)
        self.process_button.clicked.connect(self.handle_process_control_click)
        self.control_buttons_layout.addWidget(self.process_button)

        self.start_button = QPushButton('START')
        self.start_button.setMinimumSize(100, 100)
        self.start_button.clicked.connect(self.handle_start_control_click)
        self.control_buttons_layout.addWidget(self.start_button)

        self.stop_button = QPushButton('STOP')
        self.stop_button.setMinimumSize(100, 100)
        self.stop_button.clicked.connect(self.handle_stop_control_click)
        self.control_buttons_layout.addWidget(self.stop_button)

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
        self.obstacle_layout.addWidget(self.obs_x_label, 1, 0)

        self.obs_x_entry = QLineEdit()
        self.obs_x_entry.setPlaceholderText("0")
        font = self.obs_x_entry.font()
        font.setPointSize(10)
        self.obs_x_entry.setFont(font)
        self.obstacle_layout.addWidget(self.obs_x_entry, 1, 1)

        self.obs_y_label = QLabel("y:")
        font = self.obs_y_label.font()
        font.setPointSize(10)
        self.obs_y_label.setFont(font)
        self.obstacle_layout.addWidget(self.obs_y_label, 1, 2)

        self.obs_y_entry = QLineEdit()
        self.obs_y_entry.setPlaceholderText("0")
        font = self.obs_y_entry.font()
        font.setPointSize(10)
        self.obs_y_entry.setFont(font)
        self.obstacle_layout.addWidget(self.obs_y_entry, 1, 3)

        self.obs_w_label = QLabel("width:")
        font = self.obs_w_label.font()
        font.setPointSize(10)
        self.obs_w_label.setFont(font)
        self.obstacle_layout.addWidget(self.obs_w_label, 2, 0)

        self.obs_w_entry = QLineEdit()
        self.obs_w_entry.setPlaceholderText("0")
        font = self.obs_w_entry.font()
        font.setPointSize(10)
        self.obs_w_entry.setFont(font)
        self.obstacle_layout.addWidget(self.obs_w_entry, 2, 1)

        self.obs_h_label = QLabel("height:")
        font = self.obs_h_label.font()
        font.setPointSize(10)
        self.obs_h_label.setFont(font)
        self.obstacle_layout.addWidget(self.obs_h_label, 2, 2)

        self.obs_h_entry = QLineEdit()
        self.obs_h_entry.setPlaceholderText("0")
        font = self.obs_h_entry.font()
        font.setPointSize(10)
        self.obs_h_entry.setFont(font)
        self.obstacle_layout.addWidget(self.obs_h_entry, 2, 3)

        self.obs_label = QLabel("Obstacle")
        font = self.obs_label.font()
        font.setPointSize(10)
        self.obs_label.setFont(font)
        self.obstacle_layout.addWidget(self.obs_label, 0, 0)

        self.add_obstacle_button = QPushButton('+')
        self.add_obstacle_button.clicked.connect(self.handle_add_obstacle_click)
        self.obstacle_layout.addWidget(self.add_obstacle_button, 0, 1)

        self.motion_planning_label = QLabel("Motion planning params")
        font = self.motion_planning_label.font()
        font.setPointSize(10)
        self.motion_planning_label.setFont(font)
        self.obstacle_layout.addWidget(self.motion_planning_label, 3, 0)

        self.motion_planning_button = QPushButton('Apply')
        self.motion_planning_button.clicked.connect(self.handle_apply_params_motion_planning_click)
        self.obstacle_layout.addWidget(self.motion_planning_button, 3, 2)

        self.mp_PRM_label = QLabel("- PRM - ")
        self.mp_PRM_label.setAlignment(QtCore.Qt.AlignCenter)
        font = self.mp_PRM_label.font()
        font.setPointSize(10)
        self.mp_PRM_label.setFont(font)
        self.obstacle_layout.addWidget(self.mp_PRM_label, 4, 0)

        self.mp_num_samples_label = QLabel("Num. samples:")
        font = self.mp_num_samples_label.font()
        font.setPointSize(10)
        self.mp_num_samples_label.setFont(font)
        self.obstacle_layout.addWidget(self.mp_num_samples_label, 4, 1)

        self.mp_num_samples_entry = QLineEdit()
        self.mp_num_samples_entry.setPlaceholderText("100")
        font = self.mp_num_samples_entry.font()
        font.setPointSize(10)
        self.mp_num_samples_entry.setFont(font)
        self.obstacle_layout.addWidget(self.mp_num_samples_entry, 4, 2)

        self.mp_k_label = QLabel("k:")
        font = self.mp_k_label.font()
        font.setPointSize(10)
        self.mp_k_label.setFont(font)
        self.obstacle_layout.addWidget(self.mp_k_label, 4, 3)

        self.mp_k_entry = QLineEdit()
        self.mp_k_entry.setPlaceholderText("5")
        font = self.mp_k_entry.font()
        font.setPointSize(10)
        self.mp_k_entry.setFont(font)
        self.obstacle_layout.addWidget(self.mp_k_entry, 4, 4)

        self.mp_RRT_label = QLabel("- RRT -")
        self.mp_RRT_label.setAlignment(QtCore.Qt.AlignCenter)
        font = self.mp_RRT_label.font()
        font.setPointSize(10)
        self.mp_RRT_label.setFont(font)
        self.obstacle_layout.addWidget(self.mp_RRT_label, 5, 0)

        self.mp_inter_incr_label = QLabel("Iter. increment:")
        font = self.mp_inter_incr_label.font()
        font.setPointSize(10)
        self.mp_inter_incr_label.setFont(font)
        self.obstacle_layout.addWidget(self.mp_inter_incr_label, 5, 1)

        self.mp_inter_incr_entry = QLineEdit()
        self.mp_inter_incr_entry.setPlaceholderText("100")
        font = self.mp_inter_incr_entry.font()
        font.setPointSize(10)
        self.mp_inter_incr_entry.setFont(font)
        self.obstacle_layout.addWidget(self.mp_inter_incr_entry, 5, 2)

        self.mp_delta_label = QLabel("Delta:")
        font = self.mp_delta_label.font()
        font.setPointSize(10)
        self.mp_delta_label.setFont(font)
        self.obstacle_layout.addWidget(self.mp_delta_label, 5, 3)

        self.mp_delta_entry = QLineEdit()
        self.mp_delta_entry.setPlaceholderText("10")
        font = self.mp_delta_entry.font()
        font.setPointSize(10)
        self.mp_delta_entry.setFont(font)
        self.obstacle_layout.addWidget(self.mp_delta_entry, 5, 4)

        self.point = QWidget()
        self.point.setStyleSheet("#point { border: 2px solid black; }")
        self.point.setObjectName("point")
        self.point_layout = QGridLayout()
        self.point.setLayout(self.point_layout)
        self.workspace.addWidget(self.point)

        self.pnt_x_label = QLabel("x:")
        font = self.pnt_x_label.font()
        font.setPointSize(10)
        self.pnt_x_label.setFont(font)
        self.point_layout.addWidget(self.pnt_x_label, 0, 0)

        self.pnt_x_entry = QLineEdit()
        self.pnt_x_entry.setPlaceholderText("0")
        font = self.pnt_x_entry.font()
        font.setPointSize(10)
        self.pnt_x_entry.setFont(font)
        self.point_layout.addWidget(self.pnt_x_entry, 0, 1)

        self.pnt_y_label = QLabel("y:")
        font = self.pnt_y_label.font()
        font.setPointSize(10)
        self.pnt_y_label.setFont(font)
        self.point_layout.addWidget(self.pnt_y_label, 1, 0)

        self.pnt_y_entry = QLineEdit()
        self.pnt_y_entry.setPlaceholderText("0")
        font = self.pnt_y_entry.font()
        font.setPointSize(10)
        self.pnt_y_entry.setFont(font)
        self.point_layout.addWidget(self.pnt_y_entry, 1, 1)

        self.pnt_theta_label = QLabel("θ:")
        font = self.pnt_theta_label.font()
        font.setPointSize(10)
        self.pnt_theta_label.setFont(font)
        self.point_layout.addWidget(self.pnt_theta_label, 2, 0)

        self.pnt_theta_entry = QLineEdit()
        self.pnt_theta_entry.setPlaceholderText("0")
        font = self.pnt_theta_entry.font()
        font.setPointSize(10)
        self.pnt_theta_entry.setFont(font)
        self.point_layout.addWidget(self.pnt_theta_entry, 2, 1)

        self.set_goal_button = QPushButton('GOAL')
        self.set_goal_button.clicked.connect(self.handle_set_goal_click)
        self.point_layout.addWidget(self.set_goal_button, 3, 0)

        self.goto_button = QPushButton('GOTO')
        self.goto_button.clicked.connect(self.handle_goto_click)
        self.point_layout.addWidget(self.goto_button, 3, 1)

        # Timer per aggiornare i dati
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_data)
        self.timer.start(100)  # 200 ms per ottenere 5 aggiornamenti al secondo

    def on_changed_control(self, text):
        if text == 'Posture regulation':
            control = POSTURE_REGULATION
        elif 'Linearization' in text:
            control = IO_LINEARIZATION
        message = 'control,' + str(control) + "\n"
        self.bluetooth.sendBluetoothMessage(message)
        print(message)

    def on_changed_motion_planner(self, text):
        if text == 'PRM':
            motion_planner.node_generation = NODE_GEN_PRM
        elif text == 'RRT':
            motion_planner.node_generation = NODE_GEN_RRT
        else:
            raise ValueError("Node generation algorithm not valid")

    def handle_add_obstacle_click(self):
        x = float(self.obs_x_entry.text())
        y = float(self.obs_y_entry.text())
        height = float(self.obs_h_entry.text())
        width = float(self.obs_w_entry.text())
        _id = gmap.add_obstacle(height, width, (x, y))
        gmap.inflate_obstacle(_id, 0.1)
        gmap.draw()

    def handle_apply_params_motion_planning_click(self):
        if self.mp_num_samples_entry.text():
            motion_planner.num_samples = int(self.mp_num_samples_entry.text())
        if self.mp_k_entry.text():
            motion_planner.k = int(self.mp_k_entry.text())
        if self.mp_inter_incr_entry.text():
            motion_planner.iteration_increment = int(self.mp_inter_incr_entry.text())
        if self.mp_delta_entry.text():
            motion_planner.delta = int(self.mp_delta_entry.text())

    def handle_process_control_click(self):
        motion_planner.new_path()

    def handle_start_control_click(self):
        #message = 'trajectory,'
        #points = ','.join(["(" + str(int(x_pnt)) + "," + str(int(y_pnt)) + ")" for x_pnt, y_pnt in zip(x, y)])
        #print(message + points + '\n')
        #self.bluetooth.sendBluetoothMessage(message + points + '\n')
        self.bluetooth.sendBluetoothMessage('start\n')

    def handle_stop_control_click(self):
        self.bluetooth.sendBluetoothMessage('stop\n')

    def handle_set_goal_click(self):
        x = int(float(self.pnt_x_entry.text())/gmap.resolution)
        y = int(float(self.pnt_y_entry.text())/gmap.resolution)
        motion_planner.goal = (x, y)

    def handle_goto_click(self):
        x = self.pnt_x_entry.text()
        y = self.pnt_y_entry.text()
        theta = self.pnt_theta_entry.text()
        message = 'goto,'+str(x)+','+str(y)+','+str(theta)+'\n'
        self.bluetooth.sendBluetoothMessage(message)

    def update_data(self):
        # Recupera i dati ricevuti tramite Bluetooth
        try:
            x, y, theta, v, omega = self.bluetooth.last_message[0:5]  # Assicurati che il metodo esista e ritorni questi dati
            self.odo_x_label.setText("x: " + str(x))
            self.odo_y_label.setText("x: " + str(y))
            self.odo_theta_label.setText("θ: " + str(theta))
            self.cont_v_label.setText("v: " + str(v))
            self.cont_w_label.setText("ω: " + str(omega))
            # Memorizza i dati ricevuti in un file txt
            with open('data_log', 'a') as file:
                file.write(f"{x}, {y}, {theta}, {v}, {omega}")
        except Exception as e:
            pass


def main():
    os.environ['SDL_VIDEO_WINDOW_POS'] = "%d,%d" % (100, 100)

    # Setup app
    pygame.init()
    pygame.display.set_caption('Motion Planning Graph')
    width, height = gmap.shape
    screen = pygame.display.set_mode((height * CELL_SIZE, width * CELL_SIZE))
    screen_num_func = pygame.display.set_mode((height * CELL_SIZE, width * CELL_SIZE))
    font = pygame.font.Font(None, 36)

    # Create text elements
    a_star_text = font.render('A*', True, (255, 0, 0))  # Red color for A*
    a_star_text_rect = a_star_text.get_rect()
    a_star_text_rect.topleft = (screen.get_width() - 60, 5)
    bfs_text = font.render('BFS', True, (0, 0, 255))  # Blue color for BFS
    bfs_text_rect = bfs_text.get_rect()
    bfs_text_rect.topleft = (screen.get_width() - 60, 25)

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

        # Background
        screen.blit(pygame.transform.scale(pygame.image.load('gridmap.png'), (height * CELL_SIZE, width * CELL_SIZE)), (0, 0))
        pygame.draw.circle(screen, 'orange', (start[0] * CELL_SIZE + OFFSET, start[1] * CELL_SIZE + OFFSET), 5)
        pygame.draw.circle(screen, 'green', (motion_planner.goal[0] * CELL_SIZE + OFFSET, motion_planner.goal[1] * CELL_SIZE + OFFSET), 5)
        motion_planner.draw_graph(screen)
        motion_planner.draw_path(screen, 'bfs', 'blue')
        motion_planner.draw_path(screen, 'a_star', 'red')
        # Draw the text
        screen.blit(a_star_text, a_star_text_rect)
        screen.blit(bfs_text, bfs_text_rect)
        # Update screen
        pygame.display.flip()
        clock.tick(30)  # Limit to 30 FPS
        # Run the PyQt event loop
        app.processEvents()


if __name__ == "__main__":
    main()

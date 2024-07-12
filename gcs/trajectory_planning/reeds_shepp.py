import math
import copy
from enum import Enum
from typing import List
import pygame
from numpy import cos, sin, pi
if __name__ == "__main__":
    from ..motion_planning import gridmap
else:
    from motion_planning import gridmap

def R(x, y):
    """
    Polar transform (r, theta) for (x, y)
    """
    r = math.sqrt(x**2 + y**2)
    theta = math.atan2(y, x)
    return r, theta

def M(theta):
    """
    Map theta to [-pi, pi)
    """
    theta %= 2 * math.pi
    if theta < -math.pi: theta += 2 * math.pi
    elif theta >= math.pi: theta -= 2 * math.pi
    return theta

def normalize_basis(p1, p2):
    """
    Normalize the basis so that p1 equals (0, 0, 0)
    """
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    phi1 = p1[2]
    # Rotation matrix with -phi
    x_new = math.cos(phi1) * dx + math.sin(phi1) * dy
    y_new = -math.sin(phi1) * dx + math.cos(phi1) * dy
    phi_new = p2[2] - p1[2]
    return x_new, y_new, phi_new

def rad2deg(rad):
    return rad / math.pi * 180

class Steering(Enum):
    """
    Steering wheel, go left, straight, or right
    """
    LEFT = 1
    STRAIGHT = 0
    RIGHT = -1


class Gear(Enum):
    """
    Gear shift, go forward or backward
    """
    FORWARD = 1
    BACKWARD = -1


class Letter():
    """
    A letter is an element of a word, while a word is a possible path pattern
    """
    def __init__(self, param, steering, gear):
        self.param = param  # t, u, or v, length of the letter curve
        self.steering = steering
        self.gear = gear

    def __repr__(self):
        if self.steering == Steering.LEFT:
            steering_str = 'left'
        elif self.steering == Steering.RIGHT:
            steering_str = 'right'
        else:
            steering_str = 'straight'

        if self.gear == Gear.FORWARD:
            gear_str = 'forward'
        else:
            gear_str = 'backward'

        info_str = '{ Steering: ' + steering_str + '\tGear: ' + gear_str \
                   + '\tLength: ' + str(round(self.param, 2)) + ' }'

        return info_str

    def reverse_steering(self):
        self.steering = Steering(-self.steering.value)
        # if self.steering == Steering.LEFT:
        #     self.steering = Steering.RIGHT
        # elif self.steering == Steering.RIGHT:
        #     self.steering = Steering.LEFT

    def reverse_gear(self):
        self.gear = Gear(-self.gear.value)
        # self.gear = Gear.BACKWARD if self.gear == Gear.FORWARD else Gear.FORWARD


class ReedSheep: 
    def __init__(self, path_list):
        self.path_list = path_list

    def optimal_reed_sheep(self):
        path_length = 0
        optimal_path = []
        for i in range(len(self.path_list) - 1):
            path: List[dict] = self.get_optimal_word(self.path_list[i], self.path_list[i + 1])
            optimal_path.append(path)
            for subpath in path:
                path_length += float(subpath['Length'])
        return optimal_path, path_length
    
    @staticmethod
    def word_length(word):
        return sum([abs(letter.param) for letter in word])

    @staticmethod
    def timeflip(word):
        """
        Interchange + and - in a word, which results in (x, y, phi) to (-x, y, -phi)
        """
        word_new = copy.deepcopy(word)  # deepcopy method copies object **and its children**
        for letter in word_new:
            letter.reverse_gear()
        return word_new

    @staticmethod
    def reflect(word):
        """
        Interchange left and right in a word, which results in (x, y, phi) to (x, -y, -phi)
        """
        word_new = copy.deepcopy(word)
        for letter in word_new:
            letter.reverse_steering()
        return word_new

    @staticmethod
    def get_all_words(p1, p2):
        # 12 x 4 = 48 kinds of word patterns
        formulas = [ReedSheep.word_cluster_1, ReedSheep.word_cluster_2, ReedSheep.word_cluster_3, 
                    ReedSheep.word_cluster_4, ReedSheep.word_cluster_5, ReedSheep.word_cluster_6, 
                    ReedSheep.word_cluster_7, ReedSheep.word_cluster_8, ReedSheep.word_cluster_9, 
                    ReedSheep.word_cluster_10, ReedSheep.word_cluster_11, ReedSheep.word_cluster_12]
        x, y, phi = normalize_basis(p1, p2)
        words = []
        for f in formulas:
            words.append(f(x, y, phi))
            words.append(ReedSheep.timeflip(f(-x, y, -phi)))
            words.append(ReedSheep.reflect(f(x, -y, -phi)))
            words.append(ReedSheep.reflect(ReedSheep.timeflip(f(-x, -y, phi))))

        # remove letters that have parameter 0
        for i in range(len(words)):
            words[i] = list(filter(lambda e: e.param != 0, words[i]))

        # remove empty words
        words = list(filter(None, words))

        return words

    @staticmethod
    def get_dict_from_word(word):
        cleaned_string = word.strip('{}')
        pairs = cleaned_string.split()
        result_dict = {}
        i = 0
        while i < len(pairs):
            key = pairs[i].rstrip(':')
            value = pairs[i + 1]
            if value.replace('.', '', 1).isdigit():
                value = float(value) if '.' in value else int(value)
            result_dict[key] = value
            i += 2
        return result_dict

    def get_optimal_word(self, p1, p2) -> List[dict]:
        words = ReedSheep.get_all_words(p1, p2)
        i_min = -1
        L_min = float('inf')
        for i, word in enumerate(words):
            L = ReedSheep.word_length(word)
            if L <= L_min:
                i_min, L_min = i, L
        return [self.get_dict_from_word(str(word)) for word in words[i_min]]

    @staticmethod
    def word_cluster_1(x, y, phi):
        """
        Formula 8.1 for CSC words
        """
        word = []

        u, t = R(x - math.sin(phi), y - 1 + math.cos(phi))
        v = M(phi - t)

        params = [t, u, v]
        steerings = [Steering.LEFT, Steering.STRAIGHT, Steering.LEFT]
        gears = [Gear.FORWARD, Gear.FORWARD, Gear.FORWARD]

        for i in range(len(params)):
            word.append(Letter(params[i], steerings[i], gears[i]))

        return word

    @staticmethod
    def word_cluster_2(x, y, phi):
        """
        Formula 8.2 for CSC words
        """
        word = []

        u1, t1 = R(x + math.sin(phi), y - 1 - math.cos(phi))
        
        if u1**2 >= 4:
            u = math.sqrt(u1**2 - 4)
            _, theta = R(u, 2)
            t = M(t1 + theta)
            v = M(t - phi)

            params = [t, u, v]
            steerings = [Steering.LEFT, Steering.STRAIGHT, Steering.RIGHT]
            gears = [Gear.FORWARD, Gear.FORWARD, Gear.FORWARD]

            for i in range(len(params)):
                word.append(Letter(params[i], steerings[i], gears[i]))

        return word

    @staticmethod
    def word_cluster_3(x, y, phi):
        """
        Formule 8.3 for C|C|C words    *** TYPO IN PAPER ***
        """
        word = []

        xi = x - math.sin(phi)
        eta = y - 1 + math.cos(phi)
        rho, theta = R(xi, eta)

        if rho <= 4:
            A = math.acos(rho / 4)
            t = M(theta + math.pi/2 + A)
            u = M(math.pi - 2*A)
            v = M(phi - t - u)

            params = [t, u, v]
            steerings = [Steering.LEFT, Steering.RIGHT, Steering.LEFT]
            gears = [Gear.FORWARD, Gear.BACKWARD, Gear.FORWARD]

            for i in range(len(params)):
                word.append(Letter(params[i], steerings[i], gears[i]))

        return word

    @staticmethod
    def word_cluster_4(x, y, phi):
        """
        Formule 8.4 for C|CC words    *** TYPO IN PAPER ***
        """
        word = []

        xi = x - math.sin(phi)
        eta = y - 1 + math.cos(phi)
        rho, theta = R(xi, eta)

        if rho <= 4:
            A = math.acos(rho / 4)
            t = M(theta + math.pi/2 + A)
            u = M(math.pi - 2*A)
            v = M(t + u - phi)

            params = [t, u, v]
            steerings = [Steering.LEFT, Steering.RIGHT, Steering.LEFT]
            gears = [Gear.FORWARD, Gear.BACKWARD, Gear.BACKWARD]

            for i in range(len(params)):
                word.append(Letter(params[i], steerings[i], gears[i]))

        return word

    @staticmethod
    def word_cluster_5(x, y, phi):
        """
        Formule 8.4 for CC|C words    *** TYPO IN PAPER ***
        """
        word = []

        xi = x - math.sin(phi)
        eta = y - 1 + math.cos(phi)
        rho, theta = R(xi, eta)

        if rho <= 4:
            u = math.acos(1 - rho**2 / 8)
            A = math.asin(2 * math.sin(u) / rho)
            t = M(theta + math.pi/2 - A)
            v = M(t - u - phi)

            params = [t, u, v]
            steerings = [Steering.LEFT, Steering.RIGHT, Steering.LEFT]
            gears = [Gear.FORWARD, Gear.FORWARD, Gear.BACKWARD]

            for i in range(len(params)):
                word.append(Letter(params[i], steerings[i], gears[i]))

        return word

    @staticmethod
    def word_cluster_6(x, y, phi):
        """
        Formule 8.7 for CCu|CuC words
        """
        word = []

        xi = x + math.sin(phi)
        eta = y - 1 - math.cos(phi)
        rho, theta = R(xi, eta)

        if rho <= 4:
            if rho <= 2:
                A = math.acos((rho + 2) / 4)
                t = M(theta + math.pi/2 + A)
                u = M(A)
                v = M(phi - t + 2*u)
            else:
                A = math.acos((rho - 2) / 4)
                t = M(theta + math.pi/2 - A)
                u = M(math.pi - A)
                v = M(phi - t + 2*u)

            params = [t, u, u, v]
            steerings = [Steering.LEFT, Steering.RIGHT, Steering.LEFT, Steering.RIGHT]
            gears = [Gear.FORWARD, Gear.FORWARD, Gear.BACKWARD, Gear.BACKWARD]

            for i in range(len(params)):
                word.append(Letter(params[i], steerings[i], gears[i]))

        return word

    @staticmethod
    def word_cluster_7(x, y, phi):
        """
        Formule 8.8 for C|CuCu|C words
        """
        word = []

        xi = x + math.sin(phi)
        eta = y - 1 - math.cos(phi)
        rho, theta = R(xi, eta)
        u1 = (20 - rho*rho) / 16

        if rho <= 6 and 0 <= u1 and u1 <= 1:
            u = math.acos(u1)
            A = math.asin(2 * math.sin(u) / rho)
            t = M(theta + math.pi/2 + A)
            v = M(t - phi)

            params = [t, u, u, v]
            steerings = [Steering.LEFT, Steering.RIGHT, Steering.LEFT, Steering.RIGHT]
            gears = [Gear.FORWARD, Gear.BACKWARD, Gear.BACKWARD, Gear.FORWARD]

            for i in range(len(params)):
                word.append(Letter(params[i], steerings[i], gears[i]))

        return word

    @staticmethod
    def word_cluster_8(x, y, phi):
        """
        Formule 8.9 for C|C[pi/2]SC words
        """
        word = []

        xi = x - math.sin(phi)
        eta = y - 1 + math.cos(phi)
        rho, theta = R(xi, eta)

        if rho >= 2:
            u = math.sqrt(rho*rho - 4) - 2
            A = math.atan2(2, u+2)
            t = M(theta + math.pi/2 + A)
            v = M(t - phi + math.pi/2)

            params = [t, math.pi/2, u, v]
            steerings = [Steering.LEFT, Steering.RIGHT, Steering.STRAIGHT, Steering.LEFT]
            gears = [Gear.FORWARD, Gear.BACKWARD, Gear.BACKWARD, Gear.BACKWARD]

            for i in range(len(params)):
                word.append(Letter(params[i], steerings[i], gears[i]))

        return word

    @staticmethod
    def word_cluster_9(x, y, phi):
        """
        Formule 8.9 for CSC[pi/2]|C words
        """
        word = []

        xi = x - math.sin(phi)
        eta = y - 1 + math.cos(phi)
        rho, theta = R(xi, eta)

        if rho >= 2:
            u = math.sqrt(rho*rho - 4) - 2
            A = math.atan2(u+2, 2)
            t = M(theta + math.pi/2 - A)
            v = M(t - phi - math.pi/2)

            params = [t, u, math.pi/2, v]
            steerings = [Steering.LEFT, Steering.STRAIGHT, Steering.RIGHT, Steering.LEFT]
            gears = [Gear.FORWARD, Gear.FORWARD, Gear.FORWARD, Gear.BACKWARD]

            for i in range(len(params)):
                word.append(Letter(params[i], steerings[i], gears[i]))

        return word

    @staticmethod
    def word_cluster_10(x, y, phi):
        """
        Formule 8.10 for C|C[pi/2]SC words
        """
        word = []

        xi = x + math.sin(phi)
        eta = y - 1 - math.cos(phi)
        rho, theta = R(xi, eta)

        if rho >= 2:
            t = M(theta + math.pi/2)
            u = rho - 2
            v = M(phi - t - math.pi/2)

            params = [t, math.pi/2, u, v]
            steerings = [Steering.LEFT, Steering.RIGHT, Steering.STRAIGHT, Steering.RIGHT]
            gears = [Gear.FORWARD, Gear.BACKWARD, Gear.BACKWARD, Gear.BACKWARD]

            for i in range(len(params)):
                word.append(Letter(params[i], steerings[i], gears[i]))

        return word

    @staticmethod
    def word_cluster_11(x, y, phi):
        """
        Formule 8.10 for CSC[pi/2]|C words
        """
        word = []

        xi = x + math.sin(phi)
        eta = y - 1 - math.cos(phi)
        rho, theta = R(xi, eta)

        if rho >= 2:
            t = M(theta)
            u = rho - 2
            v = M(phi - t - math.pi/2)

            params = [t, u, math.pi/2, v]
            steerings = [Steering.LEFT, Steering.STRAIGHT, Steering.LEFT, Steering.RIGHT]
            gears = [Gear.FORWARD, Gear.FORWARD, Gear.FORWARD, Gear.BACKWARD]

            for i in range(len(params)):
                word.append(Letter(params[i], steerings[i], gears[i]))

        return word

    @staticmethod
    def word_cluster_12(x, y, phi):
        """
        Formule 8.11 for C|C[pi/2]SC[pi/2]|C words
        """
        word = []

        xi = x + math.sin(phi)
        eta = y - 1 - math.cos(phi)
        rho, theta = R(xi, eta)

        if rho >= 4:
            u = math.sqrt(rho*rho - 4) - 4
            A = math.atan2(2, u+4)
            t = M(theta + math.pi/2 + A)
            v = M(t - phi)
            params = [t, math.pi/2, u, math.pi/2, v]
            steerings = [Steering.LEFT, Steering.RIGHT, Steering.STRAIGHT, Steering.LEFT, Steering.RIGHT]
            gears = [Gear.FORWARD, Gear.BACKWARD, Gear.BACKWARD, Gear.BACKWARD, Gear.FORWARD]
            for i in range(len(params)):
                word.append(Letter(params[i], steerings[i], gears[i]))

        return word

    @staticmethod
    def draw_arc(screen, start_pos, orientation, angle, radius, direction):
        start_x, start_y = start_pos
        if direction == 'left':
            irc = (start_y - sin(orientation)*radius, start_x + cos(orientation)*radius)
            rect = pygame.Rect(irc[1] - radius, irc[0] - radius, 2*radius, 2*radius)  # y, x
            pygame.draw.arc(screen, (0, 255, 0), rect, pi+orientation, pi+orientation+angle, 2)
            spostamento = 2*sin(angle/2)*radius
            angolo_spostamento = orientation+(angle/2)
        elif direction == 'right':
            irc = (start_y + sin(orientation) * radius, start_x - cos(orientation) * radius)
            rect = pygame.Rect(irc[1] - radius, irc[0] - radius, 2 * radius, 2 * radius)  # y, x
            pygame.draw.arc(screen, (0, 255, 0), rect, 2*pi - (angle-orientation), orientation, 2)
            spostamento = 2 * sin(angle / 2) * radius
            angolo_spostamento = orientation - (angle / 2)
        x_end = start_x + spostamento * sin(angolo_spostamento)
        y_end = start_y + spostamento * cos(angolo_spostamento)
        pygame.draw.circle(screen, 'orange', (x_end, y_end), 10)
        return [x_end, y_end]

    def draw_path(self, screen, gmap, start_pos, orientation, path):
        start_pos = start_pos
        orientation = float(orientation)
        #for segment in path:
        segment = [{'Steering': 'left', 'Gear': 'forward', 'Length': 0.0},
         {'Steering': 'straight', 'Gear': 'forward', 'Length': 11.95},
         {'Steering': 'right', 'Gear': 'forward', 'Length': 0.05}]
        for action in segment:
            if action['Steering'] == 'straight':
                length_px = action['Length'] * 0.01 / gmap.resolution * gridmap.CELL_SIZE
                end_pos = (start_pos[0] + length_px*sin(orientation), start_pos[1] + length_px*cos(orientation))
                pygame.draw.line(screen, (0, 255, 0), start_pos, end_pos, 2)
                start_pos = end_pos
                pygame.draw.circle(screen, 'orange', end_pos, 10)
            elif action['Steering'] == 'left' or action['Steering'] == 'right':
                angle = float(action['Length'])
                radius = 1 / gmap.resolution * gridmap.CELL_SIZE  # Raggio dell'arco di circonferenza (ad esempio, 5 quadretti)
                start_pos = self.draw_arc(screen, start_pos, orientation, angle, radius, action['Steering'])
                if action['Steering'] == 'left':
                    orientation += angle
                elif action['Steering'] == 'right':
                    orientation -= angle

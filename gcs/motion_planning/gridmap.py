import numpy as np
from math import ceil
from PIL import Image, ImageDraw


class GridMap(np.ndarray):

    def __new__(cls, height, length, res):
        obj = super().__new__(cls, (ceil(height / res), ceil(length / res)), dtype=np.uint8)
        return obj

    def __init__(self, height, length, res):
        self.resolution = res
        self.obstacles = []
        self.fill(0)

    def add_obstacle(self, height, length, pos):
        x, y = pos
        x_end = x + length
        y_end = y + height
        if np.any(self[y:y_end, x:x_end] == 1):
            print(f'Obstacle cannot be added because it overlaps with another one!')
            return False
        if 0 <= x < self.shape[1] and 0 <= y < self.shape[0] and x_end <= self.shape[1] and y_end <= self.shape[0]:
            self[y:y_end, x:x_end] = 1
            self.obstacles.append({'id': len(self.obstacles) + 1,
                                   'height': height,
                                   'length': length,
                                   'pos': (x, y),
                                   'end': (x_end, y_end)})
            return True
        else:
            print(f'Obstacle cannot be added to the gridmap!')
            return False

    def inflate_obstacle(self, obs_id, inflation_size):
        obstacle = next((obs for obs in self.obstacles if obs['id'] == obs_id), None)
        if not obstacle:
            print(f'Obstacle with id {obs_id} not found!')
            return False

        x_start = max(0, obstacle['pos'][0] - inflation_size)
        x_end = min(self.shape[1], obstacle['end'][0] + inflation_size)
        y_start = max(0, obstacle['pos'][1] - inflation_size)
        y_end = min(self.shape[0], obstacle['end'][1] + inflation_size)

        for y in range(y_start, y_end):
            for x in range(x_start, x_end):
                if self[y, x] == 0:
                    self[y, x] = 2
        return True

    def __str__(self):
        self.draw_grid()
        return super().__str__()

    def draw_grid(self):
        cell_size = 7
        img_size = (self.shape[1] * cell_size, self.shape[0] * cell_size)

        img = Image.new("RGB", img_size, "white")
        draw = ImageDraw.Draw(img)

        for y in range(self.shape[0]):
            for x in range(self.shape[1]):
                top_left = (x * cell_size, y * cell_size)
                bottom_right = ((x + 1) * cell_size, (y + 1) * cell_size)
                if self[y, x] == 1:
                    draw.rectangle([top_left, bottom_right], fill="black")
                elif self[y, x] == 2:
                    draw.rectangle([top_left, bottom_right], fill="red")
                else:
                    draw.rectangle([top_left, bottom_right], fill="white", outline="black")

        img.show()


if __name__ == "__main__":
    gmap = GridMap(1, 2, 0.01)
    gmap.add_obstacle(10, 10, (30, 70))
    gmap.add_obstacle(5, 20, (20, 20))

    gmap.inflate_obstacle(1, 5)
    gmap.inflate_obstacle(2, 15)

    gmap.draw_grid()

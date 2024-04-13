import cv2
import yaml
import numpy as np

import poly


def draw_crosshair(img):
    img_with_crosshair = img.copy()

    # Draw crosshair in the middle of the image
    center_x = img_with_crosshair.shape[1] // 2
    center_y = img_with_crosshair.shape[0] // 2
    crosshair_size = 10
    crosshair_color = (0, 255, 0)  # Green color
    cv2.line(
        img_with_crosshair,
        (center_x - crosshair_size, center_y),
        (center_x + crosshair_size, center_y),
        crosshair_color,
        1,
    )
    cv2.line(
        img_with_crosshair,
        (center_x, center_y - crosshair_size),
        (center_x, center_y + crosshair_size),
        crosshair_color,
        1,
    )

    return img_with_crosshair


class Map:
    def __init__(self, yaml_file):
        self.load_map(yaml_file)
        self.original_map_img = self.map_img.copy()  # Keep a copy for reference
        self.poly = poly.Poly(
            _pts_pix=np.empty((0, 2), dtype=int), map_shape=self.map_img.shape
        )

    def load_map(self, yaml_file):
        # Load map parameters from YAML file
        with open(yaml_file, "r") as file:
            data = yaml.safe_load(file)
            self.img_path = data["image"]
            self.map_img = cv2.imread(self.img_path)
            self.resolution = data["resolution"]

    def translate(self, dx, dy):
        # Translate the map image
        self.map_img = np.roll(self.map_img, dx, axis=1)
        self.map_img = np.roll(self.map_img, dy, axis=0)

    def rotate(self, angle):
        # Rotate the map image
        h, w = self.map_img.shape[:2]
        rotation_center = (w / 2, h / 2)
        rotation_matrix = cv2.getRotationMatrix2D(rotation_center, angle, 1)
        self.map_img = cv2.warpAffine(
            self.map_img, rotation_matrix, (w, h), cv2.INTER_LANCZOS4
        )

    def add_poly_pt_map(self, pt_map):
        self.poly.add_pt_map(pt_map)

    def reset(self):
        # Reset the map image to its original state
        self.map_img = self.original_map_img
        self.poly = poly.Poly(
            _pts_pix=np.empty((0, 2), dtype=int), map_shape=self.map_img.shape
        )

from dataclasses import dataclass

import cv2
import numpy as np


def draw_poly(image, pts, color=(255, 255, 255), thickness=1):
    poly_image = image.copy()
    pts = pts.reshape((-1, 1, 2))
    cv2.polylines(poly_image, [pts], isClosed=True, color=color, thickness=thickness)
    return poly_image


def calculate_iou(poly1_pts, poly2_pts, img_h, img_w):
    mask1 = np.zeros((img_h, img_w), dtype=np.uint8)
    mask2 = np.zeros((img_h, img_w), dtype=np.uint8)

    cv2.fillPoly(mask1, [poly1_pts], color=1)
    cv2.fillPoly(mask2, [poly2_pts], color=1)

    intersection = np.logical_and(mask1, mask2)
    union = np.logical_or(mask1, mask2)

    iou = np.sum(intersection) / np.sum(union)
    return iou


@dataclass
class Poly:
    _pts_m: np.ndarray = None
    _pts_pix: np.ndarray = None
    _pts_map: np.ndarray = None
    resolution: float = None
    map_shape: tuple = None

    def add_pt_map(self, pt_map):
        if self._pts_map is None:
            self._pts_map = np.array([pt_map])
        else:
            self._pts_map = np.vstack([self._pts_map, pt_map])

    def pts_m_to_pix(self):
        return (self._pts_m / self.resolution).astype(int)

    def pts_pix_to_m(self):
        return self._pts_pix * self.resolution

    def pts_pix_to_map(self, h, w):
        return self._pts_pix + np.array([w // 2, h // 2])

    @property
    def pts_m(self):
        if self._pts_m is not None:
            return self._pts_m
        self._pts_m = self.pts_pix_to_m()
        return self._pts_m

    @property
    def pts_pix(self):
        if self._pts_pix is not None:
            return self._pts_pix
        self._pts_pix = self.pts_m_to_pix()
        return self._pts_pix

    @property
    def pts_map(self):
        if self._pts_map is not None:
            return self._pts_map
        elif self._pts_pix is not None:
            self._pts_map = self.pts_pix_to_map(*self.map_shape[:2])
        else:
            self._pts_pix = self.pts_m_to_pix()
            self._pts_map = self.pts_pix_to_map(*self.map_shape[:2])

        return self._pts_map

    def tr_pts_map(self, dx, dy):
        self._pts_map += np.array([dx, dy])

    def rot_pts_map(self, angle):
        rot_mat = cv2.getRotationMatrix2D(
            (self.map_shape[1] // 2, self.map_shape[0] // 2), angle, 1
        )
        self._pts_map = (
            cv2.transform(np.array([self._pts_map]), rot_mat).squeeze().astype(int)
        )

    def pts_map_to_pix(self):
        self._pts_pix = self._pts_map - np.array(self.map_shape[:2]) // 2
        return self._pts_pix

    def pts_pix_to_m(self):
        self._pts_m = self._pts_pix * self.resolution
        return self._pts_m

    def pts_map_to_m(self):
        self.pts_map_to_pix()
        return self.pts_pix_to_m()


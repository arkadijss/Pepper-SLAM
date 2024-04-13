import cv2
import numpy as np

import map
import poly


def main():
    # Load map
    map_obj = map.Map("base.yaml")
    # Translation step
    t_step = 5
    # Rotation step
    r_step = 5

    # Ground truth polygon
    poly_gt_m = np.array(
        [[-4.5, 2.5], [-4.5, -2.5], [4.5, -2.5], [4.5, 2.5], [-4.5, 2.5]]
    )
    poly_gt = poly.Poly(
        _pts_m=poly_gt_m,
        resolution=map_obj.resolution,
        map_shape=map_obj.map_img.shape,
    )

    window_name = "Map"
    cv2.namedWindow(window_name)

    def mouse_callback(event, x, y, flags, param):
        nonlocal map_obj
        if event == cv2.EVENT_LBUTTONDOWN:
            map_obj.add_poly_pt_map(np.array([x, y]))

    cv2.setMouseCallback(window_name, mouse_callback)

    while True:
        map_img = map.draw_crosshair(map_obj.map_img)
        map_img = poly.draw_poly(
            map_img, poly_gt.pts_map, color=(0, 255, 0)
        )  # Green for GT polygon
        if len(map_obj.poly.pts_map) >= 4:
            map_img = poly.draw_poly(
                map_img,
                map_obj.poly.pts_map,
                color=(255, 0, 0),  # Blue for estimated polygon
            )
            iou = poly.calculate_iou(
                poly_gt.pts_map,
                np.vstack([map_obj.poly.pts_map, map_obj.poly.pts_map[0]]),
                *map_obj.map_img.shape[:2],
            )
            print(f"IoU: {iou:.2f}")

        cv2.imshow(window_name, map_img)

        key = cv2.waitKey(1) & 0xFF

        if key == ord("a"):
            map_obj.translate(-t_step, 0)
        elif key == ord("d"):
            map_obj.translate(t_step, 0)
        elif key == ord("w"):
            map_obj.translate(0, -t_step)
        elif key == ord("s"):
            map_obj.translate(0, t_step)

        elif key == ord("z"):
            map_obj.rotate(r_step)
        elif key == ord("c"):
            map_obj.rotate(-r_step)

        if key == ord("r"):
            map_obj.reset()
        elif key == ord("q"):
            break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

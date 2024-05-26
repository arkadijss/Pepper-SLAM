import cv2
import numpy as np
import pandas as pd

import map
import poly
import traj


def draw_points(image, pts, color=(255, 255, 255), thickness=1):
    point_image = image.copy()
    for pt in pts:
        cv2.circle(point_image, tuple(pt), thickness, color, -1)
    return point_image


def main():
    # Load map
    map_obj = map.Map(
        "map.yaml",
    )
    # Translation step
    t_step = 5
    # Rotation step
    r_step = 5

    # Ground truth polygon
    poly_gt_m = np.array(
        [[-4.52, 2.98], [-4.52, -2.98], [4.52, -2.98], [4.52, 2.98], [-4.52, 2.98]]
    )
    poly_gt = poly.Poly(
        _pts_m=poly_gt_m,
        resolution=map_obj.resolution,
        map_shape=map_obj.map_img.shape,
    )

    # Ground truth trajectory
    traj_gt_ts = np.array(
        [44, 98, 158, 220, 265, 313, 362, 415, 460, 509, 556, 613, 780, 960]
    )
    traj_gt_pos = np.array(
        [
            [0, -1.7],
            [2.4, 0],
            [0, 1.7],
            [-2.4, 0],
            [0, -1.7],
            [2.4, 0],
            [0, 1.7],
            [-2.4, 0],
            [0, -1.7],
            [2.4, 0],
            [0, 1.7],
            [-2.4, 0],
            [2.4, 0],
            [-2.4, 0],
        ]
    )
    # shift x coordinates
    x_shift = -0.46
    traj_gt_pos[:, 0] += x_shift

    traj_gt_poly = poly.Poly(
        _pts_m=traj_gt_pos,
        resolution=map_obj.resolution,
        map_shape=map_obj.map_img.shape,
    )

    # Estimated trajectory
    # Reference start timestamp
    ts_start = 1711648428
    traj_obj = traj.Trajectory("traj.csv", ts_start)
    # Extract point positions at timestamps
    traj_obj_pos = traj_obj.get_pos_at_ts(traj_gt_ts)
    # Invert y coordinates
    traj_obj_pos[:, 1] *= -1
    # Convert to 2D
    traj_obj_pos_2d = traj_obj_pos[:, :2]
    # Create a Poly object
    traj_poly = poly.Poly(
        _pts_m=traj_obj_pos_2d,
        resolution=map_obj.resolution,
        map_shape=map_obj.map_img.shape,
    )
    # Initialize trajectory map points
    traj_poly.pts_map

    # Result file
    res_path = "results.csv"
    res_dict = {
        "ts": traj_gt_ts,
        "est_pos_x": None,
        "est_pos_y": None,
        "gt_pos_x": traj_gt_pos[:, 0],
        "gt_pos_y": traj_gt_pos[:, 1],
        "loc_err": None,
        "iou": None,
    }

    window_name = "Map"
    cv2.namedWindow(window_name)

    def mouse_callback(event, x, y, flags, param):
        nonlocal map_obj
        if event == cv2.EVENT_LBUTTONDOWN:
            map_obj.add_poly_pt_map(np.array([x, y]))

    cv2.setMouseCallback(window_name, mouse_callback)

    while True:
        map_img = map.draw_crosshair(map_obj.map_img)
        # Green for GT
        map_img = poly.draw_poly(map_img, poly_gt.pts_map, color=(0, 255, 0))  # polygon
        map_img = draw_points(
            map_img, traj_gt_poly.pts_map, color=(0, 255, 0), thickness=2
        )  # trajectory

        if len(map_obj.poly.pts_map) >= 4:
            # IoU
            iou = poly.calculate_iou(
                poly_gt.pts_map,
                np.vstack([map_obj.poly.pts_map, map_obj.poly.pts_map[0]]),
                *map_obj.map_img.shape[:2],
            )
            print(f"IoU: {iou:.2f}")
            res_dict["iou"] = np.round(iou, 2)

            map_img = poly.draw_poly(
                map_img,
                map_obj.poly.pts_map,
                color=(255, 0, 0),  # Blue for estimated polygon
            )

            # Localization error
            traj_est_pos = np.round(traj_poly.pts_map_to_m(), 2)
            loc_err = np.round(traj.loc_err(traj_gt_pos, traj_est_pos), 2)
            print(f"Predicted: {traj_est_pos}")
            print(f"Ground truth: {traj_gt_pos}")
            print(f"Localization error: {loc_err}")

            res_dict["est_pos_x"] = traj_est_pos[:, 0]
            res_dict["est_pos_y"] = traj_est_pos[:, 1]
            res_dict["loc_err"] = loc_err

            map_img = draw_points(map_img, traj_poly.pts_map, color=(0, 0, 255))

            res_df = pd.DataFrame(res_dict)
            res_df.to_csv(res_path, index=False)

        cv2.imshow(window_name, map_img)

        key = cv2.waitKey(1) & 0xFF

        if key == ord("a"):
            map_obj.translate(-t_step, 0)
            traj_poly.tr_pts_map(-t_step, 0)
        elif key == ord("d"):
            map_obj.translate(t_step, 0)
            traj_poly.tr_pts_map(t_step, 0)
        elif key == ord("w"):
            map_obj.translate(0, -t_step)
            traj_poly.tr_pts_map(0, -t_step)
        elif key == ord("s"):
            map_obj.translate(0, t_step)
            traj_poly.tr_pts_map(0, t_step)

        elif key == ord("z"):
            map_obj.rotate(r_step)
            traj_poly.rot_pts_map(r_step)
        elif key == ord("c"):
            map_obj.rotate(-r_step)
            traj_poly.rot_pts_map(-r_step)

        if key == ord("r"):
            map_obj.reset()
        elif key == ord("q"):
            break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

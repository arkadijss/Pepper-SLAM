import numpy as np
import pandas as pd


def quaternion_to_z_rotation(quaternion):
    # Extract x, y, z, w components
    x, y, z, w = quaternion

    # Compute Euler angle
    z_rotation = np.arctan2(2 * (x * z + y * w), 1 - 2 * (z**2 + w**2))

    return z_rotation


def loc_err(traj_est_pos, traj_gt_pos):
    return np.linalg.norm(traj_est_pos - traj_gt_pos, axis=1)


class Trajectory:
    def __init__(self, csv_file, ts_start):
        self.read_data(csv_file, ts_start)

    def read_data(self, csv_file, ts_start):
        data = pd.read_csv(csv_file)
        self.ts = (data["Timestamp"] - ts_start).values  # Normalize timestamps
        self.pos = data[["X", "Y", "Z"]].values
        self.orientation = data[
            ["Orientation_X", "Orientation_Y", "Orientation_Z", "Orientation_W"]
        ].values

    def get_initial_orientation(self):
        return self.orientation[0]

    def get_initial_z_rotation(self):
        return quaternion_to_z_rotation(self.get_initial_orientation())

    def get_pos_at_ts(self, ts):
        indices = np.argmin(np.abs(self.ts[:, np.newaxis] - ts), axis=0)
        return self.pos[indices]

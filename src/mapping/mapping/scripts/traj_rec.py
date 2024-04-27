#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
import pandas as pd


class TrajectoryRecorder:
    def __init__(self, output_file):
        self.output_file = output_file
        self.data = (
            []
        )  # List to store (timestamp, x, y, z, orientation_x, orientation_y, orientation_z, orientation_w) tuples
        self.initial_orientation = None
        self.subscriber = rospy.Subscriber(
            "/trajectory", Path, self.trajectory_callback
        )
        rospy.loginfo("Trajectory recorder initialized")

    def trajectory_callback(self, msg):
        rospy.loginfo("Received trajectory message")
        for pose_stamped in msg.poses:
            timestamp = pose_stamped.header.stamp.to_sec()
            position = pose_stamped.pose.position
            orientation = pose_stamped.pose.orientation
            if not self.initial_orientation:
                self.initial_orientation = orientation
            self.data.append(
                (
                    timestamp,
                    position.x,
                    position.y,
                    position.z,
                    orientation.x,
                    orientation.y,
                    orientation.z,
                    orientation.w,
                )
            )

    def save_to_csv(self):
        rospy.loginfo("Saving trajectory to CSV")
        columns = [
            "Timestamp",
            "X",
            "Y",
            "Z",
            "Orientation_X",
            "Orientation_Y",
            "Orientation_Z",
            "Orientation_W",
        ]
        df = pd.DataFrame(self.data, columns=columns)
        df.to_csv(self.output_file, index=False)


def main():
    rospy.init_node("trajectory_recorder")
    rospy.loginfo("Node initialized")

    output_file = "/workspace/traj.csv"
    recorder = TrajectoryRecorder(output_file)

    rospy.on_shutdown(recorder.save_to_csv)

    rospy.spin()
    rospy.loginfo("Exiting main loop")


if __name__ == "__main__":
    main()

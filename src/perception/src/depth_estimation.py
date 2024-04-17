#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import torch
import numpy as np
from PIL import Image as PILImage
import scripts.depth_estimation.metric_depth.estimate as estimate


class DepthEstimation:
    def __init__(
        self,
        img_topic="~/camera/image_raw",
        cam_info_topic="~/camera/camera_info",
        depth_topic="~/depth_nn/image",
        depth_nn_cam_info_topic="~/depth_nn/camera_info",
    ):
        rospy.init_node("depth_estimation_node", anonymous=True)
        self.bridge = CvBridge()
        self.model = estimate.build_model_metric()

        self.cam_info = None
        self.cam_info_sub = rospy.Subscriber(
            cam_info_topic, CameraInfo, self.cam_info_callback
        )
        self.img_sub = rospy.Subscriber(img_topic, Image, self.img_callback)

        self.depth_pub = rospy.Publisher(depth_topic, Image, queue_size=10)
        self.depth_nn_cam_info_pub = rospy.Publisher(
            depth_nn_cam_info_topic, CameraInfo, queue_size=10
        )

    def cam_info_callback(self, cam_info):
        self.cam_info = cam_info

    def img_callback(self, ros_img):
        try:
            cam_info = self.cam_info
            cv_img = self.bridge.imgmsg_to_cv2(ros_img, "bgr8")
            img_torch = estimate.prepare_sample(cv_img).cuda()

            pred_depth = estimate.infer(self.model, img_torch)

            # interpolate to original size
            pred = torch.nn.functional.interpolate(
                pred_depth,
                size=cv_img.shape[:-1],
                mode="bicubic",
                align_corners=False,
            )

            output = pred.squeeze().cpu().numpy()

            output = (output * 1000).astype(np.uint16)

            # Convert depth image to ROS message
            depth_msg = self.bridge.cv2_to_imgmsg(output, "16UC1")
            depth_msg.header = ros_img.header

            # Publish the depth image
            self.depth_pub.publish(depth_msg)

            # Publish the camera info
            self.depth_nn_cam_info_pub.publish(cam_info)

        except Exception as e:
            rospy.logerr(e)


if __name__ == "__main__":
    try:
        img_topic = "/naoqi_driver/camera/front/image_raw"
        cam_info_topic = "/naoqi_driver/camera/front/camera_info"
        depth_node = DepthEstimation(
            img_topic=img_topic,
            cam_info_topic=cam_info_topic,
        )
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Depth estimation node terminated.")

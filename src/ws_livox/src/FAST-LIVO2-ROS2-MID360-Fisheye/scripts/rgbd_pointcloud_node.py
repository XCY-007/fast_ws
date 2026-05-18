#!/usr/bin/env python3
import math
from typing import Optional

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2


class RgbdPointCloudNode(Node):
    def __init__(self):
        super().__init__("rgbd_pointcloud")
        self.declare_parameter("color_topic", "/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("cloud_topic", "/camera/depth/color/points")
        self.declare_parameter("output_frame", "camera_init")
        self.declare_parameter("fx", 461.0)
        self.declare_parameter("fy", 461.0)
        self.declare_parameter("cx", 320.0)
        self.declare_parameter("cy", 240.0)
        self.declare_parameter("stride", 3)
        self.declare_parameter("min_depth", 0.25)
        self.declare_parameter("max_depth", 8.0)

        self.bridge = CvBridge()
        self.color_msg: Optional[Image] = None
        self.color_img = None

        color_topic = self.get_parameter("color_topic").value
        depth_topic = self.get_parameter("depth_topic").value
        cloud_topic = self.get_parameter("cloud_topic").value

        self.create_subscription(Image, color_topic, self.color_callback, qos_profile_sensor_data)
        self.create_subscription(Image, depth_topic, self.depth_callback, qos_profile_sensor_data)
        self.pub = self.create_publisher(PointCloud2, cloud_topic, qos_profile_sensor_data)

    def color_callback(self, msg: Image):
        self.color_msg = msg
        self.color_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def depth_callback(self, msg: Image):
        if self.color_img is None:
            return

        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        if depth is None:
            return

        depth = np.asarray(depth)
        if msg.encoding in ("16UC1", "mono16"):
            depth_m = depth.astype(np.float32) * 0.001
        else:
            depth_m = depth.astype(np.float32)

        color = np.asarray(self.color_img)
        if color.ndim == 2:
            color = np.repeat(color[:, :, None], 3, axis=2)

        h, w = depth_m.shape[:2]
        ch, cw = color.shape[:2]
        sx = cw / float(w)
        sy = ch / float(h)

        fx = float(self.get_parameter("fx").value)
        fy = float(self.get_parameter("fy").value)
        cx = float(self.get_parameter("cx").value)
        cy = float(self.get_parameter("cy").value)
        stride = max(1, int(self.get_parameter("stride").value))
        min_depth = float(self.get_parameter("min_depth").value)
        max_depth = float(self.get_parameter("max_depth").value)

        points = []
        encoding = (self.color_msg.encoding or "").lower() if self.color_msg else ""
        for v in range(0, h, stride):
            for u in range(0, w, stride):
                z = float(depth_m[v, u])
                if not math.isfinite(z) or z < min_depth or z > max_depth:
                    continue

                cu = min(cw - 1, int(u * sx))
                cv = min(ch - 1, int(v * sy))
                pix = color[cv, cu]
                if "rgb" in encoding:
                    r, g, b = int(pix[0]), int(pix[1]), int(pix[2])
                else:
                    b, g, r = int(pix[0]), int(pix[1]), int(pix[2])
                rgb = (r << 16) | (g << 8) | b

                x = (u - cx) * z / fx
                y = (v - cy) * z / fy
                points.append((x, y, z, rgb))

        header = msg.header
        header.frame_id = str(self.get_parameter("output_frame").value)
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1),
        ]
        self.pub.publish(point_cloud2.create_cloud(header, fields, points))


def main():
    rclpy.init()
    node = RgbdPointCloudNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

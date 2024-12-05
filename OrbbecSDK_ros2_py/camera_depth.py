import cv2
import time
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from pyorbbecsdk import Config
from pyorbbecsdk import OBSensorType
from pyorbbecsdk import Pipeline

ESC_KEY = 27
PRINT_INTERVAL = 1  # seconds
MIN_DEPTH = 20  # 20mm
MAX_DEPTH = 10000  # 10000mm


class TemporalFilter:
    def __init__(self, alpha):
        self.alpha = alpha
        self.previous_frame = None

    def process(self, frame):
        if self.previous_frame is None:
            result = frame
        else:
            result = cv2.addWeighted(
                frame, self.alpha, self.previous_frame, 1 - self.alpha, 0
            )
        self.previous_frame = result
        return result


class CameraNode(Node):
    def __init__(self):
        super().__init__("camera_node")
        self.publisher_ = self.create_publisher(Image, "camera/image_raw", 10)
        self.timer = self.create_timer(0.1, self.publish_image)  # 每0.1秒发布一次图像

        config = Config()
        self.pipeline = Pipeline()
        self.temporal_filter = TemporalFilter(alpha=0.5)
        try:
            profile_list = self.pipeline.get_stream_profile_list(
                OBSensorType.DEPTH_SENSOR
            )
            assert profile_list is not None
            depth_profile = profile_list.get_default_video_stream_profile()
            assert depth_profile is not None
            print("depth profile: ", depth_profile)
            config.enable_stream(depth_profile)
        except Exception as e:
            print(e)
            return
        self.pipeline.start(config)
        self.last_print_time = time.time()

        self.bridge = CvBridge()

    def publish_image(self):

        frames = self.pipeline.wait_for_frames(100)
        if frames is None:
            return
        depth_frame = frames.get_depth_frame()
        if depth_frame is None:
            return
        width = depth_frame.get_width()
        height = depth_frame.get_height()
        scale = depth_frame.get_depth_scale()

        depth_data = np.frombuffer(depth_frame.get_data(), dtype=np.uint16)
        depth_data = depth_data.reshape((height, width))

        depth_data = depth_data.astype(np.float32) * scale
        depth_data = np.where(
            (depth_data > MIN_DEPTH) & (depth_data < MAX_DEPTH), depth_data, 0
        )
        depth_data = depth_data.astype(np.uint16)
        # Apply temporal filtering
        depth_data = self.temporal_filter.process(depth_data)

        center_y = int(height / 2)
        center_x = int(width / 2)
        center_distance = depth_data[center_y, center_x]

        current_time = time.time()
        if current_time - self.last_print_time >= PRINT_INTERVAL:
            print("center distance: ", center_distance)
            self.last_print_time = current_time

        depth_image = cv2.normalize(
            depth_data, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U
        )
        depth_image = cv2.applyColorMap(depth_image, cv2.COLORMAP_JET)

        # 转换图像为ROS 2消息格式并发布
        image_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding="bgr8")
        self.publisher_.publish(image_msg)
        self.get_logger().info("Published image")

    def destroy_node(self):
        self.pipeline.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()

    try:
        rclpy.spin(camera_node)
    except KeyboardInterrupt:
        camera_node.get_logger().info("Shutting down camera node")
    finally:
        camera_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

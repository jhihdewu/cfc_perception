#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class RgbtSplitterNode(Node):
    def __init__(self):
        super().__init__('rgbt_splitter_node')

        # Declare parameters for topics
        self.declare_parameter('input_topic', '/rgbt/rgbt/compressed')
        self.declare_parameter('rgb_output_topic', '/rgb')
        self.declare_parameter('thermal_output_topic', '/t')
        self.declare_parameter('standardized_thermal_output_topic', '/t_standardized')

        self.bridge = CvBridge()

        # Retrieve parameters
        input_topic = self.get_parameter('input_topic').value
        rgb_output_topic = self.get_parameter('rgb_output_topic').value
        thermal_output_topic = self.get_parameter('thermal_output_topic').value
        standardized_thermal_output_topic = self.get_parameter('standardized_thermal_output_topic').value

        # SUBSCRIBER
        self.subscription = self.create_subscription(
            CompressedImage,
            input_topic,
            self.listener_callback,
            10)

        # PUBLISHERS
        self.rgb_publisher = self.create_publisher(Image, rgb_output_topic, 10)
        self.thermal_publisher = self.create_publisher(Image, thermal_output_topic, 10)
        self.standardized_thermal_publisher = self.create_publisher(Image, standardized_thermal_output_topic, 10)

        self.get_logger().info('BGRA RGBT Splitter Node has started.')
        self.get_logger().info(f'Publishing to {rgb_output_topic}, {thermal_output_topic}, and {standardized_thermal_output_topic}')

    def listener_callback(self, msg):
        try:
            bgra_cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgra8")
        except Exception as e:
            self.get_logger().error(f'Failed to convert compressed BGRA image: {e}')
            return

        height, width, channels = bgra_cv_image.shape

        # Split the 4-channel image
        b_chan, g_chan, r_chan, alpha_chan = cv2.split(bgra_cv_image)

        # Create the standard RGB image
        rgb_cv_image = cv2.merge((b_chan, g_chan, r_chan))

        # The alpha channel is our raw thermal data
        thermal_cv_image = alpha_chan

        # --- Create the Standardized Visualization Image ---
        # 1. Normalize the thermal image to use the full 0-255 range
        normalized_thermal = cv2.normalize(thermal_cv_image, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
        
        # 2. Apply a colormap to the normalized image
        # JET is a common choice, but others like INFERNO or VIRIDIS also work well
        colorized_thermal = cv2.applyColorMap(normalized_thermal, cv2.COLORMAP_JET)
        
        try:
            # Publish the standard RGB image
            rgb_msg = self.bridge.cv2_to_imgmsg(rgb_cv_image, "bgr8")
            rgb_msg.header = msg.header
            rgb_msg.width = width
            rgb_msg.height = height
            self.rgb_publisher.publish(rgb_msg)

            # Publish the raw single-channel thermal image
            thermal_msg = self.bridge.cv2_to_imgmsg(thermal_cv_image, "mono8")
            thermal_msg.header = msg.header
            thermal_msg.width = width
            thermal_msg.height = height            
            self.thermal_publisher.publish(thermal_msg)
            
            # --- Publish the new standardized image ---
            standardized_msg = self.bridge.cv2_to_imgmsg(colorized_thermal, "bgr8")
            standardized_msg.header = msg.header
            standardized_msg.width = width
            standardized_msg.height = height              
            self.standardized_thermal_publisher.publish(standardized_msg)

        except Exception as e:
            self.get_logger().error(f'Failed to convert or publish images: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = RgbtSplitterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

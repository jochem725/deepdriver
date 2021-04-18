#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#                                                                               #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#   You may not use this file except in compliance with the License.            #
#   You may obtain a copy of the License at                                     #
#                                                                               #
#       http://www.apache.org/licenses/LICENSE-2.0                              #
#                                                                               #
#   Unless required by applicable law or agreed to in writing, software         #
#   distributed under the License is distributed on an "AS IS" BASIS,           #
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.    #
#   See the License for the specific language governing permissions and         #
#   limitations under the License.                                              #
#################################################################################


import time
import signal
import threading
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from deepracer_interfaces_pkg.msg import (
    EvoSensorMsg,
    LineDeltaMsg,
)
from line_detection_pkg import constants, utils


class LineDetectionNode(Node):
    def __init__(self, qos_profile):
        """Create a ObjectDetectionNode."""
        super().__init__("line_detection_node")
        self.get_logger().info("line_detection_node started.")

        # Double buffer to hold the input images for detection.
        self.input_buffer = utils.DoubleBuffer(clear_data_on_get=True)

        # Check if the inference output needs to be published to localhost using web_video_server
        self.declare_parameter("PUBLISH_DISPLAY_OUTPUT")
        self.publish_display_output = (
            self.get_parameter("PUBLISH_DISPLAY_OUTPUT")
            .get_parameter_value()
            .bool_value
        )
        self.get_logger().info(f"Publish output set to {self.publish_display_output}")

        # Create subscription to sensor messages from camera.
        self.image_subscriber = self.create_subscription(
            EvoSensorMsg,
            constants.SENSOR_FUSION_TOPIC,
            self.on_image_received_cb,
            qos_profile,
        )

        # Creating publisher for display_image.
        self.display_image_publisher = self.create_publisher(
            Image, constants.DISPLAY_IMAGE_PUBLISHER_TOPIC, 10
        )

        # Publisher for inference results.
        self.line_result_publisher = self.create_publisher(
            LineDeltaMsg, constants.LINE_RESULT_PUBLISHER_TOPIC, qos_profile
        )

        self.bridge = CvBridge()

        # Launching a separate thread to run inference.
        self.stop_thread = False
        self.thread_initialized = False
        self.thread = threading.Thread(target=self.run_inference)
        self.thread.start()
        self.thread_initialized = True
        self.get_logger().info(
            f"Waiting for input images on {constants.SENSOR_FUSION_TOPIC}"
        )

    def wait_for_thread(self):
        """Function which joins the created background thread."""
        if self.thread_initialized:
            self.thread.join()
            self.get_logger().info("Thread joined")

    def thread_shutdown(self):
        """Function which sets the flag to shutdown background thread."""
        self.stop_thread = True

    def on_image_received_cb(self, sensor_data):
        """Call back for adding to the input double buffer whenever
           new sensor image is received from sensor_fusion_node.

        Args:
            sensor_data (EvoSensorMsg): Message containing sensor images and lidar data.
        """
        self.input_buffer.put(sensor_data)

    def run_inference(self):
        """Method for running inference on received input image."""

        try:
            while not self.stop_thread:
                # Get an input image from double buffer.
                sensor_data = self.input_buffer.get()
                start_time = time.time()

                image = self.bridge.imgmsg_to_cv2(sensor_data.images[0])

                height, width = image.shape[:2]

                # Blur so edge detection works nicer.
                blurred = cv2.GaussianBlur(image, (5, 5), 0)

                # First: convert to HSV for easy color segmentaton.
                hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
                hsl = cv2.cvtColor(blurred, cv2.COLOR_BGR2HLS)

                # Secment high luminance.
                hsl_mask = cv2.inRange(
                    hsl, np.array([0, 0, 175]), np.array([255, 255, 255])
                )

                # Segment all yellow lines (centerline of a standard DeepRacer track)
                hsv_mask = cv2.inRange(
                    hsv, np.array([15, 75, 100]), np.array([30, 255, 255])
                )

                mask = cv2.bitwise_and(hsl_mask, hsv_mask)
                edge_mask = cv2.Canny(mask, 100, 200)

                # Divide the image in 25 lines and compute the centers of the edges we detected.
                scanlines = np.linspace(0, height - 1, 25, dtype=np.int)
                center_points = []
                for y in scanlines:
                    positions = np.where(edge_mask[y, :] > 0)

                    # If there are at least two edges in a line we found a potential road line to follow.
                    if len(positions) > 0 and len(positions[0]) > 1:
                        average = np.int(np.mean(positions[0]))
                        center_points.append((y, average))

                heading_line = None
                num_detection_points = 3
                # We need to be confident before we can fit a line.
                if len(center_points) >= num_detection_points:
                    center_points = center_points[:num_detection_points]
                    points = list(zip(*center_points))
                    slope, intercept = np.polyfit(points[0], points[1], deg=1)
                    heading_line = np.poly1d([slope, intercept])

                    line_delta = LineDeltaMsg()
                    # We compare the center vertical line of the image frame (width / 2)
                    # with the position of the heading line at the top of the frame and then normalize.
                    # This gives us an approximate direction to steer towards.
                    line_delta.delta_x = (heading_line(height) - (width / 2)) / width

                    # Publish inference results.
                    self.line_result_publisher.publish(line_delta)

                if self.publish_display_output:
                    self.get_logger().info("Publishing display output")
                    display_image = image

                    mask = np.zeros(display_image.shape, display_image.dtype)
                    mask[:, :] = (0, 255, 0)
                    mask = cv2.bitwise_and(mask, mask, mask=edge_mask)
                    cv2.addWeighted(display_image, 0.5, mask, 0.5, 0.0, display_image)

                    # Now for all the points we found, approximate a polyline we can use for direction.
                    # Threshold: at least 5 points.
                    if heading_line is not None:
                        start = int(heading_line(0))
                        end = int(heading_line(height))
                        cv2.line(
                            display_image, (start, 0), (end, height), (0, 255, 0), 3
                        )

                    for (y, x) in center_points:
                        cv2.circle(display_image, (x, y), 3, (255, 0, 0), 2)

                    # Publish to display topic (Can be viewed on localhost:8080).
                    display_image = self.bridge.cv2_to_imgmsg(
                        np.array(display_image), "bgr8"
                    )

                    self.display_image_publisher.publish(display_image)

                self.get_logger().info(
                    f"Total execution time = {time.time() - start_time}"
                )
        except Exception as ex:
            self.get_logger().error(f"Failed inference step: {ex}")
            # Destroy the ROS Node running in another thread as well.
            self.destroy_node()
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    qos = QoSProfile(
        reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
        depth=1,
        history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    )

    try:
        line_detection_node = LineDetectionNode(qos)
        executor = MultiThreadedExecutor()

        def signal_handler(signum, frame):
            """Callback function to handle registered signal handler
               to join and stop executing running thread created.
            Args:
                signum: The signal number
                frame: the current stack frame (None or a frame object)
            """
            line_detection_node.get_logger().info("Signal Handler initiated")
            line_detection_node.thread_shutdown()
            line_detection_node.wait_for_thread()

        # Register SIGINT handler
        signal.signal(signal.SIGINT, signal_handler)

        rclpy.spin(line_detection_node, executor)

    except Exception as ex:
        line_detection_node.get_logger().error(
            f"Exception in Object Detection Node: {ex}"
        )
        line_detection_node.destroy_node()
        rclpy.shutdown()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    line_detection_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

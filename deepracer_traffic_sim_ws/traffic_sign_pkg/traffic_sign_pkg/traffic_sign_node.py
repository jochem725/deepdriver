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

"""
traffic_sign_node.py

This module creates the traffic_sign_node which is responsible for collecting inference results
from object_detection_pkg and post-processing the signs.

After postprocessing this module provides a suggested directives based on detected signs.

The node defines:
    inference_subscriber: A subscriber to the /object_detection_pkg/inference_results published
                      by the object_detection_pkg with inference data.
    traffic_sign_publisher: A publisher that publishes directives based on detected signs.
"""
import time
import signal
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from deepracer_interfaces_pkg.msg import InferResultsArray, TrafficSign
from traffic_sign_pkg import constants, utils, cv_utils


class TrafficSignNode(Node):
    def __init__(self, qos_profile):
        """Create a ObjectDetectionNode."""
        super().__init__("traffic_sign_node")
        self.get_logger().info("traffic_sign_node started.")

        # Double buffer to hold the input inference data.
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
            InferResultsArray,
            constants.INFERENCE_RESULT_TOPIC,
            self.on_inference_received_cb,
            qos_profile,
        )

        # Creating publisher for display_image.
        self.display_image_publisher = self.create_publisher(
            Image, constants.DISPLAY_IMAGE_PUBLISHER_TOPIC, 10
        )

        # Publisher for detection results.
        self.traffic_sign_publisher = self.create_publisher(
            TrafficSign, constants.TRAFFIC_SIGN_PUBLISHER_TOPIC, 10
        )

        self.bridge = CvBridge()

        # Launching a separate thread to run processing.
        self.stop_thread = False
        self.thread_initialized = False
        self.thread = threading.Thread(target=self.run_detection)
        self.thread.start()
        self.thread_initialized = True
        self.get_logger().info(
            f"Waiting for input data on {constants.INFERENCE_RESULT_TOPIC}"
        )

    def wait_for_thread(self):
        """Function which joins the created background thread."""
        if self.thread_initialized:
            self.thread.join()
            self.get_logger().info("Thread joined")

    def thread_shutdown(self):
        """Function which sets the flag to shutdown background thread."""
        self.stop_thread = True

    def on_inference_received_cb(self, inference_data):
        """Call back for adding to the input double buffer whenever
           new sensor image is received from sensor_fusion_node.

        Args:
            inference_data (InferResultsArray): Message containing inference results from object detection.
        """
        self.input_buffer.put(inference_data)

    def run_detection(self):
        """Method for running processing based on the received input data."""

        try:
            while not self.stop_thread:
                # Get an input data from double buffer (InferResultsArray)
                inference_results = self.input_buffer.get()
                start_time = time.time()

                # First get the input into a format we can work with.
                image = self.bridge.imgmsg_to_cv2(inference_results.images[0])

                results = inference_results.results  # InferResults object

                self.get_logger().info(
                    f"Got input data... Results: {len(inference_results.results)} Images: {len(inference_results.images)}"
                )

                # List of tuples (sign, bbox_size)
                detected = []

                # Process each inference result object detected:
                for res in results:
                    # First check detected label.
                    coco_label = constants.COCO_LABELS[res.class_label]
                    bounding_box = (
                        np.int(res.x_min),
                        np.int(res.y_min),
                        np.int(res.x_max),
                        np.int(res.y_max),
                    )
                    bounding_box_size = (bounding_box[2] - bounding_box[0]) * (
                        bounding_box[3] - bounding_box[1]
                    )

                    self.get_logger().info(f"Postprocessing {coco_label}")
                    if coco_label == "traffic light":
                        color = cv_utils.detect_traffic_light_color(image, bounding_box)
                        self.get_logger().info(f"Traffic detected -> {color}")

                        detected.append(
                            ("traffic_light_{}".format(color), bounding_box_size)
                        )
                    elif coco_label == "street sign":
                        # TODO :)
                        pass
                    elif coco_label == "stop sign":
                        # TODO :)
                        pass
                    else:
                        self.get_logger().info(f"No logic for label {coco_label}")

                    # TODO: Compute bounding box coverage? Used to make decision which to process first?

                # TODO: Publish detection results.
                if len(detected) > 0:
                    # Take object with highest bounding box.
                    sign, _ = list(sorted(detected, key=lambda x: x[1]))[-1]
                    traffic_message = TrafficSign()
                    traffic_message.detected_sign = sign
                    self.traffic_sign_publisher.publish(traffic_message)

                # TODO: Output debug data on top of input image.
                if self.publish_display_output:
                    self.get_logger().info("Publishing display output")
                    display_image = image

                    # Publish to display topic (Can be viewed on localhost:8080).
                    display_image = self.bridge.cv2_to_imgmsg(
                        np.array(display_image), "bgr8"
                    )

                    self.display_image_publisher.publish(display_image)

                self.get_logger().info(
                    f"Total execution time = {time.time() - start_time}"
                )
        except Exception as ex:
            self.get_logger().error(f"Failed detection step: {ex}")
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
        traffic_sign_node = TrafficSignNode(qos)
        executor = MultiThreadedExecutor()

        def signal_handler(signum, frame):
            """Callback function to handle registered signal handler
               to join and stop executing running thread created.
            Args:
                signum: The signal number
                frame: the current stack frame (None or a frame object)
            """
            traffic_sign_node.get_logger().info("Signal Handler initiated")
            traffic_sign_node.thread_shutdown()
            traffic_sign_node.wait_for_thread()

        # Register SIGINT handler
        signal.signal(signal.SIGINT, signal_handler)

        rclpy.spin(traffic_sign_node, executor)

    except Exception as ex:
        traffic_sign_node.get_logger().error(f"Exception in Traffic Sign Node: {ex}")
        traffic_sign_node.destroy_node()
        rclpy.shutdown()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    traffic_sign_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

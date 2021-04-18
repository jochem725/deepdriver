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

from deepracer_interfaces_pkg.msg import EvoSensorMsg, InferResults, InferResultsArray
from openvino.inference_engine import IECore
import ngraph as ng
from object_detection_pkg import constants, utils


class ObjectDetectionNode(Node):
    def __init__(self, qos_profile):
        """Create a ObjectDetectionNode."""
        super().__init__("object_detection_node")
        self.get_logger().info("object_detection_node started.")

        # Double buffer to hold the input images for inference.
        self.input_buffer = utils.DoubleBuffer(clear_data_on_get=True)

        # Get DEVICE parameter (CPU/MYRIAD) from launch file.
        self.declare_parameter("DEVICE")
        self.device = self.get_parameter("DEVICE").get_parameter_value().string_value
        if not self.device:
            self.device = constants.DEVICE

        # Check if the inference output needs to be published to localhost using web_video_server
        self.declare_parameter("PUBLISH_DISPLAY_OUTPUT")
        self.publish_display_output = (
            self.get_parameter("PUBLISH_DISPLAY_OUTPUT")
            .get_parameter_value()
            .bool_value
        )
        self.get_logger().info(f"Publish output set to {self.publish_display_output}")

        # Initialize Intel Inference Engine
        self.init_network()

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
        self.inference_result_publisher = self.create_publisher(
            InferResultsArray, constants.INFERENCE_RESULT_PUBLISHER_TOPIC, qos_profile
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

    def init_network(self):
        """Function which initializes Intel Inference Engine."""
        # Load OpenVINO Inference Engine.
        self.get_logger().info(f"Loading Inference Engine on {self.device}")
        self.ie = IECore()

        # Read and load the network.
        self.net = self.ie.read_network(
            model=constants.MODEL_XML, weights=constants.MODEL_BIN
        )
        self.func = ng.function_from_cnn(self.net)
        self.ops = self.func.get_ordered_ops()
        self.exec_net = self.ie.load_network(network=self.net, device_name=self.device)

        # Read expected input image info from network and prepare input blobs.
        # n: batch size, c: no. of channels, h: input height, w: input width
        for self.input_key in self.net.input_info:
            self.input_name = self.input_key
            self.n, self.c, self.h, self.w = self.net.input_info[
                self.input_key
            ].input_data.shape

        # Initializing to float for optimizing in later functions
        self.h = float(self.h)
        self.w = float(self.w)

        # Prepare output blobs
        self.out_blob = next(iter(self.net.outputs))

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

    def preprocess(self, sensor_data):
        """Method that preprocesses the input data to be provided for inference to network.

        Args:
            sensor_data (EvoSensorMsg): Contains sensor images and lidar data.

        Returns:
            image: Preprosessed image expected by the network.
        """

        image = self.bridge.imgmsg_to_cv2(sensor_data.images[0])
        ih, iw = image.shape[:-1]

        # Resize to required input size
        if (ih, iw) != (int(self.h), int(self.w)):
            image = cv2.resize(image, (int(self.w), int(self.h)))

        # Change data layout from HWC to CHW.
        image = image.transpose((2, 0, 1))
        return image

    def run_inference(self):
        """Method for running inference on received input image."""

        try:
            while not self.stop_thread:
                # Get an input image from double buffer.
                sensor_data = self.input_buffer.get()

                start_time = time.time()

                # Pre-process input.
                input_data = {}
                input_image = self.preprocess(sensor_data)
                input_data[self.input_name] = input_image

                # Perform Inference.
                res = self.exec_net.infer(inputs=input_data)

                # Read and postprocess output.
                res = res[self.out_blob]
                output_data = res[0][0]

                # Object to store infer results in.
                infer_results_array = InferResultsArray()
                infer_results_array.results = []  # List of InferResults objects.

                # Image for which inferences were done.
                infer_results_array.images = []
                infer_results_array.images = [
                    self.bridge.cv2_to_imgmsg(
                        np.array(input_image.transpose((1, 2, 0))), "bgr8"
                    )
                ]

                # For each detected model in the inference data:
                # - Check if confident enough (> CONFIDENCE_TRESHOLD)
                # - Check if belongs to one of the classes we're interested in.

                outputs = []

                for _, proposal in enumerate(output_data):
                    confidence = np.float(proposal[2])

                    if confidence <= constants.CONFIDENCE_THRESHOLD:
                        continue

                    # Human readable.
                    label_id = np.int(proposal[1])
                    label = constants.COCO_LABELS[label_id]

                    if label not in constants.DETECT_CLASSES:
                        continue

                    self.get_logger().info(
                        f"Detected {label} - confidence {confidence}"
                    )

                    xmin = np.int(self.w * proposal[3])
                    ymin = np.int(self.h * proposal[4])
                    xmax = np.int(self.w * proposal[5])
                    ymax = np.int(self.h * proposal[6])

                    # Compute bounding box, coordinates are in normalized format ([0, 1])
                    infer_result = InferResults()
                    infer_result.class_label = label_id
                    infer_result.class_prob = confidence
                    infer_result.x_min = np.float(xmin)  # Top left
                    infer_result.y_min = np.float(ymin)  # Top left
                    infer_result.x_max = np.float(xmax)  # Bottom right
                    infer_result.y_max = np.float(ymax)  # Bottom right

                    infer_results_array.results.append(infer_result)

                    outputs.append((label_id, confidence, xmin, ymin, xmax, ymax))

                if self.publish_display_output:
                    self.get_logger().info("Publishing display output")

                    # Change data layout from CHW to HWC.
                    display_image = input_data[self.input_name].transpose((1, 2, 0))

                    for (label_id, confidence, xmin, ymin, xmax, ymax) in outputs:
                        # Drawing bounding boxes on the image.
                        cv2.rectangle(
                            display_image,
                            (xmin, ymin),
                            (xmax, ymax),
                            (232, 35, 244),
                            2,
                        )
                        cv2.putText(
                            display_image,
                            "{} ({:.2f})".format(
                                constants.COCO_LABELS[label_id], confidence
                            ),
                            (xmin, ymin - 10),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.4,
                            (232, 35, 244),
                            2,
                        )

                    # Publish to display topic (Can be viewed on localhost:8080).
                    display_image = self.bridge.cv2_to_imgmsg(
                        np.array(display_image), "bgr8"
                    )

                    self.display_image_publisher.publish(display_image)

                # Publish inference results.
                self.inference_result_publisher.publish(infer_results_array)

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
        object_detection_node = ObjectDetectionNode(qos)
        executor = MultiThreadedExecutor()

        def signal_handler(signum, frame):
            """Callback function to handle registered signal handler
               to join and stop executing running thread created.
            Args:
                signum: The signal number
                frame: the current stack frame (None or a frame object)
            """
            object_detection_node.get_logger().info("Signal Handler initiated")
            object_detection_node.thread_shutdown()
            object_detection_node.wait_for_thread()

        # Register SIGINT handler
        signal.signal(signal.SIGINT, signal_handler)

        rclpy.spin(object_detection_node, executor)

    except Exception as ex:
        object_detection_node.get_logger().error(
            f"Exception in Object Detection Node: {ex}"
        )
        object_detection_node.destroy_node()
        rclpy.shutdown()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    object_detection_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

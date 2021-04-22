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
deepdriver_navigation_node.py

This module decides the action messages (servo control messages specifically angle
and throttle) to be sent out using the detection deltas from object_detection_node.

The node defines:
    detection_delta_subscriber: A subscriber to the /object_detection_pkg/object_detection_delta
                                published by the object_detection_pkg with the normalized delta
                                of the detected object position from the target (reference) position
                                with respect to x and y axes.
    The node defines:
    action_publisher: A publisher to publish the action (angle and throttle values).
    set_max_speed_service: A service to dynamically set MAX_SPEED_PCT representing
                           the max speed percentage scale as per request.
"""
import copy
import time
import signal
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from deepracer_interfaces_pkg.msg import ServoCtrlMsg, TrafficMsg
from deepracer_interfaces_pkg.srv import SetMaxSpeedSrv, SetLedCtrlSrv
from deepdriver_navigation_pkg import constants, utils, control_utils


class TrafficNavigationNode(Node):
    """Node responsible for deciding the action messages (servo control messages specifically angle
    and throttle) to be sent out using the detection deltas from object_detection_node.
    """

    def __init__(self, qos_profile):
        """Create a TrafficNavigationNode."""
        super().__init__("deepdriver_navigation_node")
        self.get_logger().info("deepdriver_navigation_node started.")

        # Double buffer to hold the input inferences from object detection.
        self.sign_msg_buffer = utils.DoubleBuffer(clear_data_on_get=True)
        self.line_msg_buffer = utils.DoubleBuffer(clear_data_on_get=True)

        # Creating publisher to publish action (angle and throttle).
        self.action_publisher = self.create_publisher(
            ServoCtrlMsg, constants.ACTION_PUBLISH_TOPIC, qos_profile
        )

        # Service to dynamically set MAX_SPEED_PCT.
        self.set_max_speed_service = self.create_service(
            SetMaxSpeedSrv, constants.SET_MAX_SPEED_SERVICE_NAME, self.set_max_speed_cb
        )

        # Service to dynamically set LED COLOR.
        set_led_color_cb_group = MutuallyExclusiveCallbackGroup()
        self.set_led_ctrl_client = self.create_client(
            SetLedCtrlSrv,
            constants.SET_LED_CTRL_SERVICE,
            callback_group=set_led_color_cb_group,
        )

        while not self.set_led_ctrl_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                f"{self.set_led_ctrl_client.srv_name} service not available, waiting again..."
            )

        # Create subscription to object detections from the traffic sign node.
        self.traffic_sign_results_subscriber = self.create_subscription(
            TrafficMsg,
            constants.TRAFFIC_SIGN_RESULTS_TOPIC,
            self.traffic_msg_cb,
            qos_profile,
        )

        self.lock = threading.Lock()

        # Boolean to control stop/start state.
        self.is_driving = True

        # Default maximum speed percentage (updated as per request using service call).
        self.max_speed_pct = constants.MAX_SPEED_PCT

        # Default LED colors to cycle between.
        self.led_blinking = False
        self.led_color = constants.RGB_COLOR_MAP["black"]

        # Create a background servo publish thread.
        self.stop_thread = False
        self.thread_initialized = False
        self.thread = threading.Thread(target=self.main_loop)
        self.thread.start()
        self.thread_initialized = True

        # Create a separate thread to deal with traffic sign input.
        self.stop_sign_thread = False
        self.sign_thread_initialized = False
        self.sign_thread = threading.Thread(target=self.sign_loop)
        self.sign_thread.start()
        self.sign_thread_initialized = True

        # Launching a separate thread to deal with the LED.
        self.stop_led_thread = False
        self.led_thread_initialized = False
        self.led_thread = threading.Thread(target=self.led_loop)
        self.led_thread.start()
        self.led_thread_initialized = True

        self.get_logger().info("Waiting for input...")

    def wait_for_thread(self):
        """Function which joins the created background thread."""
        if self.thread_initialized:
            self.thread.join()
            self.get_logger().info("Thread joined")

    def thread_shutdown(self):
        """Function which sets the flag to shutdown background thread."""
        self.stop_thread = True

    def traffic_msg_cb(self, msg):
        self.sign_msg_buffer.put(msg)

    def line_msg_cb(self, msg):
        self.line_msg_buffer.put(msg)

    def set_led_ctrl_cb(self, req, res):
        pass

    def set_max_speed_cb(self, req, res):
        """Callback which dynamically sets the max_speed_pct.

        Args:
            req (SetMaxSpeedSrv.Request): Request object with the updated
                                          max speed percentage.
            res (SetMaxSpeedSrv.Response): Response object with error(int) flag
                                           indicating successful max speed pct
                                           update.

        Returns:
            SetMaxSpeedSrv.Response: Response object with error(int) flag indicating
                                             successful max speed pct update.

        """
        self.lock.acquire()
        try:
            self.max_speed_pct = req.max_speed_pct
            self.get_logger().info(
                f"Incoming request: max_speed_pct: {req.max_speed_pct}"
            )
            res.error = 0
        except Exception as ex:
            self.get_logger().error(f"Failed set max speed pct: {ex}")
            res.error = 1
        finally:
            self.lock.release()
        return res

    def led_loop(self):
        """Function which runs in a separate thread and decides the actions the car should take.

        Args:
            msg: (ServoCtrlMsg): Message containing the angle and speed values based on the mapping
        """
        try:
            blink = False
            blink_interval = 1.0

            last_update = time.time()
            current_color = None
            while not self.stop_led_thread:
                if self.led_blinking:
                    current_time = time.time()
                    if current_time - last_update >= blink_interval:
                        blink = not blink
                        last_update = current_time
                else:
                    blink = False

                if blink:
                    selected_color = constants.RGB_COLOR_MAP["black"]
                else:
                    selected_color = self.led_color

                if current_color != selected_color:
                    LED_SCALING_FACTOR = 39215

                    set_led_color_req = SetLedCtrlSrv.Request()
                    r, g, b = selected_color

                    set_led_color_req.red = r * LED_SCALING_FACTOR
                    set_led_color_req.green = g * LED_SCALING_FACTOR
                    set_led_color_req.blue = b * LED_SCALING_FACTOR

                    self.set_led_ctrl_client.call_async(set_led_color_req)
                    current_color = copy.copy(selected_color)

                time.sleep(constants.DEFAULT_SLEEP)

        except Exception as ex:
            self.get_logger().error(f"Failed to update car LED: {ex}")
            self.is_driving = False

            # Destroy the ROS Node running in another thread as well.
            self.destroy_node()
            rclpy.shutdown()

    def update_led(self, color="black", blinking=False):
        self.led_color = constants.RGB_COLOR_MAP[color]
        self.led_blinking = blinking

    def update_driving_state(self, is_driving=False):
        self.is_driving = is_driving

    def sign_loop(self):
        """Function which runs in a separate thread and processes the input from signs."""

        try:
            while not self.stop_thread:
                # Get a new message to plan action.
                traffic_msg = self.sign_msg_buffer.get()
                signs = traffic_msg.signs
                lights = traffic_msg.lights

                # Merge the list of objects
                objects = signs + lights

                # TrafficLight and TrafficSign both have a distance field so we can sort to get the closest.
                objects = list(sorted(objects, key=lambda x: x.distance))
                closest_object = None

                if len(objects) > 0:
                    closest_object = objects[0]

                self.get_logger().info(f"Closest: {closest_object}")

                # If no object, clear the LED and continue driving.
                if not closest_object:
                    self.update_led()
                    self.update_driving_state(is_driving=True)
                    continue

                # If object too far away, clear the LED and continue driving.
                if closest_object.distance >= constants.DISTANCE_THRESHOLD:
                    self.update_led()
                    self.update_driving_state(is_driving=True)
                    continue

                # If object detected:
                if closest_object.type == "stop sign":
                    self.update_led(color="red", blinking=True)
                    self.update_driving_state(is_driving=False)
                elif closest_object.type == "traffic light":
                    self.update_led(color=closest_object.color)
                    self.update_driving_state(
                        is_driving=closest_object.color == "green"
                    )
                else:
                    self.get_logger().error(
                        f"No logic for object type {closest_object.type}"
                    )
                    # Stop the car for safety reasons.
                    self.update_driving_state(is_driving=False)
                    self.update_led()

        except Exception as ex:
            self.get_logger().error(f"Failed to process traffic sign input: {ex}")

            # Stop the car for safety reasons.
            self.update_driving_state(is_driving=False)

            # Stop the car
            msg = ServoCtrlMsg()
            msg.angle, msg.throttle = (
                constants.ActionValues.DEFAULT,
                constants.ActionValues.DEFAULT,
            )
            self.action_publisher.publish(msg)

            # Destroy the ROS Node running in another thread as well.
            self.destroy_node()
            rclpy.shutdown()

    def plan_action(self, delta_x):
        if not self.is_driving:
            # No Action
            return constants.ACTION_SPACE[1][constants.ActionSpaceKeys.CATEGORY]

        # For now only drive straight ahead.
        return constants.ACTION_SPACE[2][constants.ActionSpaceKeys.CATEGORY]

    def main_loop(self):
        """Function which runs in a separate thread and decides the actions
        the car should take based on the input from the traffic signs node.
        """

        msg = ServoCtrlMsg()
        msg.angle, msg.throttle = (
            constants.ActionValues.DEFAULT,
            constants.ActionValues.DEFAULT,
        )

        try:
            while not self.stop_thread:
                # Keep planning new actions, car may need to stop because of sign input.
                action_category = self.plan_action(0)
                msg.angle, msg.throttle = control_utils.get_mapped_action(
                    action_category, self.max_speed_pct
                )

                # Log the action.
                action = constants.ACTION_SPACE[action_category][
                    constants.ActionSpaceKeys.ACTION
                ]

                self.get_logger().info(f"Action -> {action}")

                # Publish blind action
                self.action_publisher.publish(msg)

                time.sleep(constants.DEFAULT_SLEEP)

        except Exception as ex:
            self.get_logger().error(f"Failed to publish action to servo: {ex}")

            # Stop the car
            msg = ServoCtrlMsg()
            msg.angle, msg.throttle = (
                constants.ActionValues.DEFAULT,
                constants.ActionValues.DEFAULT,
            )
            self.action_publisher.publish(msg)

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
        deepdriver_navigation_node = TrafficNavigationNode(qos)
        executor = MultiThreadedExecutor()

        def signal_handler(signum, frame):
            """Callback function to handle registered signal handler
               to join and stop executing running thread created.
            Args:
                signum: The signal number.
                frame: the current stack frame (None or a frame object).
            """
            deepdriver_navigation_node.get_logger().info("Signal Handler initiated")
            deepdriver_navigation_node.thread_shutdown()
            deepdriver_navigation_node.wait_for_thread()

        # Register SIGINT handler
        signal.signal(signal.SIGINT, signal_handler)
        rclpy.spin(deepdriver_navigation_node, executor)
    except Exception as ex:
        deepdriver_navigation_node.get_logger().error(
            f"Exception in TrafficNavigationNode: {ex}"
        )
        deepdriver_navigation_node.destroy_node()
        rclpy.shutdown()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    deepdriver_navigation_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

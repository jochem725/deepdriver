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
traffic_navigation_node.py

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
import time
import signal
import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy

from deepracer_interfaces_pkg.msg import ServoCtrlMsg
from deepracer_interfaces_pkg.srv import SetMaxSpeedSrv
from traffic_navigation_pkg import constants, utils


class TrafficNavigationNode(Node):
    """Node responsible for deciding the action messages (servo control messages specifically angle
    and throttle) to be sent out using the detection deltas from object_detection_node.
    """

    def __init__(self, qos_profile):
        """Create a TrafficNavigationNode."""
        super().__init__("traffic_navigation_node")
        self.get_logger().info("traffic_navigation_node started.")

        # Creating publisher to publish action (angle and throttle).
        self.action_publisher = self.create_publisher(
            ServoCtrlMsg, constants.ACTION_PUBLISH_TOPIC, qos_profile
        )

        # Service to dynamically set MAX_SPEED_PCT.
        self.set_max_speed_service = self.create_service(
            SetMaxSpeedSrv, constants.SET_MAX_SPEED_SERVICE_NAME, self.set_max_speed_cb
        )

        # Initializing the msg to be published.
        msg = ServoCtrlMsg()
        msg.angle, msg.throttle = (
            constants.ActionValues.DEFAULT,
            constants.ActionValues.DEFAULT,
        )

        self.lock = threading.Lock()
        # Default maximum speed percentage (updated as per request using service call).
        self.max_speed_pct = constants.MAX_SPEED_PCT

        # Create a background servo publish thread.
        self.stop_thread = False
        self.thread_initialized = False
        self.thread = threading.Thread(target=self.main_loop, args=(msg,))
        self.thread.start()
        self.thread_initialized = True
        self.get_logger().info("Waiting for input...")

    def wait_for_thread(self):
        """Function which joins the created background thread."""
        if self.thread_initialized:
            self.thread.join()
            self.get_logger().info("Thread joined")

    def thread_shutdown(self):
        """Function which sets the flag to shutdown background thread."""
        self.stop_thread = True

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

    def main_loop(self, msg):
        """Function which runs in a separate thread and decides the actions the car should take.

        Args:
            msg: (ServoCtrlMsg): Message containing the angle and speed values based on the mapping
        """
        try:
            while not self.stop_thread:
                # Get a new message to plan action.
                # TODO: Currently do nothing, take no action.
                msg.angle, msg.throttle = utils.get_mapped_action(1, self.max_speed_pct)
                # Publish msg based on action planned and mapped from a new object detection.
                self.action_publisher.publish(msg)

                self.get_logger().info("Main loop iteration...")

                # Sleep for a default amount of time before checking if new data is available.
                time.sleep(constants.DEFAULT_SLEEP)

        except Exception as ex:
            self.get_logger().error(f"Failed to publish action to servo: {ex}")
            # Stop the car
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
        traffic_navigation_node = TrafficNavigationNode(qos)
        executor = MultiThreadedExecutor()

        def signal_handler(signum, frame):
            """Callback function to handle registered signal handler
               to join and stop executing running thread created.
            Args:
                signum: The signal number.
                frame: the current stack frame (None or a frame object).
            """
            traffic_navigation_node.get_logger().info("Signal Handler initiated")
            traffic_navigation_node.thread_shutdown()
            traffic_navigation_node.wait_for_thread()

        # Register SIGINT handler
        signal.signal(signal.SIGINT, signal_handler)
        rclpy.spin(traffic_navigation_node, executor)
    except Exception as ex:
        traffic_navigation_node.get_logger().error(
            f"Exception in TrafficNavigationNode: {ex}"
        )
        traffic_navigation_node.destroy_node()
        rclpy.shutdown()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    traffic_navigation_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

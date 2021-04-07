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

ACTION_PUBLISH_TOPIC = "traffic_drive"
SET_MAX_SPEED_SERVICE_NAME = "set_max_speed"

TRAFFIC_SIGN_PKG_NS = "/traffic_sign_pkg"
TRAFFIC_SIGN_RESULTS_TOPIC = f"{TRAFFIC_SIGN_PKG_NS}/traffic_sign_results"

LINE_PKG_NS = "/line_detection_pkg"
LINE_RESULTS_TOPIC = f"{LINE_PKG_NS}/line_results"


OBJECT_DETECTION_PKG_NS = "/object_detection_pkg"
OBJECT_DETECTION_INFERENCE_TOPIC = f"{OBJECT_DETECTION_PKG_NS}/inference_results"

SERVO_PKG_NS = "/servo_pkg"
SET_LED_CTRL_SERVICE = f"{SERVO_PKG_NS}/set_led_state"


class DeltaValueMap:
    # TODO UPDATE DOCS.
    FORWARD_RIGHT_DELTA_X = 0.13
    FORWARD_FAST_RIGHT_DELTA_X = 0.33
    FORWARD_LEFT_DELTA_X = -0.13
    FORWARD_FAST_LEFT_DELTA_X = -0.33


class ActionSpaceKeys:
    """Class with keys for the action space."""

    ACTION = "action"
    ANGLE = "angle"
    THROTTLE = "throttle"
    CATEGORY = "category"


class ActionValues:
    """Class with the PWM values with respect to
    the possible actions that can be sent to servo, pertaining to
    the angle and throttle.
    """

    FORWARD = 0.7
    FAST_LEFT = 1.0
    SLOW_LEFT = 0.9
    FAST_RIGHT = -1.0
    SLOW_RIGHT = -0.9
    DEFAULT = 0.0


# Action Space configuration.
ACTION_SPACE = {
    1: {
        ActionSpaceKeys.ACTION: "No Action",
        ActionSpaceKeys.ANGLE: ActionValues.DEFAULT,
        ActionSpaceKeys.THROTTLE: ActionValues.DEFAULT,
        ActionSpaceKeys.CATEGORY: 1,
    },
    2: {
        ActionSpaceKeys.ACTION: "Forward",
        ActionSpaceKeys.ANGLE: ActionValues.DEFAULT,
        ActionSpaceKeys.THROTTLE: ActionValues.FORWARD,
        ActionSpaceKeys.CATEGORY: 2,
    },
    3: {
        ActionSpaceKeys.ACTION: "Slow Left, Forward",
        ActionSpaceKeys.ANGLE: ActionValues.SLOW_LEFT,
        ActionSpaceKeys.THROTTLE: ActionValues.FORWARD,
        ActionSpaceKeys.CATEGORY: 3,
    },
    4: {
        ActionSpaceKeys.ACTION: "Fast Left, Forward",
        ActionSpaceKeys.ANGLE: ActionValues.FAST_LEFT,
        ActionSpaceKeys.THROTTLE: ActionValues.FORWARD,
        ActionSpaceKeys.CATEGORY: 4,
    },
    5: {
        ActionSpaceKeys.ACTION: "Slow Right, Forward",
        ActionSpaceKeys.ANGLE: ActionValues.SLOW_RIGHT,
        ActionSpaceKeys.THROTTLE: ActionValues.FORWARD,
        ActionSpaceKeys.CATEGORY: 5,
    },
    6: {
        ActionSpaceKeys.ACTION: "Fast Right, Forward",
        ActionSpaceKeys.ANGLE: ActionValues.FAST_RIGHT,
        ActionSpaceKeys.THROTTLE: ActionValues.FORWARD,
        ActionSpaceKeys.CATEGORY: 6,
    },
}

# Max speed percentage on a scale between 0.0 and 1.0.
# The maximum speed value is used to non linearly map the raw value obtained for the forward
# and reverse throttle to the PWM values of the servo/motor.
# We use the maximum speed % to map to a range of [1.0, 5.0] speed scale values using the
# calculated coefficients of the equation y = ax^2 + bx.
# This allows us to recalculate the curve for each maximum speed % value and use that to
# map the speed values. The idea behind this mapping is a lower percentage of maximum speed %
# should map to a higher speed scale value while calculating the coefficients so that the curve
# is more flatter and the impact of actual speed values is less for lower max speed %.
MAX_SPEED_PCT = 0.75

# Action space mapped to on the vehicle for speed values of 0.8 and 0.4.
DEFAULT_SPEED_SCALES = [1.0, 0.8]
# Speed scale bounds to pick from while calculating the coefficients.
MANUAL_SPEED_SCALE_BOUNDS = [1.0, 5.0]

# Default value to sleep for in sec.
DEFAULT_SLEEP = 0.2

# To set the car LED color.
RGB_COLOR_MAP = {
    "black": (0, 0, 0),
    "red": (255, 0, 0),
    "green": (0, 255, 0),
    "yellow": (255, 165, 0),
    "blue": (0, 0, 255),
}

# Threshold before the car should take an action.
DISTANCE_THRESHOLD = 30

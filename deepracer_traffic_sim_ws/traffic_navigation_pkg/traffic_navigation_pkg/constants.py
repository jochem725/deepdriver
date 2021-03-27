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

from enum import Enum


class ActionSpaceTypes(Enum):
    """Enum containing the action space type values in model metadata."""
    DISCRETE = 1
    CONTINUOUS = 2


class ModelMetadataKeys():
    """Class with keys in the model metadata.json
    """
    ACTION_SPACE_TYPE = "action_space_type"
    CONTINUOUS_HIGH = "high"
    CONTINUOUS_LOW = "low"
    STEERING = "steering_angle"
    SPEED = "speed"


# Default action space
DEFAULT_ACTION_SPACE = [{ModelMetadataKeys.SPEED: 0.8, ModelMetadataKeys.STEERING: 30.0},
                        {ModelMetadataKeys.SPEED: 0.8, ModelMetadataKeys.STEERING: -30.0},
                        {ModelMetadataKeys.SPEED: 0.8, ModelMetadataKeys.STEERING: 0.0},
                        {ModelMetadataKeys.SPEED: 0.8, ModelMetadataKeys.STEERING: 15.0},
                        {ModelMetadataKeys.SPEED: 0.8, ModelMetadataKeys.STEERING: -15.0},
                        {ModelMetadataKeys.SPEED: 0.4, ModelMetadataKeys.STEERING: 0.0}]

# Default scale values
DEFAULT_SPEED_SCALES = [1.0, 0.8]

# deepracer_navigation_node topics
AUTO_DRIVE_TOPIC_NAME = "auto_drive"
NAVIGATION_THROTTLE_SERVICE_NAME = "navigation_throttle"
LOAD_ACTION_SPACE_SERVICE_NAME = "load_action_space"

# inference results topic
INFERENCE_PKG_PKG_NS = "/inference_pkg"
INFERENCE_PKG_RL_RESULTS_TOPIC = f"{INFERENCE_PKG_PKG_NS}/rl_results"

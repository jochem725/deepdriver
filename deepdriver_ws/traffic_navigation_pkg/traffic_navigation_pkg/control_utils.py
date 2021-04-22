import math
from traffic_navigation_pkg import constants


def get_mapped_action(action_category, max_speed_pct):
    """Return the angle and throttle values to be published for servo.

    Args:
        action_category (int): Integer value corresponding to the action space category.
        max_speed_pct (float): Float value ranging from 0.0 to 1.0 taken as input
                                from maximum speed input.
    Returns:
        angle (float): Angle value to be published to servo.
        throttle (float): Throttle value to be published to servo.
    """
    angle = constants.ACTION_SPACE[action_category][constants.ActionSpaceKeys.ANGLE]
    categorized_throttle = constants.ACTION_SPACE[action_category][
        constants.ActionSpaceKeys.THROTTLE
    ]
    throttle = get_rescaled_manual_speed(categorized_throttle, max_speed_pct)
    return angle, throttle


def get_rescaled_manual_speed(categorized_throttle, max_speed_pct):
    """Return the non linearly rescaled speed value based on the max_speed_pct.

    Args:
        categorized_throttle (float): Float value ranging from -1.0 to 1.0.
        max_speed_pct (float): Float value ranging from 0.0 to 1.0 taken as input
                               from maximum speed input.
    Returns:
        float: Categorized value of the input speed.
    """
    # return 0.0 if categorized_throttle or maximum speed pct is 0.0.
    if categorized_throttle == 0.0 or max_speed_pct == 0.0:
        return 0.0

    # get the parameter value to calculate the coefficients a, b in the equation y=ax^2+bx
    # The lower the update_speed_scale_value parameter, higher the impact on the
    # final mapped_speed.
    # Hence the update_speed_scale_value parameter is inversely associated with max_speed_pct
    # and bounded by MANUAL_SPEED_SCALE_BOUNDS.
    # Ex: max_speed_pct = 0.5; update_speed_scale_value = 3
    #     max_speed_pct = 1.0; update_speed_scale_value = 1
    # Lower the update_speed_scale_value: categorized_throttle value gets mapped to
    # higher possible values.
    #   Example: update_speed_scale_value = 1.0;
    #            categorized_throttle = 0.8 ==> mapped_speed = 0.992
    # Higher the update_speed_scale_value: categorized_throttle value gets mapped to
    # lower possible values.
    #   Example: update_speed_scale_value = 3.0;
    #            categorized_throttle = 0.8 ==> mapped_speed = 0.501

    inverse_max_speed_pct = 1 - max_speed_pct
    update_speed_scale_value = constants.MANUAL_SPEED_SCALE_BOUNDS[
        0
    ] + inverse_max_speed_pct * (
        constants.MANUAL_SPEED_SCALE_BOUNDS[1] - constants.MANUAL_SPEED_SCALE_BOUNDS[0]
    )
    speed_mapping_coefficients = dict()

    # recreate the mapping coefficients for the non-linear equation ax^2 + bx based on
    # the update_speed_scale_value.
    # These coefficents map the [update_speed_scale_value, update_speed_scale_value/2]
    # values to DEFAULT_SPEED_SCALE values [1.0, 0.8].
    speed_mapping_coefficients["a"] = (1.0 / update_speed_scale_value ** 2) * (
        2.0 * constants.DEFAULT_SPEED_SCALES[0]
        - 4.0 * constants.DEFAULT_SPEED_SCALES[1]
    )
    speed_mapping_coefficients["b"] = (1.0 / update_speed_scale_value) * (
        4.0 * constants.DEFAULT_SPEED_SCALES[1] - constants.DEFAULT_SPEED_SCALES[0]
    )
    return math.copysign(1.0, categorized_throttle) * (
        speed_mapping_coefficients["a"] * abs(categorized_throttle) ** 2
        + speed_mapping_coefficients["b"] * abs(categorized_throttle)
    )

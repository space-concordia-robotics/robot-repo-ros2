from enum import IntEnum


class VKBButtonLayout(IntEnum):
    """Button index mapping for the VKB NXT joystick as reported by joy_node.

    These values are zero-based indices into sensor_msgs/Joy.buttons[].
    They document the expected mapping — use joy_button_probe to verify that
    the indices match your specific hardware / driver enumeration before
    relying on them for safety-critical params (e.g. can_safety_node).
    """

    FIRST_FIRE   = 0
    SECOND_FIRE  = 1
    A2           = 2
    B2           = 3
    D1           = 4
    A3_UP        = 5
    A3_RIGHT     = 6
    A3_DOWN      = 7
    A3_LEFT      = 8
    A3_PRESS     = 9
    A4_UP        = 10
    A4_RIGHT     = 11
    A4_DOWN      = 12
    A4_LEFT      = 13
    A4_PRESS     = 14
    C1_UP        = 15
    C1_RIGHT     = 16
    C1_DOWN      = 17
    C1_LEFT      = 18
    C1_PRESS     = 19
    TRIGGER_UP   = 20
    TRIGGER_DOWN = 21
    EN1_UP       = 22
    EN1_DOWN     = 23
    EN2_UP       = 24
    EN2_DOWN     = 25
    F1           = 26  # reserved — can_safety_node wheel_force_stop_button
    F2           = 27  # reserved — can_safety_node wheel_resume_button
    F3           = 28


class VKBAxesLayout(IntEnum):
    """Axis index mapping for the VKB NXT joystick as reported by joy_node."""

    STICK_X        = 0
    STICK_Y        = 1
    STICK_Z        = 5
    MIDDLE_SCROLL  = 2
    A1_X_LED_ON    = 3
    A1_Y_LED_ON    = 4
    A1_X_LED_OFF   = 8
    A1_Y_LED_OFF   = 9

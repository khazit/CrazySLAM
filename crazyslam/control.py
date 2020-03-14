import os
from pynput import keyboard


def get_key_capture():
    return keyCapture


def on_release(key):
    """Triggered by key release event. Save released key to global variable"""
    global keyCapture
    keyCapture = key
    if key == keyboard.Key.esc:
        return False


def key_controller(mc, current_velocity_x, current_velocity_y):
    """Manage the keyboard control commands

    Keymap:
        z:      Forward
        q:      Left
        s:      Backward
        d:      Right
        a:      Yaw left
        e:      Yaw right
        DOWN:   Land (Crazyflie lands softly)
        ENTER:  Abort (Shut down motors)

    Each command increments the speed in the proper axis/direction by VELOCITY
    and then starts a linear motion with the updated velocity values on both
    x and y axis.
    Yaw commands however are blocking: at each press, the Crazyflie yaw for
    YAW_DEG degrees.

    Args:
        mc: MotionCommander object
        current_velocity_x: Current velocity on the x axis
        current_velocity_y: CUrrent velocity on the y axis

    Returns:
        bool: True if ABORT command received
        bool: True if LAND command received
        float: Velocity along the x direction
        float: Velocity along the y direction
    """
    # Init constants
    global keyCapture
    VELOCITY = float(os.environ["VELOCITY"])
    YAW_DEG = float(os.environ["YAW_DEG"])
    RATE = float(os.environ["RATE"])

    # ABORT
    if keyCapture == keyboard.Key.enter:
        return True, False, None, None
    # OK LANDO, BOX BOX BOX
    elif keyCapture == keyboard.Key.down:
        mc.stop()
        return False, True, 0, 0
    # Control commands
    elif keyCapture.char == 'z':
        current_velocity_x += VELOCITY
    elif keyCapture.char == 'q':
        current_velocity_y += VELOCITY
    elif keyCapture.char == 's':
        current_velocity_x -= VELOCITY
    elif keyCapture.char == 'd':
        current_velocity_y -= VELOCITY
    elif keyCapture.char == 'a':
        mc.turn_left(angle_degrees=YAW_DEG, rate=RATE)
    elif keyCapture.char == 'e':
        mc.turn_right(angle_degrees=YAW_DEG, rate=RATE)
    # Start motion
    mc.start_linear_motion(
        current_velocity_x,
        current_velocity_y,
        0
    )
    keyCapture = None
    return False, False, current_velocity_x, current_velocity_y

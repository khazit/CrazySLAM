"""

"""

import time
import logging
from getkey import getkey, keys

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)
logger = logging.getLogger()

VELOCITY = 0.1      # meters/second
RATE = 360.0 / 5    # degrees/second
YAW_DEG = 10        # degrees


def connect():
    """Find the adress of the first available Crazyflie"""
    cflib.crtp.init_drivers(enable_debug_driver=False)
    available = cflib.crtp.scan_interfaces()
    assert len(available) != 0, "No Crazyflie found"
    return available[0][0]


def keyController(mc, current_velocity_x, current_velocity_y):
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
    key = getkey()
    # ABORT
    if key == keys.ENTER:
        return True, False, None, None
    # OK LANDO, BOX BOX BOX
    elif key == keys.DOWN:
        mc.stop()
        return False, True, 0, 0
    # Control commands
    elif key == 'z':
        current_velocity_x += VELOCITY
    elif key == 'q':
        current_velocity_y += VELOCITY
    elif key == 's':
        current_velocity_x -= VELOCITY
    elif key == 'd':
        current_velocity_y -= VELOCITY
    elif key == 'a':
        mc.turn_left(angle_degrees=YAW_DEG, rate=RATE)
    elif key == 'e':
        mc.turn_right(angle_degrees=YAW_DEG, rate=RATE)
    # Start motion
    mc.start_linear_motion(
        current_velocity_x,
        current_velocity_y,
        0
    )
    return False, False, current_velocity_x, current_velocity_y


if __name__ == '__main__':
    cf = Crazyflie(rw_cache="cache")
    abort, land = False
    velocity_x = 0.0
    velocity_y = 0.0
    with SyncCrazyflie(connect(), cf=cf) as scf:
        with MotionCommander(scf, default_height=1) as mc:
            while not abort and not land:
                abort, land, velocity_x, velocity_y = keyController(
                    mc,
                    velocity_x,
                    velocity_y
                )
                # TODO: replace with log function?
                print(velocity_x, velocity_y)
                time.sleep(0.1)
            if abort:
                cf.send_stop_setpoint()

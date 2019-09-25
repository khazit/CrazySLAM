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

VELOCITY = 0.2 # meters/second
RATE = 360.0 / 5 # degrees/second


def connect():
    """
    """
    cflib.crtp.init_drivers(enable_debug_driver=False)
    available = cflib.crtp.scan_interfaces()
    assert len(available)!=0, "No Crazyflie found"
    return available[0][0]

def keyController(mc, velocity_x, velocity_y):
    """
        z : forward
        q : left
        s : backward
        d : right
        a : yaw left
        e : yaw right
        down : abort
    """
    mc.stop()
    key = getkey()
    if key == 'z':
        velocity_x += VELOCITY
    elif key == 'q':
        velocity_y += VELOCITY
    elif key == 's':
        velocity_x -= VELOCITY
    elif key == 'd':
        velocit_y -= VELOCITY
    elif key == 'a':
        mc.start_turn_left(rate=RATE)
    elif key == 'e':
        mc.start_turn_right(rate=RATE)
    elif key == keys.DOWN:
        return True, 0, 0
    return False, velocity_x, velocity_y

if __name__ == '__main__':
    cf = Crazyflie(rw_cache="cache")
    abort = False
    with SyncCrazyflie(connect(), cf=cf) as scf:
        with MotionCommander(scf, default_height=1) as mc:
            while not abort:
                abort, velocity_x, velocit_y = keyController(mc)
                motion_commander.start_linear_motion(
                    velocity_x,
                    velocity_y,
                    0
                )
                time.sleep(0.1)

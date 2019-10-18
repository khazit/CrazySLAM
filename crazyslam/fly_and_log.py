"""
"""

import os
import time
import logging
from pynput import keyboard

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

from utils import connect
from control import key_controller, on_release, get_key_capture


# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)
logger = logging.getLogger()

listener = keyboard.Listener(
    on_release=on_release
)
listener.start()

os.environ["VELOCITY"] = str(0.1)      # meters/second
os.environ["RATE"] = str(360.0 / 5)    # degrees/second
os.environ["YAW_DEG"] = str(10)        # degrees


if __name__ == '__main__':
    # Initialization
    cf = Crazyflie(rw_cache="cache")
    abort = False
    land = False
    velocity_x = 0.0
    velocity_y = 0.0

    # Main loop
    with SyncCrazyflie(connect(), cf=cf) as scf:
        with MotionCommander(scf, default_height=1) as mc:
            while not abort and not land:
                if get_key_capture() is not None:
                    abort, land, velocity_x, velocity_y = key_controller(
                        mc,
                        velocity_x,
                        velocity_y
                    )
                # TODO: replace with logging function?
                print(velocity_x, velocity_y)
                time.sleep(0.1)
            if abort:
                cf.commander.send_stop_setpoint()

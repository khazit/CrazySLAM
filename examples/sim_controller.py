import sys
sys.path.append("")
from crazyslam.control import *


listener = keyboard.Listener(
    on_release=on_release
)
listener.start()
keyCapture = None


if __name__ == '__main__':
    os.environ["VELOCITY"] = str(0.1)      # meters/second
    os.environ["RATE"] = str(360.0 / 5)    # degrees/second
    os.environ["YAW_DEG"] = str(10)        # degrees
    cf = Crazyflie(rw_cache="cache")
    abort = False
    land = False
    velocity_x = 0.0
    velocity_y = 0.0
    with SyncCrazyflie(get_address(), cf=cf) as scf:
        with MotionCommander(scf, default_height=0.1) as mc:
            while not abort and not land:
                if get_key_capture() is not None:
                    abort, land, velocity_x, velocity_y = key_controller(
                        mc,
                        velocity_x,
                        velocity_y
                    )
                print(velocity_x, velocity_y)
                time.sleep(0.1)
            if abort:
                cf.commander.send_stop_setpoint()

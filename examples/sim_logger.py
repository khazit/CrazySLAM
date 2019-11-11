from crazyslam.logging import *


if __name__ == '__main__':
    from cflib.crazyflie import Crazyflie
    from utils import get_address
    import time
    i = 0
    cf = Crazyflie(rw_cache="cache")
    with SyncCrazyflie(get_address(), cf=cf) as scf:
        init_log_conf(scf, logging_callback)
        while i < 50:
            print("Sleep")
            time.sleep(0.5)
            i += 1

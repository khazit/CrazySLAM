import sys
from crazyslam.logging import *


def logging_callback(timestamp, data, logconf):
    """Receive the data at each timestamp"""
    print("New data received")
    print(data)
    write_to_disk("data", timestamp, data)


if __name__ == '__main__':
    from cflib.crazyflie import Crazyflie
    from crazyslam.utils import get_address
    import time
    i = 0
    cf = Crazyflie(rw_cache="cache")
    with SyncCrazyflie(get_address(), cf=cf) as scf:
        init_log_conf(scf, logging_callback)
        while i < 50:
            print("Sleep")
            time.sleep(0.5)
            i += 1

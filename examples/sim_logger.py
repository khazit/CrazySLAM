import sys
import time
import logging
from crazyslam.logging import *
from cflib.crazyflie import Crazyflie
from crazyslam.utils import get_address


def logging_callback(timestamp, data, logconf):
    """Receive the data at each timestamp"""
    print("New data received", data)
    epoch_id = os.listdir("log")[0].split("_")[1]
    write_to_disk(
        logconf.name,
        int(time.time()),
        data
    )


if __name__ == '__main__':
    start = int(time.time())
    end = start + int(sys.argv[1])
    cf = Crazyflie(rw_cache="cache")
    data_dir = "log"
    with SyncCrazyflie(get_address(), cf=cf) as scf:
        init_log_conf(
            scf, logging_callback, data_dir)
        while int(time.time()) < end:
            pass

"""Logging module

This module logs variables that are coming from the Crazyflie (range data and a
state estimate (x, y and yaw)) that are needed for the slam algorithm. Also
writes all the logged data to disk for post flight analysis
"""

import os
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie


def init_log_conf(scf, callback):
    """Initialize and start the logging

    Args:
        scf: Synced Crazyflie
        callback: Function called when new data is received

    """
    log_conf = LogConfig(name='MainLog', period_in_ms=1000)

    # Logged variables
    # Multiranger
    log_conf.add_variable('range.zrange', 'uint16_t')
    log_conf.add_variable('range.up', 'uint16_t')
    log_conf.add_variable('range.front', 'uint16_t')
    log_conf.add_variable('range.back', 'uint16_t')
    log_conf.add_variable('range.left', 'uint16_t')
    log_conf.add_variable('range.right', 'uint16_t')
    # State estimate (x, y, yaw)
    log_conf.add_variable('stateEstimate.x', 'float')
    log_conf.add_variable('stateEstimate.y', 'float')
    log_conf.add_variable('controller.yaw', 'float')

    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(callback)
    log_conf.start()


def write_to_disk(dir, timestamp, data):
    """Log data to disk"""
    # TODO: create dir/file if they don't exist. File with header
    assert os.path.isdir(dir), "No data directory found"
    filename = "data"
    data_point = str(timestamp)
    for key, value in data.items():
        data_point += ", {}".format(value)
    data_point += "\n"
    f = open(os.path.join(dir, filename), "a")
    f.write(data_point)
    f.close()

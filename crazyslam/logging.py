"""Logging module

This module logs variables that are coming from the Crazyflie (range data and a
state estimate (x, y and yaw)) and that are needed for the slam algorithm. Also
writes all the logged data to disk for post flight analysis

PROBLEM: File name have to be hardcoded in callback function
"""

import os
import time
from cflib.crazyflie.log import LogConfig


def init_log_conf(scf, callback, data_dir):
    """Initialize and start the logging

    Args:
        scf: Synced Crazyflie
        callback: Function called when new data is received
        data_dir: Directory where the log file will be saved
    """
    log_conf = LogConfig(name='MainLog', period_in_ms=100)

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
    log_conf.add_variable('stabilizer.yaw', 'float')

    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(callback)
    log_conf.name = start_file_manager(data_dir)
    log_conf.start()


def start_file_manager(data_dir):
    """Log files manager

    Make sure the data directory exists and create the blank log file with
    right header

    Args:
        data_dir: Path to the log files directory

    Returns:
        Path to the logging file
    """
    header = "timestamp,down,up,front,back,left,right,x,y,yaw\n"  # File header
    epoch = int(time.time())
    filename = "flight_{}.log".format(epoch)

    # If directory doesn't exist, create one
    if not os.path.isdir(data_dir):
        os.mkdir(data_dir)
    # Create sub directory for log files and figures
    sub_dir = os.path.join(data_dir, "flight_{}".format(epoch))
    os.mkdir(sub_dir)

    # Write header
    print("Log file :", filename)
    file = open(os.path.join(sub_dir, filename), "a")
    file.write(header)
    file.close()

    return os.path.join(data_dir, "flight_{}".format(epoch), filename)


def write_to_disk(file_path, timestamp, data):
    """Log data to disk"""
    assert os.path.isfile(file_path), "No log file found @ " + file_path

    # Concatenate all fieds into one string
    data_point = str(timestamp)
    for _, value in data.items():
        data_point += ",{}".format(value)
    data_point += "\n"

    # Write data
    file = open(file_path, "a")
    file.write(data_point)
    file.close()

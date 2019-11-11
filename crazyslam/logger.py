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
    # TODO: add state estimate : (x, y, yaw)

    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(callback)
    log_conf.start()


def write_to_disk(dir, timestamp, data):
    """Log data to disk"""
    assert os.path.isdir(dir), "No data directory found"
    filename = "data"
    data_point = str(timestamp)
    for key, value in data.items():
        data_point += ", {}".format(value)
    data_point += "\n"
    f = open(os.path.join(dir, filename), "a")
    f.write(data_point)
    f.close()


def logging_callback(timestamp, data, logconf):
    """Receive the data at each timestamp"""
    # NOTE: maybe define it in main.py
    print("New data received")
    write_to_disk("data", timestamp, data)

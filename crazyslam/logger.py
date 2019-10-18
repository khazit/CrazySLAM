from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig


def init_log_conf(scf, callback):
    """Initialize and start the logging

    Args:
        scf: Synced Crazyflie
        callback: Function called when new data is received

    """
    log_conf = LogConfig(name='MainLog', period_in_ms=500)

    # Logged variables
    log_conf.add_variable('range.zrange', 'uint16_t')

    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(callback)
    log_conf.start()


def logging_callback(timestamp, data, logconf):
    """Receive the data at each timestamp"""
    x = data['range.zrange']
    print('zrange: ({})'.format(x))
    print(timestamp, type(timestamp))


if __name__ == '__main__':
    from cflib.crazyflie import Crazyflie
    from utils import get_address
    import time

    cf = Crazyflie(rw_cache="cache")
    with SyncCrazyflie(get_address(), cf=cf) as scf:
        init_log_conf(scf, logging_callback)
        while 1:
            print("Sleep")
            time.sleep(0.5)

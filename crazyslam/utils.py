import cflib.crtp


def connect():
    """Find the address of the first available Crazyflie"""
    cflib.crtp.init_drivers(enable_debug_driver=False)
    available = cflib.crtp.scan_interfaces()
    assert len(available) != 0, "No Crazyflie found"
    return available[0][0]

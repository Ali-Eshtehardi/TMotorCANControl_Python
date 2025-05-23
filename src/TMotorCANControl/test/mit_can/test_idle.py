from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
try:
     from TMotorCANControl.mit_can import TMotorManager
except ModuleNotFoundError:
    from sys import path
    path.append("/home/pi/TMotorCANControl/src")
    from TMotorCANControl.mit_can import TMotorManager
import time


with TMotorManager(motor_type='AK80-9', motor_ID=3, CSV_file="log.csv") as dev:
    dev.zero_position() # has a delay!
    time.sleep(1.5)
    # dev.set_current_gains()
    loop = SoftRealtimeLoop(dt=0.01, report=True, fade=0.0)
    for t in loop:
        dev.update()
        print("\r" + str(dev) + "    Temp: " + str(dev.T) + "C   Error: " + str(dev.get_motor_error_code()),end='')
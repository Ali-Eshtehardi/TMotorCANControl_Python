from sys import path
path.append("/home/pi/TMotorCANControl/src/")
from TMotorCANControl.TMotorManager_servo_serial import *

vel = 1

with TMotorManager_servo_serial(port = '/dev/ttyUSB0', baud=961200, motor_params=Servo_Params_Serial['AK80-9']) as dev1:
    with TMotorManager_servo_serial(port = '/dev/ttyUSB1', baud=961200, motor_params=Servo_Params_Serial['AK80-9']) as dev2:
        loop = SoftRealtimeLoop(dt=0.005, report=True, fade=0.0)
        dev1.enter_position_control()
        dev2.enter_idle_mode()
        
        dev1.set_zero_position()
        dev2.set_zero_position()

        dev1.update()
        dev2.update()
        
        for t in loop:
            dev1.set_position(dev2.θ)
            dev1.update()
            dev2.update()
            print("\r" + str(dev1) + str(dev2), end='')

        
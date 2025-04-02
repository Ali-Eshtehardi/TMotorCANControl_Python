

from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
from sys import path
path.append("/home/pi/TMotorCANControl/src/")
from TMotorCANControl.servo_can import TMotorManager_servo_can
import time
import math

with TMotorManager_servo_can(motor_type='AK80-9', motor_ID=104) as dev:
    dev.set_zero_position()
    time.sleep(1.5)
    loop = SoftRealtimeLoop(dt=0.01, report=True, fade=0.0)
    dev.enter_position_velocity_control()
    
    for t in loop:
        target_position = 90 
        max_velocity = 10.0 
        max_acceleration = 5.0  
        
        dev.set_output_angle_radians(target_position, max_velocity, max_acceleration)
        dev.update()
        print("\r" + str(dev), end='')

# why is motor ID 104 ? 
#00002968 (hex) = 0000 0010 1001 0110 1000 (binary)
#The lowest 8 bits are 01101000, which is 0x68 in hex, or 104 in decimal


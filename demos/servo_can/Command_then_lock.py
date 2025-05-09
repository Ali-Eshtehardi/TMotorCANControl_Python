from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
from sys import path
path.append("/home/pi/TMotorCANControl/src/")
from TMotorCANControl.servo_can import TMotorManager_servo_can
import time



with TMotorManager_servo_can(motor_type='AK80-9', motor_ID=104) as dev:
    target_limit = -50    # double check this value
    dev.set_zero_position()
    time.sleep(1.5)
    loop = SoftRealtimeLoop(dt=0.01, report=True, fade=0.0)
    dev.enter_position_velocity_control()
    
    for t in loop:
        target_position = -90             # in radian 
        max_velocity = 1.0
        max_acceleration = 0.1  
        
        dev.set_output_angle_radians(target_position, max_velocity, max_acceleration)
        dev.update()
        print("\r" + str(dev), end='')
    

        
    

        # If the motor reaches or exceeds 20 degrees, lock it in place
        if target_limit >= target_position:
            
            # Lock motor in place (set to duty cycle control mode)
            dev.enter_duty_cycle_control()
            dev.set_duty_cycle_percent()  # Lock motor in place (no active movement)
            dev.update()

        time.sleep(0.01)  # Delay to reduce rapid updates



        #write a code thats alaways looping thats waiting for a target angle , like type in a position, 
        # like not like running again, but loop listening to a target position, that goes to the desirted positon from keyboard, once it 
        # goes to that angle , lets say 200 , and then give another angle command that is 220 , once it reached the 220 , lock , 
        # do the same for 180 degrees. 


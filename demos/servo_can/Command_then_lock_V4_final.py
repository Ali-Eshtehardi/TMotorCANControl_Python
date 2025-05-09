from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
from sys import path
path.append("/home/pi/TMotorCANControl/src/")
from TMotorCANControl.servo_can import TMotorManager_servo_can
import time
import threading
import queue
import math
import numpy as np

DEG_TO_RAD = math.pi / 180.0  
RAD_TO_DEG = 180.0 / math.pi  
FREE_DUTY_CYCLE = 0  # Zero duty cycle for minimum resistance
POSITION_TOLERANCE = 1  # degrees

def Questions(target_queue):
    while True:
        try:
            user_input = input("\n Enter Target Position in degrees - Press Z to cancel - ")
        except Exception as e:
            print(f"\n Error reading input: {e}")

        if user_input.lower() == 'z':
            target_queue.put('cancel')
            break
        else:
            try:
                target = float(user_input)
                target_queue.put(target)
            except ValueError:
                print("Invalid target position. Please enter a number or 'Z' to cancel.")

def main():
    target = None
    motor_locked = False       
    target_queue = queue.Queue()
    input_thread = threading.Thread(target=Questions, args=(target_queue,), daemon=True)
    input_thread.start()


    max_velocity = 0.5  # rad/s
    max_acceleration = 0.1  # rad/s²

    try:
        with TMotorManager_servo_can(motor_type='AK80-9', motor_ID=104) as dev:
            
            dev.rad_per_Eang = DEG_TO_RAD  # This is fix
            
            dev.set_zero_position()
            time.sleep(1.5)
            
          
            print(f"Initial motor state: {dev}")
            
            loop = SoftRealtimeLoop(dt=0.01, report=True, fade=0.0)
            dev.enter_position_velocity_control()
            
            for t in loop:             
                
                try:
                    new_target = target_queue.get_nowait()
                    if new_target == 'cancel':
                        break  
                    else:
                        target = new_target
                        motor_locked = False 
                        print(f"\nNew target set: {target}°")
                        
                        print(f"Current motor state: {dev}")
                except queue.Empty:
                    pass

             
                if target is not None and not motor_locked:
                    
                    target_rad = target * DEG_TO_RAD
                    
                   ####################################################################################
                    scaling_factor = 1  
                    adjusted_target_rad = target_rad * scaling_factor
                    
                 
                    gear_ratio = 1.0
                    
                    ####################################################################################################
                    dev.set_output_angle_radians(adjusted_target_rad / gear_ratio, max_velocity, max_acceleration)
                    dev.update()
                    
                    
                    current_pos = dev.position * RAD_TO_DEG
                    angle_difference = abs(current_pos - target)
                    
                    print(f"\rCurrent pos: {current_pos:.2f}°, Target: {target}°, Diff: {angle_difference:.2f}°", end='')

                   
                    if angle_difference <= POSITION_TOLERANCE:
                        print(f"\nTarget reached! Current position: {current_pos:.2f}°. Locking motor...")
                        dev.enter_current_brake_control()
                        dev.set_motor_current_qaxis_amps(50)  
                        dev.update()
                        motor_locked = True
                        print("\nMotor LOCKED. Enter a new target position.\n")
                elif motor_locked:
                    dev.update() 
                    current_pos = dev.position * RAD_TO_DEG
                    print(f"\rMotor LOCKED at {current_pos:.2f}°. Enter a new target position.", end='')
                    time.sleep(0.5)  

                time.sleep(0.01)  

    except KeyboardInterrupt:
        print("\nProgram interrupted by user.")
    finally:
   
        if 'dev' in locals():
            dev.enter_duty_cycle_control()
            dev.set_duty_cycle_percent(FREE_DUTY_CYCLE)
            dev.update()
            time.sleep(0.5)
            print("\nMotor returned to free mode. Program ended.")

if __name__ == "__main__":
    main()
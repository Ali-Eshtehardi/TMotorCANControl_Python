from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop   #Imports a SoftRealtimeLoop class for timing control
from sys import path #Adding the path to the system path for module import
path.append("/home/pi/TMotorCANControl/src/")  #allows below import to work
from TMotorCANControl.servo_can import TMotorManager_servo_can
import time #Importing time for sleep functionality
import threading #Importing threading for concurrent input handling
import queue #Importing queue for thread-safe communication
import math 
import numpy as np #Importing numpy for numerical operations

DEG_TO_RAD = math.pi / 180.0  
RAD_TO_DEG = 180.0 / math.pi  
FREE_DUTY_CYCLE = 0  # Zero duty cycle for minimum resistance
POSITION_TOLERANCE = 1  # degrees

def Questions(target_queue):   # is a thread, and works in the terminal
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
    target = None   # these 2 are initialization of variable states
    motor_locked = False       
    target_queue = queue.Queue()  #creation of a queue for thread-safe communication
    input_thread = threading.Thread(target=Questions, args=(target_queue,), daemon=True)   # this is the thread that will run the Questions function in the terminal
    input_thread.start() #starts the above thread when u run the code


    max_velocity = 0.5  # rad/s
    max_acceleration = 0.1  # rad/s²

    try:
        with TMotorManager_servo_can(motor_type='AK80-9', motor_ID=104) as dev:
            
            dev.rad_per_Eang = DEG_TO_RAD  # This is fix
            
            dev.set_zero_position()
            time.sleep(1.5)
            
          
            print(f"Initial motor state: {dev}")
            
            loop = SoftRealtimeLoop(dt=0.01, report=True, fade=0.0)  # Creates a soft real-time loop with 10ms intervals , can also use time.sleep(0.01) instead of this
            # loop.start()  # Starts the loop <- this is not needed, as the loop is started in the class itself , good because we need it to continously ask for input
            dev.enter_position_velocity_control() # Puts the motor in position-velocity control mode
            
            for t in loop:             
                
                try:
                    new_target = target_queue.get_nowait() #Checks for new target values in the queue without waiting
                    if new_target == 'cancel': #If a 'cancel' message is received, it breaks out of the loop
                        break  
                    else:
                        target = new_target #If a new target is received, it updates the target and unlocks the motor
                        motor_locked = False 
                        print(f"\nNew target set: {target}°")
                        
                        print(f"Current motor state: {dev}")
                except queue.Empty:
                    pass

             
                if target is not None and not motor_locked: #If there's a target and the motor isn't locked
                    
                    target_rad = target * DEG_TO_RAD  #Converts the target from degrees to radians
                    
                   ####################################################################################
                    scaling_factor = 1  
                    adjusted_target_rad = target_rad * scaling_factor
                    
                 
                    gear_ratio = 1.0
                    
                    ####################################################################################################
                    dev.set_output_angle_radians(adjusted_target_rad / gear_ratio, max_velocity, max_acceleration)
                    dev.update()
                    
                    
                    current_pos = dev.position * RAD_TO_DEG #Updates the motor state
                    angle_difference = abs(current_pos - target) # Calculates the difference between the current position and the target
                    
                    print(f"\rCurrent pos: {current_pos:.2f}°, Target: {target}°, Diff: {angle_difference:.2f}°", end='')

                   
                    if angle_difference <= POSITION_TOLERANCE: #If the motor reaches the target position within tolerance
                        print(f"\nTarget reached! Current position: {current_pos:.2f}°. Locking motor...")
                        dev.enter_current_brake_control() # Switches to current brake control mode
                        dev.set_motor_current_qaxis_amps(50)  # Sets a big current to hold the position
                        dev.update()
                        motor_locked = True #Sets the motor_locked flag
                        print("\nMotor LOCKED. Enter a new target position.\n")
                elif motor_locked: #If the motor is locked:
                    dev.update() 
                    current_pos = dev.position * RAD_TO_DEG
                    print(f"\rMotor LOCKED at {current_pos:.2f}°. Enter a new target position.", end='')
                    time.sleep(0.5)  

                time.sleep(0.01)  

    except KeyboardInterrupt:
        print("\nProgram interrupted by user.")
    finally:    #below code will run when the program ends, turns off all the inhibitors and returns to free mode
   
        if 'dev' in locals():
            dev.enter_duty_cycle_control()
            dev.set_duty_cycle_percent(FREE_DUTY_CYCLE)
            dev.update()
            time.sleep(0.5)
            print("\nMotor returned to free mode. Program ended.")

if __name__ == "__main__":
    main()
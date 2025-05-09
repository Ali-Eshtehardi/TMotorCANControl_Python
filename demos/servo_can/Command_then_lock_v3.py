from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
from sys import path
path.append("/home/pi/TMotorCANControl/src/")
from TMotorCANControl.servo_can import TMotorManager_servo_can
import time
import threading
import queue
import math
import numpy as np

DEG_TO_RAD = math.pi / 180.0  # Convert degrees to radians
RAD_TO_DEG = 180.0 / math.pi  # Convert radians to degrees
FREE_DUTY_CYCLE = 0  # Zero duty cycle for minimum resistance
POSITION_TOLERANCE = 0.1  # degrees

def Questions(target_queue):
    while True:
        # Print a visible separator before asking for input
        print("\n" + "="*50)
        print(" INPUT MODE: Enter Target Position in degrees or Z to cancel ")
        print("="*50)
        
        try:
            user_input = input("\n>> ")
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

    try:
        with TMotorManager_servo_can(motor_type='AK80-9', motor_ID=104) as dev:
            # Override the rad_per_Eang value to fix the conversion issue
            # The original calculation might not be appropriate for your motor
            # Setting it to a more direct conversion between radians and degrees
            dev.rad_per_Eang = DEG_TO_RAD  # This is a key fix!
            
            dev.set_zero_position()
            time.sleep(1.5)
            
            # Print initial motor state for debugging
            print(f"Initial motor state: {dev}")
            
            loop = SoftRealtimeLoop(dt=0.01, report=True, fade=0.0)
            dev.enter_position_velocity_control()
            
            for t in loop:             
                max_velocity = 0.5
                max_acceleration = 0.1
                
                # Check for new target from input thread
                try:
                    new_target = target_queue.get_nowait()
                    if new_target == 'cancel':
                        break  # Exit loop if user cancels
                    else:
                        target = new_target
                        motor_locked = False # Unlock when new target is set
                        print(f"\nNew target set: {target}°")
                        # Print current motor state for debugging
                        print(f"Current motor state before moving: {dev}")
                except queue.Empty:
                    pass

                # Control motor to target position
                if target is not None and not motor_locked:
                    # Directly set the target in degrees
                    # Since we've overridden rad_per_Eang to be DEG_TO_RAD,
                    # this will correctly convert from degrees to the motor's internal units
                    dev.set_output_angle_radians((target * DEG_TO_RAD)/2, max_velocity, max_acceleration)
                    dev.update()
                    
                    # Calculate current position in degrees
                    current_pos = dev.position * RAD_TO_DEG
                    angle_difference = abs(current_pos - target)
                    
                    print(f"\rCurrent pos: {current_pos:.2f}°, Target: {target}°, Diff: {angle_difference:.2f}°", end='')

                    # Check if motor reached target position
                    if angle_difference <= POSITION_TOLERANCE:
                        print(f"\nTarget reached! Current position: {current_pos:.2f}°. Locking motor...")
                        dev.enter_current_brake_control()
                        dev.set_motor_current_qaxis_amps(50)  # Set appropriate brake current
                        dev.update()
                        motor_locked = True
                        print("\nMotor LOCKED. Enter a new target position.\n")
                elif motor_locked:
                    dev.update() # Keep brake current active
                    current_pos = dev.position * RAD_TO_DEG
                    print(f"\rMotor LOCKED at {current_pos:.2f}°. Enter a new target position.", end='')
                    time.sleep(0.5)  # Reduce update frequency when locked

                time.sleep(0.01)  # Delay to reduce rapid updates

    except KeyboardInterrupt:
        print("\nProgram interrupted by user.")
    finally:
        # Ensure motor is in free mode on exit
        if 'dev' in locals():
            dev.enter_duty_cycle_control()
            dev.set_duty_cycle_percent(FREE_DUTY_CYCLE)
            dev.update()
            time.sleep(0.5)
            print("\nMotor returned to free mode. Program ended.")

if __name__ == "__main__":
    main()
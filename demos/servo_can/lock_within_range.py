#!/usr/bin/env python3

import threading
import queue
import time
import math
from TMotorCANControl.servo_can import TMotorManager_servo_can

# Constants
DEG_TO_RAD = math.pi / 180.0
RAD_TO_DEG = 180.0 / math.pi
lock_current = 50  
FREE_DUTY_CYCLE = 0  
BRAKE_UPDATE_RATE = 0.01  

def get_target_input(target_queue):
    while True:
        try:
            user_input = input("\nEnter target position (degrees), 'z' to set current as zero, 'r' to return to zero, or 'q' to quit: ").strip().lower()
            if user_input == 'q':
                break
            elif user_input == 'z':
                target_queue.put('set_zero')
            elif user_input == 'r':
                target_queue.put(0)  # Return to zero position
            else:
                target = float(user_input)
                target_queue.put(target)
        except ValueError:
            print("Invalid input. Please enter a number or 'z', 'r', 'q'")

def main():
    target_queue = queue.Queue()
    current_target = None
    motor_locked = False
    position_tolerance = 1.0  # degrees
    
    # Start input thread
    input_thread = threading.Thread(target=get_target_input, args=(target_queue,), daemon=True)
    input_thread.start()
    
    try:
        with TMotorManager_servo_can(motor_type='AK80-9', motor_ID=104) as dev:
            print("Starting in FREE MODE. Enter a position (in degrees) to lock at.")
            dev.set_zero_position()
            # Initialize in duty cycle mode with zero resistance
            dev.enter_duty_cycle_control()
            dev.set_duty_cycle_percent(FREE_DUTY_CYCLE)
            dev.update()
            time.sleep(0.5)
            
            while True:
                # Get current position
                current_pos = dev.position * RAD_TO_DEG
                
                # Maintain brake current if locked
                if motor_locked:
                    dev.enter_current_brake_control()
                    dev.set_motor_current_qaxis_amps(lock_current)
                    dev.update()
                else:
                    # Keep motor in free mode with minimum resistance
                    dev.enter_duty_cycle_control()
                    dev.set_duty_cycle_percent(FREE_DUTY_CYCLE)
                    dev.update()
                
                # Check for new target input
                try:
                    new_target = target_queue.get_nowait()
                    if new_target == 'set_zero':
                        dev.set_zero_position()
                        print("\nCurrent position set as new zero")
                        current_target = None
                        motor_locked = False
                        dev.enter_duty_cycle_control()
                        dev.set_duty_cycle_percent(FREE_DUTY_CYCLE)
                        dev.update()
                    else:
                        current_target = new_target
                        motor_locked = False  # Reset locked state for new target
                        dev.enter_duty_cycle_control()
                        dev.set_duty_cycle_percent(FREE_DUTY_CYCLE)
                        dev.update()
                        print(f"\nNew target set: {current_target}°")
                        print("Motor will lock when position is reached.")
                except queue.Empty:
                    pass
                
                # Check if we should lock the motor
                if current_target is not None and not motor_locked:
                    at_target = abs(current_pos - current_target) <= position_tolerance
                    
                    if at_target:
                        print(f"\nPosition {current_target}° reached! Locking motor...")
                        dev.enter_current_brake_control()
                        dev.set_motor_current_qaxis_amps(lock_current)
                        dev.update()
                        motor_locked = True
                        print("Motor LOCKED. Enter a new position to set new lock target.")
                
                # Status display
                print(f"\rCurrent: {current_pos:>6.2f}° | Target: {current_target if current_target is not None else 'None':>6} | Status: {'LOCKED' if motor_locked else 'FREE'}", end='')
                
                time.sleep(BRAKE_UPDATE_RATE)  # Faster update rate for more consistent braking
            
    except KeyboardInterrupt:
        print("\nProgram terminated by user.")
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


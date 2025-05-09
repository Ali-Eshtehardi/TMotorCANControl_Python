#!/usr/bin/env python3

import threading
import queue
import time
import math
from TMotorCANControl.servo_can import TMotorManager_servo_can

# Constants
DEG_TO_RAD = math.pi / 180.0
RAD_TO_DEG = 180.0 / math.pi
lock_current = 50  # Brake current (Amps)
FREE_DUTY_CYCLE = 0  # Zero duty cycle for minimum resistance
BRAKE_UPDATE_RATE = 0.01  # Update brake current every 10ms

def get_target_input(target_queue):
    while True:
        try:
            user_input = input("\nEnter target position (degrees), 'z' to set current as zero, 'r' to return to zero, 'l' to set lock position, or 'q' to quit: ").strip().lower()
        except Exception as e:
            print(f"Error reading input: {e}")
            
            if user_input == 'q':
                target_queue.put('quit')
                break
            elif user_input == 'z':
                target_queue.put('set_zero')
            elif user_input == 'r':
                target_queue.put(0)  # Return to zero position
            elif user_input == 'l':
                lock_pos_input = input("Enter lock position (degrees): ").strip()
                try:
                    lock_pos = float(lock_pos_input)
                    target_queue.put(('set_lock', lock_pos))
                except ValueError:
                    print("Invalid lock position. Please enter a number.")
            else:
                try:
                    target = float(user_input)
                    target_queue.put(('set_target', target))
                except ValueError:
                    print("Invalid target position. Please enter a number or 'z', 'r', 'l', 'q'")

def normalize_angle(angle):
    """Normalize angle to range -180 to 180 degrees"""
    angle = (angle + 180) % 360 - 180
    return angle

def angle_diff(a, b):
    """Calculate the shortest signed difference between two angles in degrees."""
    diff = (a - b + 180) % 360 - 180
    return diff

def main():
    target_queue = queue.Queue()
    current_target = None
    lock_position = None
    motor_locked = False
    position_tolerance = 1.0  # degrees
    
    # Start input thread
    input_thread = threading.Thread(target=get_target_input, args=(target_queue,), daemon=True)
    input_thread.start()
    
    try:
        with TMotorManager_servo_can(motor_type='AK80-9', motor_ID=104) as dev:
            print("/n Starting in FREE MODE. Enter a position (in degrees) to lock at.")
            dev.set_zero_position()
            # Initialize in duty cycle mode with zero resistance
            dev.enter_duty_cycle_control()
            dev.set_duty_cycle_percent(FREE_DUTY_CYCLE)
            dev.update()
            time.sleep(0.5)
            
            while True:
                # Get current position
                current_pos = dev.position * RAD_TO_DEG
                
                # Normalize current position
                current_pos = normalize_angle(current_pos)
                
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
                    if new_target == 'quit':
                        break
                    elif new_target == 'set_zero':
                        dev.set_zero_position()
                        print("\nCurrent position set as new zero")
                        current_target = None
                        lock_position = None
                        motor_locked = False
                        dev.enter_duty_cycle_control()
                        dev.set_duty_cycle_percent(FREE_DUTY_CYCLE)
                        dev.update()
                    elif isinstance(new_target, tuple):
                        command, value = new_target
                        if command == 'set_target':
                            current_target = value
                            print(f"\nNew target set: {current_target}°")
                            print("Motor will move to target.")
                            motor_locked = False
                        elif command == 'set_lock':
                            lock_position = value
                            print(f"\nLock position set: {lock_position}°")
                            motor_locked = False
                except queue.Empty:
                    pass
                
                # Check if we should lock the motor
                if current_target is not None and lock_position is not None and not motor_locked:
                    diff = angle_diff(current_pos, current_target)
                    at_target = abs(diff) <= position_tolerance
                    
                    if at_target:
                        print(f"\nPosition {current_target}° reached! Setting lock position...")
                        current_target = lock_position  # Change target to lock position
                    
                    diff_lock = angle_diff(current_pos, lock_position)
                    at_lock = abs(diff_lock) <= position_tolerance
                    
                    if at_lock:
                        print(f"\nPosition {lock_position}° reached! Locking motor...")
                        dev.enter_current_brake_control()
                        dev.set_motor_current_qaxis_amps(lock_current)
                        dev.update()
                        motor_locked = True
                        print("Motor LOCKED. Enter a new position to set new lock target.")
                
                # Status display
                print(f"\rCurrent: {current_pos:>6.2f}° | Target: {current_target if current_target is not None else 'None':>6} | Lock: {lock_position if lock_position is not None else 'None':>6} | Status: {'LOCKED' if motor_locked else 'FREE'}", end='')
                
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


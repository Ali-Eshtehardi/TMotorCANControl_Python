from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
from sys import path
path.append("/home/pi/TMotorCANControl/src/")
from TMotorCANControl.servo_can import TMotorManager_servo_can
import time
import threading
import queue
import math


DEG_TO_RAD = math.pi / 180.0  # Convert degrees to radians
RAD_TO_DEG = 180.0 / math.pi  # Convert radians to degrees
FREE_DUTY_CYCLE = 0  # Zero duty cycle for minimum resistance
POSITION_TOLERANCE = 1.0  # degrees

def Questions(target_queue):
    while True:
        try:
            user_input = input("\n Enter Target Position in degrees - Press Z to cancel - ")
        except Exception as e:
            print(f"\n Error reading input: {e}")

        if user_input == 'z':
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
            dev.set_zero_position()
            time.sleep(1.5)
            loop = SoftRealtimeLoop(dt=0.01, report=True, fade=0.0)
            dev.enter_position_velocity_control()
            
            for t in loop:             
                max_velocity = 1.0
                max_acceleration = 0.5
                
                # Check for new target from input thread
                try:
                    new_target = target_queue.get_nowait()
                    if new_target == 'cancel':
                        break  # Exit loop if user cancels
                    else:
                        target = new_target
                        motor_locked = False # Unlock when new target is set
                        print(f"\nNew target set: {target}°")
                except queue.Empty:
                    pass

                # Control motor to target position
                if target is not None and not motor_locked:
                    dev.set_output_angle_radians(target * math.pi / 180.0, max_velocity, max_acceleration)
                    dev.update()
                    print("\r" + str(dev), end='')

                    # Check if motor reached target position
                    current_pos = dev.position * 180 / math.pi
                    angle_difference = abs((current_pos - target))
                    if angle_difference <= POSITION_TOLERANCE:
                        print("\nTarget reached! Locking motor...")
                        dev.enter_current_brake_control()
                        dev.set_motor_current_qaxis_amps(50)  # Set appropriate brake current
                        dev.update()
                        motor_locked = True
                        print("\nMotor LOCKED. Enter a new target position.\n")
                elif motor_locked:
                    dev.update() # Keep brake current active
                    print("\rMotor LOCKED. Enter a new target position.", end='\n')

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
            


            # write a code thats alaways looping thats waiting for a target angle , like type in a position, 
            # like not like running again, but loop listening to a target position, that goes to the desirted positon from keyboard, once it 
            # goes to that angle , lets say 200 , and then give another angle command that is 220 , once it reached the 220 , lock , 
          


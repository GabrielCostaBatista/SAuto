#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import os
import signal
import threading
import math

# Import the AlphaBot driver
try:
    from alphabot_driver.AlphaBot import AlphaBot
    from alphabot_driver.PCA9685 import PCA9685
except ImportError:
    from AlphaBot import AlphaBot
    from PCA9685 import PCA9685

# Define key mappings
key_mapping = {
    'w': 'forward',
    's': 'backward',
    'a': 'turnleft',
    'd': 'turnright',
    'x': 'stop',
    '\x1b[A': 'camera_up',     # Up arrow - Camera tilt up
    '\x1b[B': 'camera_down',   # Down arrow - Camera tilt down
    '\x1b[D': 'camera_left',   # Left arrow - Camera pan left
    '\x1b[C': 'camera_right',   # Right arrow - Camera pan right
    't': 'forward_step',
    'g': 'turnback_180',
    'f': 'turnleft_90',
    'h': 'turnright_90',
    "1": '1',
    "2": '2',
    "3": '3',
    "4": '4',
    "5": '5',
    "6": '6'
}

# Speed settings
SPEED_INCREMENT = 0.2
MAX_SPEED = 1.0
MAX_PWM = 100.0  # Maximum PWM value (percentage)

# Camera servo settings
SERVO_STEP = 5     # Step size for servo movement
MIN_PULSE = 500    # Minimum servo pulse
MAX_PULSE = 2500   # Maximum servo pulse

MIN_PWM = 0.4*MAX_PWM  # Minimum PWM value (percentage)
TIME_FORWARD    = 1.0  # Time to move forward (in seconds)
TURN_TIME_90    = 1.2  # Time to turn 90 degrees (in seconds)    

def turnleft_1():
    TURN_TIME_90 = 0.5 + 0.1*1  # Time to turn 90 degrees (in seconds)
    rospy.loginfo("0.6")
def turnleft_2():
    TURN_TIME_90 = 0.5 + 0.1*2  # Time to turn 90 degrees (in seconds)
    rospy.loginfo("0.7")
def turnleft_3():
    TURN_TIME_90 = 0.5 + 0.1*3  # Time to turn 90 degrees (in seconds)
    rospy.loginfo("0.8")
def turnleft_4():
    TURN_TIME_90 = 0.5 + 0.1*1  # Time to turn 90 degrees (in seconds)
    rospy.loginfo("0.9")
def turnleft_5():
    TURN_TIME_90 = 0.5 + 0.1*2  # Time to turn 90 degrees (in seconds)
    rospy.loginfo("1.0")
def turnleft_6():
    TURN_TIME_90 = 0.5 + 0.1*1  # Time to turn 90 degrees (in seconds)    
    rospy.loginfo("1.1")
    

def forward_step(): # press "y"
    """Turn the robot right."""
    rospy.loginfo("Move 1 step forward")
    rospy.sleep(1.0)
    Ab.setPWMA(MIN_PWM)
    Ab.setPWMB(MIN_PWM*0.3)
    Ab.forward()
    rospy.sleep(TIME_FORWARD)
    Ab.stop()
    rospy.sleep(1.0)
def turnleft_90(): # press "f"
    """Turn the robot left."""
    rospy.loginfo("turn_left")
    rospy.sleep(1.0)
    Ab.setPWMA(MAX_PWM)
    Ab.setPWMB(MAX_PWM)
    Ab.setMotor(30,-30)
    rospy.sleep(1.0)
    Ab.forward()
    #Ab.left()
    #rospy.sleep(TURN_TIME_90)
    Ab.stop()
    rospy.sleep(1.0)
    
def turnright_90(): # press "h"
    """Turn the robot right."""
    rospy.loginfo("turn_right")
    rospy.sleep(1.0)
    Ab.setPWMA(MAX_PWM)
    Ab.setPWMB(MAX_PWM)
    Ab.right()
    rospy.sleep(TURN_TIME_90)
    Ab.stop()
    rospy.sleep(1.0)
def turnback_180(): # press "g"
    """Turn the robot 180 degrees."""
    rospy.loginfo("turn_back")
    rospy.sleep(1.0)
    Ab.setPWMA(MAX_PWM)
    Ab.setPWMB(MAX_PWM)
    Ab.left()
    rospy.sleep(2.0*TURN_TIME_90)  # Adjust time for 180 turn
    Ab.stop()
    rospy.sleep(1.0)
def stop_robot():
    """Stop the robot."""
    Ab.stop()
    rospy.loginfo("Stopping robot")
    
    
def get_key():
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    # Handle arrow keys (escape sequences)
    if key == '\x1b':
        # Read the next two characters
        seq = sys.stdin.read(2)
        if seq[0] == '[':
            # Arrow keys are represented as escape sequences: \x1b[A, \x1b[B, \x1b[C, \x1b[D
            key = key + seq
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def clean_shutdown(signum=None, frame=None):
    rospy.loginfo("Shutting down...")
    if 'Ab' in globals():
        Ab.stop()  # Stop the motors
    if 't' in globals():
        t.cancel()  # Stop the servo timer thread
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    rospy.signal_shutdown("User requested shutdown")
    sys.exit(0)

def servo_timer_func():
    global HPulse, VPulse, HStep, VStep, pwm
    
    if(HStep != 0):
        HPulse += HStep
        if(HPulse >= MAX_PULSE): 
            HPulse = MAX_PULSE
        if(HPulse <= MIN_PULSE):
            HPulse = MIN_PULSE
        #set channel 0, the Horizontal servo (pan)
        pwm.setServoPulse(0, HPulse)    
        
    if(VStep != 0):
        VPulse += VStep
        if(VPulse >= MAX_PULSE): 
            VPulse = MAX_PULSE
        if(VPulse <= MIN_PULSE):
            VPulse = MIN_PULSE
        #set channel 1, the vertical servo (tilt)
        pwm.setServoPulse(1, VPulse)   
    
    global t
    t = threading.Timer(0.01, servo_timer_func)
    t.daemon = True
    t.start()

def main():
    rospy.init_node('keyboard_motor_controller')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)

    # Initialize the AlphaBot hardware controller
    global Ab, settings, pwm, HPulse, VPulse, HStep, VStep, t
    Ab = AlphaBot()
    
    # Initialize PWM for servos
    pwm = PCA9685(0x40)
    pwm.setPWMFreq(50)
    
    # Set servo parameters
    HPulse = 1500  # Sets the initial Pulse for horizontal (pan)
    HStep = 0      # Sets the initial step length
    VPulse = 1500  # Sets the initial Pulse for vertical (tilt)
    VStep = 0      # Sets the initial step length
    
    # Initialize servos
    pwm.setServoPulse(0, HPulse)  # Pan servo
    pwm.setServoPulse(1, VPulse)  # Tilt servo
    
    # Set up signal handlers for clean shutdown
    signal.signal(signal.SIGINT, clean_shutdown)
    signal.signal(signal.SIGTERM, clean_shutdown)
    
    settings = termios.tcgetattr(sys.stdin)
    
    # Track current velocity
    current_linear_velocity = 0.0
    current_angular_velocity = 0.0
    
    # Start servo control thread
    t = threading.Timer(0.01, servo_timer_func)
    t.daemon = True
    t.start()
    
    rospy.loginfo("Keyboard motor controller node started.")
    rospy.loginfo("Controls: WASD to move, X to stop, Arrow keys for camera control.")

    try:
        while not rospy.is_shutdown():
            key = get_key()
            if key in key_mapping:
                command = key_mapping[key]
                
                # Handle progressive velocity changes
                if command == 'forward':
                    current_angular_velocity = 0.0
                    # If going backward, first stop
                    if current_linear_velocity < 0:
                        current_linear_velocity = 0.0
                        Ab.stop()
                        rospy.loginfo("Stopping before changing direction")
                    # Otherwise increase speed (with limit)
                    else:
                        # Start with minimum speed if currently stopped
                        current_linear_velocity = min(current_linear_velocity + SPEED_INCREMENT, MAX_SPEED)
                        
                        # Convert linear velocity to PWM (0.0-1.0 -> 0-100%)
                        pwm_value = abs(current_linear_velocity) * MAX_PWM
                        Ab.setPWMA(pwm_value)
                        Ab.setPWMB(pwm_value)
                        Ab.forward()
                        rospy.loginfo(f"PWM set to: {pwm_value:.1f}%")
                
                elif command == 'backward':
                    current_angular_velocity = 0.0
                    # If going forward, first stop
                    if current_linear_velocity > 0:
                        current_linear_velocity = 0.0
                        Ab.stop()
                        rospy.loginfo("Stopping before changing direction")
                    # Otherwise increase speed (with limit) 
                    else:
                        # Start with minimum speed if currently stopped
                        current_linear_velocity = max(current_linear_velocity - SPEED_INCREMENT, -MAX_SPEED)
                        
                        # Convert linear velocity to PWM (0.0-1.0 -> 0-100%)
                        pwm_value = abs(current_linear_velocity) * MAX_PWM
                        Ab.setPWMA(pwm_value)
                        Ab.setPWMB(pwm_value)
                        Ab.backward()
                        rospy.loginfo(f"PWM set to: {pwm_value:.1f}%")
                
                elif command == 'turnleft':
                    current_linear_velocity = 0.0
                    # If turning right, first stop
                    if current_angular_velocity < 0:
                        current_angular_velocity = 0.0
                        Ab.stop()
                        rospy.loginfo("Stopping rotation before changing direction")
                    # Otherwise increase angular speed (with limit)
                    else:
                        # Start with minimum speed if currently stopped
                        current_angular_velocity = min(current_angular_velocity + SPEED_INCREMENT, MAX_SPEED)
                        
                        # Convert to PWM value
                        pwm_value = abs(current_angular_velocity) * MAX_PWM
                        Ab.setPWMA(pwm_value)
                        Ab.setPWMB(pwm_value)
                        Ab.left()
                        rospy.loginfo(f"Turn PWM set to: {pwm_value:.1f}%")
                
                elif command == 'turnright':
                    current_linear_velocity = 0.0
                    # If turning left, first stop
                    if current_angular_velocity > 0:
                        current_angular_velocity = 0.0
                        Ab.stop()
                        rospy.loginfo("Stopping rotation before changing direction")
                    # Otherwise increase angular speed (with limit)
                    else:
                        # Start with minimum speed if currently stopped
                        current_angular_velocity = max(current_angular_velocity - SPEED_INCREMENT, -MAX_SPEED)
                        
                        # Convert to PWM value
                        pwm_value = abs(current_angular_velocity) * MAX_PWM
                        Ab.setPWMA(pwm_value)
                        Ab.setPWMB(pwm_value)
                        Ab.right()
                        rospy.loginfo(f"Turn PWM set to: {pwm_value:.1f}%")
                
                elif command == 'stop':
                    current_linear_velocity = 0.0
                    current_angular_velocity = 0.0
                    Ab.stop()
                
                # Camera controls
                elif command == 'camera_up':
                    # If moving down, first stop
                    if VStep > 0:
                        VStep = 0
                        rospy.loginfo("Stopping camera tilt before changing direction")
                    else:
                        VStep = -SERVO_STEP  # Negative to move up (servo orientation)
                        rospy.loginfo("Camera tilt up")
                
                elif command == 'camera_down':
                    # If moving up, first stop
                    if VStep < 0:
                        VStep = 0
                        rospy.loginfo("Stopping camera tilt before changing direction")
                    else:
                        VStep = SERVO_STEP   # Positive to move down (servo orientation)
                        rospy.loginfo("Camera tilt down")
                
                elif command == 'camera_left':
                    # If moving right, first stop
                    if HStep < 0:
                        HStep = 0
                        rospy.loginfo("Stopping camera pan before changing direction")
                    else:
                        HStep = SERVO_STEP   # Pan left
                        rospy.loginfo("Camera pan left")
                
                elif command == 'camera_right':
                    # If moving left, first stop
                    if HStep > 0:
                        HStep = 0
                        rospy.loginfo("Stopping camera pan before changing direction")
                    else:
                        HStep = -SERVO_STEP  # Pan right
                        rospy.loginfo("Camera pan right")
                elif command == 'turnright_90':
                    turnright_90()
                elif command == 'turnleft_90':
                    turnleft_90()
                elif command == 'forward_step':
                    forward_step()
                elif command == 'turnback_180':
                    turnback_180()
                elif command == '1':
                    turnleft_1()
                elif command == '2':
                    turnleft_2()
                elif command == '3':
                    turnleft_3()
                elif command == '4':
                    turnleft_4()
                elif command == '5':
                    turnleft_5()
                elif command == '6':
                    turnleft_6()
                
                # Create and publish Twist message
                twist = Twist()
                twist.linear.x = current_linear_velocity
                twist.angular.z = current_angular_velocity
                pub.publish(twist)
                
                rospy.loginfo(f"Command: {command}, Linear: {current_linear_velocity:.1f}, Angular: {current_angular_velocity:.1f}")
            elif key == '\x03':  # Ctrl+C
                clean_shutdown()
            rate.sleep()
    except Exception as e:
        rospy.logerr(f"Error: {e}")
    finally:
        twist = Twist()
        pub.publish(twist)  # Stop the robot
        if 'Ab' in globals():
            Ab.stop()  # Ensure motors are stopped
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()
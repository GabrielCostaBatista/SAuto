#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import os
import signal

# Import the AlphaBot driver
try:
    from alphabot_driver.AlphaBot import AlphaBot
except ImportError:
    from AlphaBot import AlphaBot

# Define key mappings
key_mapping = {
    'w': 'forward',
    's': 'backward',
    'a': 'turnleft',
    'd': 'turnright',
    'x': 'stop'
}

# Speed settings
SPEED_INCREMENT = 0.2
MAX_SPEED = 1.0
MAX_PWM = 100.0  # Maximum PWM value (percentage)

def get_key():
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def clean_shutdown(signum=None, frame=None):
    rospy.loginfo("Shutting down...")
    if 'Ab' in globals():
        Ab.stop()  # Stop the motors
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    rospy.signal_shutdown("User requested shutdown")
    sys.exit(0)

def main():
    rospy.init_node('keyboard_motor_controller')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)

    # Initialize the AlphaBot hardware controller
    global Ab, settings
    Ab = AlphaBot()
    
    # Set up signal handlers for clean shutdown
    signal.signal(signal.SIGINT, clean_shutdown)
    signal.signal(signal.SIGTERM, clean_shutdown)
    
    settings = termios.tcgetattr(sys.stdin)
    
    # Track current velocity
    current_linear_velocity = 0.0
    current_angular_velocity = 0.0
    
    rospy.loginfo("Keyboard motor controller node started. Use WASD to move, X to stop.")

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
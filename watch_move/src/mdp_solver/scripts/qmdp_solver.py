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
from POMDP_simple_solver import POMDP
import numpy as np

maze = np.array([
    [0,0,2,0,1,0,0,0,0],
    [1,1,1,0,1,2,1,1,1],
    [0,0,0,0,1,0,1,0,0],
    [0,1,0,1,1,0,1,1,2],
    [0,1,2,0,1,0,0,1,0],
    [0,0,1,0,0,0,0,0,0],
], dtype=int)

n_rows, n_cols = maze.shape
start = (0, 0)                # In this case, the robot starts at the top left corner
goal  = (0, n_cols-1)         # In this case, the robot stops at the top right corner

# Create a POMDP object
pomdp = POMDP(maze, start, goal,
              gamma=0.95, r_step=-1, r_goal=1000, slip=0.2)

pomdp.solve_mdp(tol=1e-4)

b = np.zeros(pomdp.S)
b[pomdp.state_index[start]] = 1.0

CELL_SIZE       = 0.25      # metres per grid‐cell
LINEAR_SPEED    = 0.2      # m/s   → tune so CELL_TIME = CELL_SIZE/LINEAR_SPEED
ANGULAR_SPEED   = 1 #math.pi/2 # rad/s → tune so TURN_TIME_90 = (π/2)/ANGULAR_SPEED

CELL_TIME    = CELL_SIZE / LINEAR_SPEED
TURN_TIME_90 = (math.pi/2) / 

# Import the AlphaBot driver
try:
    from alphabot_driver.AlphaBot import AlphaBot
    from alphabot_driver.PCA9685 import PCA9685
except ImportError:
    from AlphaBot import AlphaBot
    from PCA9685 import PCA9685


def clean_shutdown(signum=None, frame=None):
    rospy.loginfo("Shutting down...")
    if 'Ab' in globals():
        Ab.stop()  # Stop the motors
    if 't' in globals():
        t.cancel()  # Stop the servo timer thread
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    rospy.signal_shutdown("User requested shutdown")
    sys.exit(0)

def main():
    rospy.init_node('pomdp_move')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)

    # Initialize the AlphaBot hardware controller
    global Ab, settings, pwm, HPulse, VPulse, HStep, VStep, t
    Ab = AlphaBot()

    # Initialize PWM for servos
    pwm = PCA9685(0x40)
    pwm.setPWMFreq(50)

    # Set up signal handlers for clean shutdown
    signal.signal(signal.SIGINT, clean_shutdown)
    signal.signal(signal.SIGTERM, clean_shutdown)
    
    settings = termios.tcgetattr(sys.stdin)


    rospy.loginfo("Initializing AlphaBot...")
    try:
        while not rospy.is_shutdown():
            MOTOR_PWM = 20  # wheel PWM 0–100%; tune as needed
            heading = 0     # 0=E, 1=N, 2=W, 3=S
            r0, c0 = start

            for i in range(100000):

                
                # Get the next state from the POMDP solver
                # 1) pick action by QMDP policy
                a_idx = pomdp.qmdp_action(b)

                # 2) simulate true next state
                probs = pomdp.T[a_idx, pomdp.state_index[true]]
                true = pomdp.states[np.random.choice(pomdp.S, p=probs)]
                r, c = true
                dr, dc = r0 - r, c0 - c
                r0, c0 = r, c
                rospy.loginfo("Current cell: (%d,%d)", r, c)

                # determine desired heading
                if   (dr, dc) == ( 0,  1): desired = 0  # east
                elif (dr, dc) == (-1,  0): desired = 1  # north
                elif (dr, dc) == ( 0, -1): desired = 2  # west
                elif (dr, dc) == ( 1,  0): desired = 3  # south
                else:
                    rospy.logwarn("Invalid step from (%d,%d) to (%d,%d)", r, c, nr, nc)
                    continue

                # rotate via the shortest sequence of 90° turns
                diff = (desired - heading) % 4
                if diff == 1:
                    Ab.setPWMA(MOTOR_PWM); Ab.setPWMB(MOTOR_PWM)
                    Ab.left();   rospy.sleep(TURN_TIME_90); Ab.stop()
                elif diff == 2:
                    Ab.setPWMA(MOTOR_PWM); Ab.setPWMB(MOTOR_PWM)
                    Ab.left();   rospy.sleep(TURN_TIME_90)
                    Ab.left();   rospy.sleep(TURN_TIME_90); Ab.stop()
                elif diff == 3:
                    Ab.setPWMA(MOTOR_PWM); Ab.setPWMB(MOTOR_PWM)
                    Ab.right();  rospy.sleep(TURN_TIME_90); Ab.stop()
                # else diff==0 → already facing correct way

                heading = desired

                # drive forward one cell
                Ab.setPWMA(MOTOR_PWM); Ab.setPWMB(MOTOR_PWM)
                Ab.forward(); rospy.sleep(CELL_TIME); Ab.stop()
                rospy.loginfo("Moved to cell (%d,%d)", r0, c0)

                rospy.loginfo("Observing...: ")

            rospy.loginfo("Reached goal. Path complete.")
            # exit the while‐loop
            break

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